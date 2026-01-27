#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "esp_crt_bundle.h"

#include "my_private_key.h"

// I2S 驱动头文件
#include "driver/i2s_std.h"

// 摄像头与网络
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"

// TTS 相关头文件
#include "esp_tts.h"
#include "esp_tts_voice_xiaole.h" 
#include "esp_partition.h"
#include "esp_psram.h"

// 引脚定义 (包含摄像头)
#define BOARD_ESP32S3_WROOM
#include "camera_pinout.h"


#include "cJSON.h"  // 必须包含这个头文件

// MP3 解码库
#define MINIMP3_IMPLEMENTATION
// 使用 minimp3 单文件实现：把 https://github.com/lieff/minimp3 的 minimp3.h 放到 components 里
// 或者直接在 main/ 目录下放置 minimp3.h，并确保 CMakeLists.txt 里包含 main/
#include "minimp3.h"

mp3dec_t mp3d;

mp3dec_frame_info_t info;
short pcm_buffer[MINIMP3_MAX_SAMPLES_PER_FRAME]; // 解码后的临时存放区

// ===================== 配置参数 (补回丢失的定义) =====================
#define WIFI_SSID "LZYPura70Pro"
#define WIFI_PASS "1234567890"
#define MQTT_BROKER_URI "mqtt://192.168.43.13:1883"
#define MQTT_TOPIC_IMG  "esp32s3/camera/lzy"
#define MQTT_TOPIC_OTA  "esp32s3/ota/command"
#define OTA_URL         "http://192.168.43.13:8070/new_firmware.bin"

// 修正 URL，去掉末尾可能存在的干扰
#define TTS_URL "https://nls-gateway-cn-shanghai.aliyuncs.com/stream/v1/tts"




#define BOARD_BUTTON_GPIO 0

// I2S 硬件引脚 (老哥接的线)
#define I2S_DOUT_IO    GPIO_NUM_47  // DIN
#define I2S_BCLK_IO    GPIO_NUM_21  // BCLK
#define I2S_LRC_IO     GPIO_NUM_14  // LRC

// ===================== 全局变量 =====================
static const char *TAG = "SMART_CAM_TTS";
static esp_mqtt_client_handle_t mqtt_client;
static bool mqtt_connected = false;
static const void *g_tts_mmap_ptr = NULL; 
static bool is_tts_running = false;       
static i2s_chan_handle_t tx_chan = NULL; 

// 错误信息缓冲区
static char error_response[512] = {0};
static int error_len = 0;

// 用于处理回调中的残帧（防止 MP3 帧被切断导致无法解码）
static uint8_t mp3_res_buffer[4096];
static int mp3_res_len = 0;
static short *pcm_out_ptr = NULL;



// ===================== 摄像头配置 =====================
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 12,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// ===================== I2S 初始化 (修正报错点) =====================
static void init_i2s_driver() {
    ESP_LOGI(TAG, "==== 初始化 I2S 驱动 ====");
    // 修正报错：改为 I2S_CHANNEL_DEFAULT_CONFIG
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &tx_chan, NULL);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_IO,
            .ws   = I2S_LRC_IO,
            .dout = I2S_DOUT_IO,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    // 修正报错：改为 i2s_channel_init_std_mode
    i2s_channel_init_std_mode(tx_chan, &std_cfg);
    //i2s_channel_enable(tx_chan);
    ESP_LOGI(TAG, "✅ I2S 驱动初始化成功，（初始化为禁用状态）");
}

// 进入 TTS 模式：画质降低，帧率让路
void enter_tts_mode() {
    is_tts_running = true;
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // 质量数值越大，图片体积越小。从 12 改到 30，图片体积能缩减 60% 以上
        s->set_quality(s, 30); 
    }
    ESP_LOGW(TAG, ">>> 进入 TTS 降级模式：画质已压缩，为 HTTPS 让路");
}

// 退出 TTS 模式：恢复满血
void exit_tts_mode() {
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_quality(s, 12); // 恢复高质量
    }
    is_tts_running = false;
    ESP_LOGI(TAG, "<<< 退出 TTS 模式：恢复满血画质");
}




//                HTTP事件回调             //

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA: {
            int status = esp_http_client_get_status_code(evt->client);
            
            if (status == 200 && evt->data_len > 0 && pcm_out_ptr != NULL) {
                // ========== 正常情况：解码 MP3 ==========
                if (mp3_res_len + evt->data_len > sizeof(mp3_res_buffer)) {
                    mp3_res_len = 0; // 防止溢出
                }
                memcpy(mp3_res_buffer + mp3_res_len, evt->data, evt->data_len);
                mp3_res_len += evt->data_len;

                int offset = 0;
                while (mp3_res_len >= 128) {
                    int samples = mp3dec_decode_frame(&mp3d, mp3_res_buffer + offset, mp3_res_len, pcm_out_ptr, &info);
                    if (samples > 0) {
                        size_t written;
                        i2s_channel_write(tx_chan, pcm_out_ptr, samples * info.channels * sizeof(short), &written, portMAX_DELAY);
                        offset += info.frame_bytes;
                        mp3_res_len -= info.frame_bytes;
                    } else {
                        break;
                    }
                }
                if (mp3_res_len > 0 && offset > 0) {
                    memmove(mp3_res_buffer, mp3_res_buffer + offset, mp3_res_len);
                }
            } 
            else if (status != 200 && evt->data_len > 0) {
                // ========== 【新增】错误情况：捕获错误信息 ==========
                if (error_len + evt->data_len < sizeof(error_response) - 1) {
                    memcpy(error_response + error_len, evt->data, evt->data_len);
                    error_len += evt->data_len;
                    error_response[error_len] = '\0';
                }
            }
            break;
        }
        
        case HTTP_EVENT_ON_FINISH:
            // ========== 【新增】请求完成时打印错误详情 ==========
            if (error_len > 0) {
                ESP_LOGE(TAG, "🔴 阿里云完整错误响应: %s", error_response);
                // 重置缓冲区供下次使用
                error_len = 0;
                memset(error_response, 0, sizeof(error_response));
            }
            break;
            
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "❌ HTTP 网络传输错误");
            break;
            
        default: 
            break;
    }
    return ESP_OK;
}




// ===================== 阿里云在线 TTS 任务 =====================
void aliyun_tts_task(void *pvParameters) {
    enter_tts_mode(); // 降低摄像头画质，腾出带宽给 HTTPS
    ESP_LOGI(TAG, ">>> 启动阿里云 TTS (终极兼容模式)");

    // 每次开始播放前启用 I2S 通道
    i2s_channel_enable(tx_chan);

    // 初始化解码器
    mp3dec_init(&mp3d);
    mp3_res_len = 0;

    // 1. 确保 pcm_out_ptr 内存安全
    if (pcm_out_ptr == NULL) {
        // 大项目里优先用 PSRAM，防止栈溢出
        pcm_out_ptr = heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(short) * 2, MALLOC_CAP_SPIRAM);
        if (!pcm_out_ptr) pcm_out_ptr = malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(short) * 2);
    }

    // 2. 构造 JSON
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "appkey", ALIYUN_TTS_APPKEY);
    cJSON_AddStringToObject(root, "text", "哈喽靓仔，可唔可以同你拍拖");
    // cJSON_AddStringToObject(root, "voice", "zhiyuan"); // 改回你测试成功的发音人
    cJSON_AddStringToObject(root, "voice", "jiajia"); // 改回你测试成功的发音人
    cJSON_AddStringToObject(root, "format", "mp3");
    cJSON_AddNumberToObject(root, "sample_rate", 16000);
    cJSON_AddNumberToObject(root, "volume", 50);
    char *json_body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    // 3. 【核心修正】拼接 URL，把 appkey 拼在后面
    // char full_url[256];
    // snprintf(full_url, sizeof(full_url), "%s?appkey=%s", TTS_URL, ALIYUN_TTS_APPKEY);

    esp_http_client_config_t config = {
        .url = TTS_URL,              // 使用带参数的 URL
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,          // 增加到 15 秒
        .buffer_size = 8192,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    // 4. 设置关键 Header
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "X-NLS-Token", ALIYUN_TTS_TOKEN); // 必须是 X-NLS-Token
    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    ESP_LOGW(TAG, "📡 正在请求阿里云 (URL: %s)", TTS_URL);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "<<< 请求完成，状态码: %d", status);
    } else {
        ESP_LOGE(TAG, "<<< 请求失败: %s", esp_err_to_name(err));
    }

    // 在 aliyun_tts_task 函数末尾添加：

    // 1. 发送一段静音数据，把 I2S 硬件缓冲区里的残余“冲”出来
    size_t written;
    short *silence = calloc(1, 1024); // 全 0 数据
    if (silence) {
        for(int i=0; i<5; i++) { // 多写几次确保填满 DMA 缓冲
            i2s_channel_write(tx_chan, silence, 1024, &written, portMAX_DELAY);
        }
        free(silence);
    }

    // 2. 暂停 I2S 通道（这是解决突突突响的关键）
    i2s_channel_disable(tx_chan);

    esp_http_client_cleanup(client);
    free(json_body);
    exit_tts_mode(); // 恢复摄像头画质
    vTaskDelete(NULL);
}


// ===================== 按键监听任务 =====================
void button_monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "按键监听已就绪 (GPIO %d)", BOARD_BUTTON_GPIO);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOARD_BUTTON_GPIO),
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    while (1) {
        // 检测低电平 (按下)
        if (gpio_get_level(BOARD_BUTTON_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50)); // 消抖
            if (gpio_get_level(BOARD_BUTTON_GPIO) == 0) {
                
                // --- 标志位判断逻辑 ---
                if (!is_tts_running) {
                    ESP_LOGW(TAG, "检测到按键，触发语音任务...");
                    // 创建任务处理语音 (栈空间给 8KB)
                    // xTaskCreate(tts_pinyin_process_task, "tts_task", 1024 * 8, NULL, 3, NULL);
                    xTaskCreate(aliyun_tts_task, "aliyun_tts_task", 1024 * 20, NULL, 3, NULL);
                } else {
                    ESP_LOGE(TAG, "⚠️ 语音任务正在运行中，忽略此次按键");
                }

                // 等待按键释放，防止重复触发
                while (gpio_get_level(BOARD_BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ===================== OTA 任务 =====================
static void ota_update_task(void *pvParameter) {
    ESP_LOGI(TAG, ">>> HTTP OTA 更新启动");

    esp_http_client_config_t config = {
        .url = OTA_URL,
        .timeout_ms = 10000,
        .keep_alive_enable = true,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
    };

    esp_err_t ret = esp_https_ota(&(esp_https_ota_config_t){
        .http_config = &config,
        .partial_http_download = true,
    });

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA 成功，重启...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA 失败: 0x%x", ret);
    }
    vTaskDelete(NULL);
}

// ===================== MQTT 拍照发送 =====================
void camera_mqtt_stream_task(void *pvParameters) {
    while (1) {
        // 如果正在播语音，每 800ms 发一帧（1.2 FPS），保证画面不卡死
        // 如果没播语音，每 150ms 发一帧（6.6 FPS）
        int current_delay = is_tts_running ? 800 : 150;


        if (mqtt_connected) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGE(TAG, "摄像头捕捉失败");
            } else {
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_IMG, (const char*)fb->buf, fb->len, 0, 0);
                if (msg_id >= 0) ESP_LOGI(TAG, "图片上传成功, 大小: %zu 字节", fb->len);
                esp_camera_fb_return(fb);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(current_delay));
    }
}

// ===================== MQTT 事件 =====================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT 已连接");
            mqtt_connected = true;
            esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_OTA, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT 断开");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->data, "update", event->data_len) == 0) {
                ESP_LOGW(TAG, "收到 OTA 指令，启动更新");
                xTaskCreate(ota_update_task, "ota_task", 8192, NULL, 5, NULL);
            }
            break;
        default: break;
    }
}

// ===================== WiFi 事件 =====================
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        mqtt_connected = false;
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
        ESP_LOGI(TAG, "已获得 IP: " IPSTR, IP2STR(&event->ip_info.ip));

        esp_mqtt_client_config_t mqtt_cfg = {
            .broker.address.uri = MQTT_BROKER_URI,
        };
        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(mqtt_client);
    }
}

// ===================== app_main =====================
void app_main(void) {
    // 1. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

   // --- 【重点修改：日志强制刷出】 ---
    ESP_LOGW(TAG, "==== 系统启动：开始初始化 I2S ====");
    
    // 2. 第一时间初始化 I2S，不要让它等摄像头
    init_i2s_driver(); 
    
    // 强制把日志缓存刷到串口屏上，防止死机了看不见日志
    fflush(stdout);
    fsync(fileno(stdout));
    
    // 2. 初始化摄像头
    if (esp_camera_init(&camera_config) != ESP_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }
    ESP_LOGI(TAG, "摄像头初始化成功");


    mp3dec_init(&mp3d);

    // // 3. 【核心插入】：建立 TTS 模型 Flash 映射
    // const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x01, "voice_data");
    // if (part) {
    //     esp_partition_mmap_handle_t mmap_handle;
    //     ret = esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA, &g_tts_mmap_ptr, &mmap_handle);
    //     if (ret == ESP_OK) {
    //         ESP_LOGI(TAG, "✅ TTS 模型映射成功，虚拟地址: %p", g_tts_mmap_ptr);
    //     }
    // } else {
    //     ESP_LOGE(TAG, "❌ 未找到 voice_data 分区，请检查分区表！");
    // }

    // 4. 初始化网络 (WiFi 启动)
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE); // 👈 这一行在大项目里是救命的

    // 5. 启动功能任务
    xTaskCreate(camera_mqtt_stream_task, "mqtt_stream", 8192, NULL, 5, NULL);
    xTaskCreate(button_monitor_task, "btn_task", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "🚀 系统完全启动，等待按键触发语音测试...");
}