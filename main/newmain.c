#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_crt_bundle.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"      // 👈 核心修复：添加 GPIO 驱动支持
#include "esp_http_client.h"
#include "esp_pm.h"
#include "cJSON.h"

// minimp3 解码器
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

// ===================== 1. 核心配置 =====================
#define WIFI_SSID "LZYPura70Pro"
#define WIFI_PASS "1234567890"

#define ALIYUN_TTS_APPKEY  "r14vtjJVuThxUIPK" 
#define ALIYUN_TTS_TOKEN   "6e00871619ff487f81dbfa3a9ad6d89b" 
#define TTS_URL            "https://nls-gateway-cn-shanghai.aliyuncs.com/stream/v1/tts"

// 硬件引脚
#define I2S_DOUT_IO  GPIO_NUM_47
#define I2S_BCLK_IO  GPIO_NUM_21
#define I2S_LRC_IO   GPIO_NUM_14
#define BUTTON_GPIO  GPIO_NUM_0   // BOOT 按键

static const char *TAG = "ALIYUN_TTS";
static bool wifi_connected = false;

i2s_chan_handle_t tx_chan = NULL;
mp3dec_t mp3d;
static short pcm_buffer[MINIMP3_MAX_SAMPLES_PER_FRAME]; 
uint8_t mp3_res_buffer[4096];
int mp3_res_len = 0;

// ==================== 2. 基础驱动初始化 ====================
static void init_i2s_driver() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {.bclk = I2S_BCLK_IO, .ws = I2S_LRC_IO, .dout = I2S_DOUT_IO},
    };
    i2s_channel_init_std_mode(tx_chan, &std_cfg);
    i2s_channel_enable(tx_chan);
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connected = true;
        ESP_LOGI(TAG, "✅ WiFi 已连接");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        esp_wifi_connect();
    }
}

static void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);
    
    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // 强制指定加密方式，增加兼容性
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    
    ESP_LOGI(TAG, "正在启动 WiFi...");
    esp_wifi_start();
    
    // --- 关键修正：手动触发第一次连接 ---
    esp_wifi_connect(); 
    
    esp_wifi_set_ps(WIFI_PS_NONE); 
}


// ==================== 修正后的 HTTP 回调 ====================
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (evt->data_len > 0) {
                // 打印一下，看看有没有数据进来
                // ESP_LOGD(TAG, "收到数据长度: %d", evt->data_len); 
                
                if (mp3_res_len + evt->data_len > sizeof(mp3_res_buffer)) {
                    mp3_res_len = 0; // 防止溢出
                }
                memcpy(mp3_res_buffer + mp3_res_len, evt->data, evt->data_len);
                mp3_res_len += evt->data_len;

                int offset = 0;
                while (mp3_res_len >= 128) {
                    mp3dec_frame_info_t info;
                    int samples = mp3dec_decode_frame(&mp3d, mp3_res_buffer + offset, mp3_res_len, pcm_buffer, &info);
                    if (samples > 0) {
                        size_t written;
                        // 关键：将解码后的 PCM 写入 I2S
                        i2s_channel_write(tx_chan, pcm_buffer, samples * info.channels * sizeof(short), &written, portMAX_DELAY);
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
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "音频流下载完成");
            break;
        default:
            break;
    }
    return ESP_OK;
}

// ==================== 修正后的 TTS 任务 ====================
void aliyun_tts_task(void *pvParameters) {
    //enter_tts_mode(); 
    ESP_LOGI(TAG, ">>> 启动阿里云 TTS (纯净 Body 模式)");

    mp3dec_init(&mp3d);
    mp3_res_len = 0;

    // if (pcm_out_ptr == NULL) {
    //     pcm_out_ptr = heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(short) * 2, MALLOC_CAP_SPIRAM);
    //     if (!pcm_out_ptr) pcm_out_ptr = malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(short) * 2);
    // }

    // 1. 【注意】JSON 里必须包含 appkey
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "appkey", ALIYUN_TTS_APPKEY); 
    cJSON_AddStringToObject(root, "text", "老哥，这次我把 URL 里的尾巴砍掉了，直接走 Body 传参，必通！");
    cJSON_AddStringToObject(root, "voice", "zhiyuan"); 
    cJSON_AddStringToObject(root, "format", "mp3");
    cJSON_AddNumberToObject(root, "sample_rate", 16000);
    cJSON_AddNumberToObject(root, "volume", 50);
    char *json_body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    // 2. 【注意】这里绝对不要用 snprintf 拼接！直接用原始定义的 TTS_URL
    // 确保你的 TTS_URL 定义是 "https://nls-gateway-cn-shanghai.aliyuncs.com/stream/v1/tts"
    esp_http_client_config_t config = {
        .url = TTS_URL,               // 👈 重点：不要写成 full_url，就用这个纯净的 URL
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 20000,
        .buffer_size = 8192,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    // 3. Header 只传 Token
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "X-NLS-Token", ALIYUN_TTS_TOKEN); 
    
    // 4. 把含有 appkey 的 JSON 塞进去
    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    // 打印一下当前的 URL 确认有没有尾巴
    ESP_LOGW(TAG, "📡 正在请求 (当前 URL: %s)", TTS_URL); 
    
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "<<< 请求完成，状态码: %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "<<< 传输失败: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(json_body);
    //exit_tts_mode(); 
    vTaskDelete(NULL);
}






// ==================== 4. 按键监听任务 ====================
void button_task(void *pvParameter) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0) { // 检测按下
            ESP_LOGW(TAG, "🚀 按键触发！");
            xTaskCreate(aliyun_tts_task, "aliyun_tts", 1024 * 16, NULL, 5, NULL);
            while(gpio_get_level(BUTTON_GPIO) == 0) vTaskDelay(pdMS_TO_TICKS(10)); // 等待松开
            vTaskDelay(pdMS_TO_TICKS(500)); // 消抖
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    nvs_flash_init();
    wifi_init();
    init_i2s_driver();
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
}