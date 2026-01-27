#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_partition.h"
#include "esp_psram.h"
#include "esp_tts.h"
#include "esp_tts_voice_xiaole.h"
#include "driver/gpio.h"

static const char *TAG = "TTS_INTERNAL_CHECK";

void tts_check_task(void *arg) {
    void *mmap_ptr = arg; 
    // 测试文本：包含多音字，用来测试引擎是否聪明
    const char *text = "你好老哥，重庆的大桥重修了。"; 

    // 1. 初始化引擎
    esp_tts_voice_t *voice = esp_tts_voice_set_init(&esp_tts_voice_xiaole, mmap_ptr);
    esp_tts_handle_t tts = esp_tts_create(voice);

    if (tts == NULL) {
        ESP_LOGE(TAG, "❌ 引擎创建失败");
        vTaskDelete(NULL);
    }

    ESP_LOGW(TAG, "--- 准备触发引擎解析 ---");
    ESP_LOGI(TAG, "正在解析文本: %s", text);

    // 2. 核心：调用解析函数
    // 虽然我们没接 I2S 播放音频，但 parse_chinese 会启动前端处理
    // 在这个过程中，TTS 库通常会直接在串口打印出它识别到的拼音序列
    if (esp_tts_parse_chinese(tts, text)) {
        ESP_LOGW(TAG, "✅ 前端解析成功！请观察上方是否有引擎自动输出的拼音。");
        
        int len[1] = {0};
        // 尝试获取第一段音频流（不播放，只为了触发内部逻辑）
        short *pcm_data = esp_tts_stream_play(tts, len, 0);
        if (pcm_data) {
             ESP_LOGI(TAG, "成功生成第一帧音频数据，长度: %d 字节", len[0] * 2);
        }
    } else {
        ESP_LOGE(TAG, "❌ 解析失败，请检查模型映射地址是否正确。");
    }

    ESP_LOGW(TAG, "测试任务结束。");
    while(1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}

void app_main(void) {
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM 未初始化！");
        return;
    }

    // 映射逻辑（这一步你之前已经跑通了）
    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x01, "voice_data");
    const void *mmap_ptr = NULL;
    esp_partition_mmap_handle_t mmap_handle;
    esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA, &mmap_ptr, &mmap_handle);

    xTaskCreate(tts_check_task, "tts_check", 1024 * 6, (void*)mmap_ptr, 5, NULL);
}