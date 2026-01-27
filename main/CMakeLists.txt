idf_component_register(
    SRCS "take_picture.c"  # 请确认你的主代码文件名确实是这个
    INCLUDE_DIRS "."
    REQUIRES 
        nvs_flash 
        esp_psram 
        esp_netif 
        esp_event 
        esp_wifi 
        mqtt 
        espressif__esp32-camera 
        esp_https_ota 
        esp_http_client  # 它会自动处理 esp-tls 和证书
        app_update 
        esp-sr           # 离线语音
        json             # cJSON
)

# --- 语音包烧录逻辑保持不变 ---
set(VOICE_DATA_FILE "${CMAKE_SOURCE_DIR}/esp_tts_voice_data_xiaole.dat")
if(EXISTS "${VOICE_DATA_FILE}")
    message(STATUS "语音模型准备就绪: ${VOICE_DATA_FILE}")
    set(voice_args "voice_data" "0x810000" "${VOICE_DATA_FILE}")
    set(ESPTOOLPY_WRITE_FLASH_ARGS "${ESPTOOLPY_WRITE_FLASH_ARGS}" "${voice_args}" PARENT_SCOPE)
else()
    message(WARNING "未找到离线语音模型文件")
endif()