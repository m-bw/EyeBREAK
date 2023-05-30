idf_component_register(SRCS "main.cpp" "morse.cpp" "ble.cpp"
                       PRIV_REQUIRES fatfs esp_timer esp_http_server esp_wifi nvs_flash tflite-lib opencv)

# Create a FAT filesystem image from the contents of data/ subdirectory,
# The image will be flashed into the 'storage' partition when 'idf.py flash' is used.
#fatfs_create_spiflash_image(storage data FLASH_IN_PROJECT)
