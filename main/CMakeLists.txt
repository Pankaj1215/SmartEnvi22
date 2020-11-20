set(COMPONENT_SRCS "main.c" "app.c" "led.c" "heater.c" "button.c"  "display.c"  "lightsensor.c"  "ntp.c" "tempsensor.c" "version.c" 
                   "display_icon.c" "C:/SmartEnvi22/longhorn_fw_v1.6.0/components/display/sh1106/sh1106.c"
                   "C:/SmartEnvi22/longhorn_fw_v1.6.0/components/clock/clock.c" 
                   "communication_msg_handler.c" 
                   "communication_server_hal.c" "communication_server.c" 
                   "./common/common.c"
                   "./wifi/wifi_core.c"
                   "./common/filesystem_lib.c"
                   "./sdk_util/sdk_util.c"
                   "./sdk_util/ota_update.c"
                   "./sdk_util/non_volatile_lib.c"
                   "C:/SmartEnvi22/longhorn_fw_v1.6.0/components/mqtt_client/mqtt.c" 
                   "C:/SmartEnvi22/longhorn_fw_v1.6.0/components/display/font/dejavu_sans.c")
                   
set(COMPONENT_ADD_INCLUDEDIRS "." "C:/SmartEnvi22/longhorn_fw_v1.6.0/components/clock/include"
                     "C:/SmartEnvi22/longhorn_fw_v1.6.0/components/display/include" 
                     "./sdk_util/include"
                     "./common/include"
                     "./wifi/include"
                     "C:/SmartEnvi22/longhorn_fw_v1.6.0/components/mqtt_client/include"
                     "C:/esp-idf_v4.0.1/components/esp32/include"
                     "C:/esp-idf_v4.0.1/components/soc/esp32/include/soc"
                     "C:/esp-idf_v4.0.1/components/esp32/include"
                     "C:/esp-idf_v4.0.1/components/esp_adc_cal/include"
                     "C:/esp-idf_v4.0.1/components/log/include"    
                     "C:/esp-idf_v4.0.1/components/nvs_flash/include"             
                     "C:/SmartEnvi22/longhorn_fw_v1.6.0/sdk_list/esp-idf_40/components/nvs_flash/include"
                     "C:/SmartEnvi22/longhorn_fw_v1.6.0/sdk_list/esp-idf_40/components/freertos/include"
                     "C:/SmartEnvi22/longhorn_fw_v1.6.0/sdk_list/esp-idf_40/components/esp_rom/include"
                     "C:/SmartEnvi22/longhorn_fw_v1.6.0/sdk_list/esp-idf_40/components/esp_common/include")

register_component()


if(CONFIG_EXAMPLE_EMBEDDED_CERTS)
target_add_binary_data(${COMPONENT_TARGET} "certs/aws-root-ca.pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/certificate.pem.crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/private.pem.key" TEXT)
endif()

