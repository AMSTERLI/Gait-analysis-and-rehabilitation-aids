idf_component_register(
    SRCS "wifi_udp.cpp" "qmi8658c.cpp" "img_test.c"
    INCLUDE_DIRS "."
    REQUIRES     
                nvs_flash
                GaitDetector   
                SensorLib
                esp_wifi
                esp_netif
                lwip
)    

