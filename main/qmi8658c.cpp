#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "lv_conf.h"
#include "lvgl.h"

#include "SensorLib.h"
#include "SensorQMI8658.hpp"
#include "GaitPhaseDetector.h"
#include "wifi_udp.h"

// ===================== 传感器宏定义 =====================
#define I2C_MASTER_SCL   10
#define I2C_MASTER_SDA   11
#define I2C_MASTER_NUM   I2C_NUM_0
#define QMI8658_ADDRESS  0x6B

// ===================== LCD宏定义 =====================
#define EXAMPLE_LCD_H_RES (240)
#define EXAMPLE_LCD_V_RES (280)
#define EXAMPLE_LCD_SPI_NUM (SPI3_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ (40 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS (8)
#define EXAMPLE_LCD_PARAM_BITS (8)
#define EXAMPLE_LCD_COLOR_SPACE (ESP_LCD_COLOR_SPACE_RGB)
#define EXAMPLE_LCD_BITS_PER_PIXEL (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (50)
#define EXAMPLE_LCD_BL_ON_LEVEL (1)
#define EXAMPLE_LCD_GPIO_SCLK (GPIO_NUM_6)
#define EXAMPLE_LCD_GPIO_MOSI (GPIO_NUM_7)
#define EXAMPLE_LCD_GPIO_RST  (GPIO_NUM_8)
#define EXAMPLE_LCD_GPIO_DC   (GPIO_NUM_4)
#define EXAMPLE_LCD_GPIO_CS   (GPIO_NUM_5)
#define EXAMPLE_LCD_GPIO_BL   (GPIO_NUM_15)

static const char *TAG = "APP";

// ===================== 全局对象 =====================
SensorQMI8658 qmi;
IMUdata gyr;
static GaitPhaseDetector gait;
static lv_obj_t *avatar;
static esp_lcd_panel_io_handle_t lcd_io = nullptr;
static esp_lcd_panel_handle_t lcd_panel = nullptr;
static lv_display_t *lvgl_disp = nullptr;

// ===================== I2C初始化 =====================
static void i2c_master_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ===================== 传感器初始化 =====================
static void sensor_setup()
{
    i2c_master_init();
    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, I2C_MASTER_SDA, I2C_MASTER_SCL)) {
        ESP_LOGE(TAG, "QMI8658 not found!");
        vTaskDelete(NULL);
    }
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                            SensorQMI8658::ACC_ODR_1000Hz,
                            SensorQMI8658::LPF_MODE_0, true);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_1024DPS,
                        SensorQMI8658::GYR_ODR_112_1Hz,
                        SensorQMI8658::LPF_MODE_3, true);
    qmi.enableAccelerometer();
    qmi.enableGyroscope();
}

// ===================== WiFi数据发送任务 =====================
static void read_task(void *arg)
{
    char buf[32];
    while (true) {
        if (qmi.getDataReady() && qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
            float gz = gyr.z;
            int code = 0;
            switch (gait.update(gz)) {
                case GaitEvent::IC:  code = 1; break;
                case GaitEvent::HR:  code = 2; break;
                case GaitEvent::TO:  code = 3; break;
                case GaitEvent::FA:  code = 4; break;
                case GaitEvent::TBV: code = 5; break;
                default: break;
            }
            int len = snprintf(buf, sizeof(buf), "%d,%d\n", (int)gyr.z, code);
            wifi_udp_send(buf, len);
        }
        vTaskDelay(pdMS_TO_TICKS(19));
    }
}

// ===================== LCD 初始化 =====================
static esp_err_t app_lcd_init(void)
{
    gpio_config_t bk_gpio_config = {};
    bk_gpio_config.mode = GPIO_MODE_OUTPUT;
    bk_gpio_config.pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL;
    gpio_config(&bk_gpio_config);

    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = EXAMPLE_LCD_GPIO_SCLK;
    buscfg.mosi_io_num = EXAMPLE_LCD_GPIO_MOSI;
    buscfg.max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t);
    spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = EXAMPLE_LCD_GPIO_DC;
    io_config.cs_gpio_num = EXAMPLE_LCD_GPIO_CS;
    io_config.pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ;
    io_config.lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS;
    io_config.lcd_param_bits = EXAMPLE_LCD_PARAM_BITS;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io);

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = EXAMPLE_LCD_GPIO_RST;
    panel_config.color_space = EXAMPLE_LCD_COLOR_SPACE;
    panel_config.bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL;
    esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel);

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL);
    esp_lcd_panel_set_gap(lcd_panel, 0, 20);
    esp_lcd_panel_invert_color(lcd_panel, true);

    return ESP_OK;
}

// ===================== LVGL 初始化 =====================
static esp_err_t app_lvgl_init(void)
{
    lvgl_port_cfg_t lvgl_cfg = {};
    lvgl_cfg.task_priority = 4;
    lvgl_cfg.task_stack = 4096;
    lvgl_cfg.timer_period_ms = 5;
    lvgl_port_init(&lvgl_cfg);

    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = lcd_io;
    disp_cfg.panel_handle = lcd_panel;
    disp_cfg.buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t);
    disp_cfg.double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE;
    disp_cfg.hres = EXAMPLE_LCD_H_RES;
    disp_cfg.vres = EXAMPLE_LCD_V_RES;
    disp_cfg.flags.buff_dma = true;
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    return ESP_OK;
}

// ===================== 显示图像 =====================
static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();
    lvgl_port_lock(0);
    LV_IMG_DECLARE(img_test3);
    avatar = lv_img_create(scr);
    lv_img_set_src(avatar, &img_test3);
    lvgl_port_unlock();
}

// ===================== app_main =====================
extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(app_lcd_init());
    ESP_ERROR_CHECK(app_lvgl_init());
    app_main_display();

    wifi_udp_init("LBL", "77777777", "192.168.163.75", 5005);
    sensor_setup();
    xTaskCreate(read_task, "read_task", 4096, NULL, 10, NULL);
}
