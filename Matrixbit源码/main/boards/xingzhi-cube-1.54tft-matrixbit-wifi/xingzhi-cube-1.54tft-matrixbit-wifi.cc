#include "dual_network_board.h"
#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"

#include "system_reset.h"

#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "iot/thing_manager.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>

#include "power_manager.h"
#include "power_save_timer.h"

#include "led/single_led.h"
#include "assets/lang_config.h"
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <wifi_station.h>
#include "esp32_camera.h"

#define TAG "XINGZHI_CUBE_1_54_TFT_MATRIXBIT_WIFI"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);


class XINGZHI_CUBE_1_54_TFT_MATRIXBIT_WIFI : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    SpiLcdDisplay* display_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Esp32Camera* camera_;

    void InitializePowerManager() {
        power_manager_ = new PowerManager(POWER_USB_IN);//USB是否插入
        power_manager_->OnChargingStatusChanged([this](bool is_charging) {
            if (is_charging) {
                power_save_timer_->SetEnabled(false);
            } else {
                power_save_timer_->SetEnabled(true);
            }
        });
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            ESP_LOGI(TAG, "Enabling sleep mode");
            display_->SetChatMessage("system", "");
            display_->SetEmotion("sleepy");
        });
        power_save_timer_->OnExitSleepMode([this]() {
            display_->SetChatMessage("system", "");
            display_->SetEmotion("neutral");
        });
        power_save_timer_->OnShutdownRequest([this]() {
            ESP_LOGI(TAG, "Shutting down");
            rtc_gpio_set_level(NETWORK_MODULE_POWER_IN, 0);
            // 启用保持功能，确保睡眠期间电平不变
            rtc_gpio_hold_en(NETWORK_MODULE_POWER_IN);
            esp_lcd_panel_disp_on_off(panel_, false); //关闭显示
            esp_deep_sleep_start();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            // .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_SDA;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCL;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(DISPLAY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto& app = Application::GetInstance();
            // if (GetNetworkType() == NetworkType::WIFI) {
            //     if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
            //         // cast to WifiBoard
            //         auto& WifiBoard = static_cast<WifiBoard&>(GetCurrentBoard());
            //         WifiBoard.ResetWifiConfiguration();
            //     }
            // }
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
        boot_button_.OnDoubleClick([this]() {
            ESP_LOGE(TAG,"双击");
            power_save_timer_->WakeUp();
            auto& app = Application::GetInstance();
            // app.ResetDecoder();
            if (app.GetDeviceState() == kDeviceStateIdle)
            {  
                ESP_LOGE(TAG,"双击事件触发");
                // app.ResetDecoder();
                app.WakeWordInvoke("音量调整到一百，另外你通过摄像机看到了什么");
                vTaskDelay(pdMS_TO_TICKS(500));

            }
        });
    }

    void InitializeSt7789Display() {
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS;
        io_config.dc_gpio_num = DISPLAY_DC;
        io_config.spi_mode = 3;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(DISPLAY_SPI_HOST, &io_config, &panel_io_));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RES;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, true));

        display_ = new SpiLcdDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY, 
        {
            .text_font = &font_puhui_20_4,
            .icon_font = &font_awesome_20_4,
            .emoji_font = font_emoji_64_init(),
        });
    }

    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("Battery"));
    }

    void InitializeGpio() {
        gpio_config_t zxc = {};
        zxc.intr_type = GPIO_INTR_DISABLE;
        zxc.mode = GPIO_MODE_OUTPUT;
        zxc.pin_bit_mask = (1ULL << GPIO_NUM_11);
        zxc.pull_down_en = GPIO_PULLDOWN_DISABLE; 
        zxc.pull_up_en = GPIO_PULLUP_DISABLE;     
        gpio_config(&zxc);
        gpio_set_level(GPIO_NUM_11, 1); 
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;  // LEDC通道选择  用于生成XCLK时钟 但是S3不用
        config.ledc_timer = LEDC_TIMER_2; // LEDC timer选择  用于生成XCLK时钟 但是S3不用
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;   // 这里写-1 表示使用已经初始化的I2C接口
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }

public:
    XINGZHI_CUBE_1_54_TFT_MATRIXBIT_WIFI() :
        // WifiBoard(ML307_TX_PIN, ML307_RX_PIN, 4096),
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeGpio();
        InitializePowerManager();
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeSpi();
        InitializeButtons();
        InitializeSt7789Display();  
        InitializeIot();
        InitializeCamera();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            i2c_bus_, 
            I2C_NUM_0,
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_INPUT_REFERENCE);
            return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveMode(bool enabled) override {
        if (!enabled) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveMode(enabled);
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(XINGZHI_CUBE_1_54_TFT_MATRIXBIT_WIFI);
