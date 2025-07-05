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
#include "aht30_sensor.h"
#include "sc7a20h.h"


#include "esp_vfs_fat.h"
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_private/sdmmc_common.h"


#define TAG "XINGZHI_CUBE_1_54_TFT_MATRIXBIT_ML307"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);
LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

class Rp2040 : public I2cDevice {
public:
    Rp2040(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x41, 0x11);
        printf("rp2040 init success\n");
        //WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }
};


class CustomLcdDisplay : public SpiLcdDisplay {
private:
    lv_obj_t* text_box_container; 
    lv_obj_t* accel_label_; 
    lv_obj_t* temp_hum_data_label_; 
    lv_obj_t* sdcard_label_; 
    bool is_visible_ = false; // 记录显示状态

    void CreateTextBox() {
        DisplayLockGuard lock(this);
        
        // 创建文本框区域容器，改为纵向布局
        auto screen1 = lv_screen_active();
        lv_obj_set_style_text_font(screen1, fonts_.text_font, 0);
        lv_obj_set_style_text_color(screen1, current_theme_.text, 0);
        lv_obj_set_style_bg_color(screen1, current_theme_.background, 0);

        text_box_container = lv_obj_create(screen1);
        lv_obj_set_size(text_box_container, width_, height_ * 0.25); // 增加容器高度以容纳三行
        lv_obj_align(text_box_container, LV_ALIGN_BOTTOM_MID, 0, -5);
        lv_obj_set_style_bg_color(text_box_container, current_theme_.chat_background, 0);
        lv_obj_set_style_border_width(text_box_container, 1, 0);
        lv_obj_set_style_border_color(text_box_container, current_theme_.border, 0);
        lv_obj_set_style_radius(text_box_container, 8, 0);
        lv_obj_set_flex_flow(text_box_container, LV_FLEX_FLOW_COLUMN); // 改为纵向布局
        lv_obj_set_style_pad_all(text_box_container, 5, 0);
        lv_obj_set_scrollbar_mode(text_box_container, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_flag(text_box_container, LV_OBJ_FLAG_HIDDEN);

        // 创建温湿度数据显示标签 - 第一行
        temp_hum_data_label_ = lv_label_create(text_box_container);
        lv_label_set_text(temp_hum_data_label_, "--.--°C --.--%");
        lv_obj_set_style_text_color(temp_hum_data_label_, current_theme_.text, 0);
        lv_obj_set_style_text_font(temp_hum_data_label_, &font_puhui_14_1, 0);
        lv_obj_set_width(temp_hum_data_label_, width_ * 0.9); // 占满容器宽度
        lv_obj_align(temp_hum_data_label_, LV_ALIGN_TOP_LEFT, 10, 5); // 顶部左侧

        // 创建加速度数据显示标签 - 第二行
        accel_label_ = lv_label_create(text_box_container);
        lv_label_set_text(accel_label_, "0.00 0.00 0.00");
        lv_obj_set_style_text_color(accel_label_, current_theme_.text, 0);
        lv_obj_set_style_text_font(accel_label_, &font_puhui_14_1, 0);
        lv_obj_set_width(accel_label_, width_ * 0.9); // 占满容器宽度
        lv_obj_align(accel_label_, LV_ALIGN_TOP_LEFT, 10, 5); // 顶部左侧，由布局自动排列

        // 创建SD卡状态显示标签 - 第三行
        sdcard_label_ = lv_label_create(text_box_container);
        lv_label_set_text(sdcard_label_, "等待检测SD卡");
        lv_obj_set_style_text_color(sdcard_label_, current_theme_.text, 0);
        lv_obj_set_style_text_font(sdcard_label_, &font_puhui_14_1, 0);
        lv_obj_set_width(sdcard_label_, width_ * 0.9); // 占满容器宽度
        lv_obj_align(sdcard_label_, LV_ALIGN_TOP_LEFT, 10, 5); // 顶部左侧，由布局自动排列

        // 为容器设置内边距，使三行之间有间隔
        lv_obj_set_style_pad_row(text_box_container, 1, 0); // 行间距
    }

public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle, 
                    esp_lcd_panel_handle_t panel_handle,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy) 
        : SpiLcdDisplay(io_handle, panel_handle,
                    width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy,
                    {
                        .text_font = &font_puhui_20_4,
                        .icon_font = &font_awesome_20_4,
                        .emoji_font = font_emoji_64_init(),
                    }) {
        DisplayLockGuard lock(this);
        
        // 在父类UI初始化后添加文本框
        if (container_ != nullptr) {
            CreateTextBox();
        }
    }
    
    // 更新加速度显示
    void SetAccelerationText(const char* text) {
        DisplayLockGuard lock(this);
        if (accel_label_ != nullptr) {
            lv_label_set_text(accel_label_, text);
        }
    }

    // 更新温湿度显示
    void SetAht30SensoryText(float temp, float hum) {
        DisplayLockGuard lock(this);
        if (temp_hum_data_label_ != nullptr) {
            char buffer[30];
            sprintf(buffer, "%.2f°C %.2f%%", temp, hum);
            lv_label_set_text(temp_hum_data_label_, buffer);
        }
    }

    // 更新sd卡显示
    void SetSDcardText(const char* text) {
        DisplayLockGuard lock(this);
        if (sdcard_label_ != nullptr) {
            lv_label_set_text(sdcard_label_, text);
        }
    }

    // 显示传感器信息
    void Show() {
        DisplayLockGuard lock(this);
        lv_obj_clear_flag(text_box_container, LV_OBJ_FLAG_HIDDEN);
        is_visible_ = true;
    }
    
    // 隐藏传感器信息
    void Hide() {
        DisplayLockGuard lock(this);
        lv_obj_add_flag(text_box_container, LV_OBJ_FLAG_HIDDEN);
        is_visible_ = false;
    }

    // 查询显示状态
    bool IsVisible() const {
        return is_visible_;
    }

};

class XINGZHI_CUBE_1_54_TFT_MATRIXBIT_ML307 : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    SpiLcdDisplay* display_;
    CustomLcdDisplay* custom_display_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Esp32Camera* camera_;
    Aht30Sensor* aht30_sensor_;
    Sc7a20hSensor* sc7a20h_sensor_;
    Rp2040* Rp2040_;


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

        // Initialize PCA9557
        Rp2040_ = new Rp2040(i2c_bus_, 0x41);
    }

    void InitializeAHT30Sensor() {
        // 初始化传感器
        aht30_sensor_ = new Aht30Sensor(i2c_bus_);
        esp_err_t err = aht30_sensor_->Initialize();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize AHT30 sensor (err=0x%x)", err);
            return;
        }

        // 设置温湿度数据回调
        aht30_sensor_->SetAht30SensorCallback([this](float temp, float hum) {
            UpdateAht30SensorDisplay(temp, hum);
        });

        // 启动周期性读取（每秒一次）
        err = aht30_sensor_->StartReading(3000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start periodic readings (err=0x%x)", err);
        }
    }

    // 更新温湿度显示
    void UpdateAht30SensorDisplay(float temp, float hum) {
        if (custom_display_) {
            custom_display_->SetAht30SensoryText(temp, hum);
        }
    }

    void InitializeSC7A20HSensor() {
        // 初始化传感器
        sc7a20h_sensor_ = new Sc7a20hSensor(i2c_bus_);
        esp_err_t err = sc7a20h_sensor_->Initialize();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "初始化SC7A20H传感器失败 (err=0x%x)", err);
            return;
        }

        // 设置加速度数据回调
        sc7a20h_sensor_->SetAccelerationCallback([this](float x, float y, float z) {
            UpdateAccelerationDisplay(x, y, z);
        });

        // 启动周期性读取（每100ms一次）
        err = sc7a20h_sensor_->StartReading(3000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "启动周期性读取失败 (err=0x%x)", err);
        }
    }

    // 更新加速度显示
    void UpdateAccelerationDisplay(float x, float y, float z) {
        if (custom_display_) {
            char buffer[50];
            sprintf(buffer, "X:%.2f Y:%.2f Z:%.2f", x, y, z);
            custom_display_->SetAccelerationText(buffer);
        }
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

    void InitializeSDcardSpi() {
        // 定义挂载点路径
        // const char* mount_point = "/sdcard";
        
        // 初始化SPI总线配置
        spi_bus_config_t bus_cnf = {
            .mosi_io_num = SDCARD_PIN_MOSI,
            .miso_io_num = SDCARD_PIN_MISO,
            .sclk_io_num = SDCARD_PIN_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 400000,
        };
        
        // 初始化SPI总线
        esp_err_t err = spi_bus_initialize(SDCARD_SPI_HOST, &bus_cnf, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI总线初始化失败: %s", esp_err_to_name(err));
            return;
        }
        
        // 配置SD卡设备
        static sdspi_device_config_t slot_cnf = {
            .host_id = SDCARD_SPI_HOST,
            .gpio_cs = SDCARD_PIN_CS,
            .gpio_cd = SDSPI_SLOT_NO_CD,
            .gpio_wp = GPIO_NUM_NC,
            .gpio_int = GPIO_NUM_NC,
        };
        
        // 配置FAT文件系统挂载选项
        esp_vfs_fat_sdmmc_mount_config_t mount_cnf = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024,
        };
        
        // 声明SD卡对象
        sdmmc_card_t* card = NULL;
        
        // 先将宏结果赋值给变量，再取地址
        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cnf, &mount_cnf, &card);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SD卡挂载失败: %s", esp_err_to_name(err));
            custom_display_->SetSDcardText("SD卡挂载失败");
            return;
        } else if (err == ESP_OK) {
            ESP_LOGI(TAG, "SD卡挂载成功");
            custom_display_->SetSDcardText("SD卡挂载成功");
        }
        
        // 打印SD卡信息
        // #ifdef CONFIG_SDMMC_USE_CUSTOM_PINS
        sdmmc_card_print_info(stdout, card);

        // // Use POSIX and C standard library functions to work with files.

        // // First create a file.
        // const char *file_hello = MOUNT_POINT"/hello.txt";

        // ESP_LOGI(TAG, "Opening file %s", file_hello);
        // FILE *f = fopen(file_hello, "w");
        // if (f == NULL) {
        //     ESP_LOGE(TAG, "Failed to open file for writing");
        //     return;
        // }
        // fprintf(f, "Hello %s!\n", card->cid.name);
        // fclose(f);
        // ESP_LOGI(TAG, "File written");

        // const char *file_foo = MOUNT_POINT"/foo.txt";

        // // Check if destination file exists before renaming
        // struct stat st;
        // if (stat(file_foo, &st) == 0) {
        //     // Delete it if it exists
        //     unlink(file_foo);
        // }

        // // Rename original file
        // ESP_LOGI(TAG, "Renaming file %s to %s", file_hello, file_foo);
        // if (rename(file_hello, file_foo) != 0) {
        //     ESP_LOGE(TAG, "Rename failed");
        //     return;
        // }

        // // Open renamed file for reading
        // ESP_LOGI(TAG, "Reading file %s", file_foo);
        // f = fopen(file_foo, "r");
        // if (f == NULL) {
        //     ESP_LOGE(TAG, "Failed to open file for reading");
        //     return;
        // }

        // // Read a line from file
        // char line[64];
        // fgets(line, sizeof(line), f);
        // fclose(f);

        // // Strip newline
        // char *pos = strchr(line, '\n');
        // if (pos) {
        //     *pos = '\0';
        // }
        // ESP_LOGI(TAG, "Read from file: '%s'", line);

        // // All done, unmount partition and disable SPI peripheral
        // esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        // ESP_LOGI(TAG, "Card unmounted");

        // //deinitialize the bus after all devices are removed
        // spi_bus_free(SDCARD_SPI_HOST);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto& app = Application::GetInstance();
            if (GetNetworkType() == NetworkType::WIFI) {
                if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                    // cast to WifiBoard
                    auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                    wifi_board.ResetWifiConfiguration();
                }
            }
            app.ToggleChatState();
        });

        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting || app.GetDeviceState() == kDeviceStateWifiConfiguring) {
                SwitchNetworkType();
            } else {
                if (custom_display_->IsVisible()) {
                    custom_display_->Hide();
                    ESP_LOGI(TAG,"隐藏传感器信息");
                } else {
                    custom_display_->Show();
                    ESP_LOGI(TAG,"显示传感器信息");
                } 
            }
        });

        // boot_button_.OnDoubleClick([this]() {
        //     ESP_LOGE(TAG,"双击");
        //     power_save_timer_->WakeUp();
        //     auto& app = Application::GetInstance();
        //     // app.ResetDecoder();
        //     if (app.GetDeviceState() == kDeviceStateIdle)
        //     {  
        //         ESP_LOGE(TAG,"双击事件触发");
        //         // app.ResetDecoder();
        //         app.WakeWordInvoke("音量调整到一百，另外你通过摄像机看到了什么");
        //         vTaskDelay(pdMS_TO_TICKS(500));

        //     }
        // });
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

        custom_display_ = new CustomLcdDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        display_ = custom_display_;
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
    XINGZHI_CUBE_1_54_TFT_MATRIXBIT_ML307() :
        DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, 4096),
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeGpio();
        InitializePowerManager();
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeAHT30Sensor();
        InitializeSC7A20HSensor();
        InitializeSpi();
        InitializeButtons();
        InitializeSt7789Display();  
        InitializeSDcardSpi();
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
        DualNetworkBoard::SetPowerSaveMode(enabled);
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(XINGZHI_CUBE_1_54_TFT_MATRIXBIT_ML307);
