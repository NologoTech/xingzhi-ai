#include "aht30_sensor.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "Aht30Sensor"

// 命令定义
#define AHT30_CMD_INIT        0xBE  // 初始化命令
#define AHT30_CMD_TRIGGER     0xAC  // 触发测量命令
#define AHT30_CMD_STATUS      0x71  // 状态查询命令

// 状态位定义
#define AHT30_STATUS_BUSY     0x80  // 忙碌标志位
#define AHT30_STATUS_CALIB    0x08  // 校准标志位

Aht30Sensor::Aht30Sensor(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr) {
}

Aht30Sensor::~Aht30Sensor() {
    StopReading();
}

esp_err_t Aht30Sensor::Initialize() {
    // 发送初始化命令
    uint8_t init_cmd[2] = {AHT30_CMD_INIT, 0x08};
    esp_err_t err = i2c_master_transmit(i2c_device_, init_cmd, 2, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send initialization command (err=0x%x)", err);
        return err;
    }
    
    // 等待传感器初始化完成
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 检查校准状态
    uint8_t status = ReadReg(AHT30_CMD_STATUS);
    calibrated_ = (status & AHT30_STATUS_CALIB) != 0;
    
    ESP_LOGI(TAG, "AHT30 initialized, calibrated: %s", calibrated_ ? "YES" : "NO");
    return ESP_OK;
}

esp_err_t Aht30Sensor::StartReading(float interval_ms) {
    if (reading_) {
        return ESP_OK; // 已经在读取中
    }
    
    if (!calibrated_) {
        ESP_LOGE(TAG, "Sensor not calibrated, call Initialize() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_timer_create_args_t timer_args = {
        .callback = &Aht30Sensor::ReadTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "aht30_read_timer",
        .skip_unhandled_events = true
    };
    
    esp_err_t err = esp_timer_create(&timer_args, &read_timer_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer (err=0x%x)", err);
        return err;
    }
    
    err = esp_timer_start_periodic(read_timer_handle_, (uint64_t)(interval_ms * 1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer (err=0x%x)", err);
        esp_timer_delete(read_timer_handle_);
        read_timer_handle_ = nullptr;
        return err;
    }
    
    reading_ = true;
    ESP_LOGI(TAG, "Started periodic readings with interval %.1f ms", interval_ms);
    return ESP_OK;
}

esp_err_t Aht30Sensor::StopReading() {
    if (!reading_) {
        return ESP_OK; // 已经停止
    }
    
    if (read_timer_handle_) {
        esp_timer_stop(read_timer_handle_);
        esp_timer_delete(read_timer_handle_);
        read_timer_handle_ = nullptr;
    }
    
    reading_ = false;
    ESP_LOGI(TAG, "Stopped periodic readings");
    return ESP_OK;
}

// 静态定时器回调函数
void Aht30Sensor::ReadTimerCallback(void* arg) {
    Aht30Sensor* sensor = static_cast<Aht30Sensor*>(arg);
    sensor->OnReadTimer();
}

// 定时器触发的读取操作
void Aht30Sensor::OnReadTimer() {
    float temperature, humidity;
    esp_err_t err = TriggerMeasurement();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger measurement (err=0x%x)", err);
        return;
    }
    
    // 等待测量完成
    vTaskDelay(pdMS_TO_TICKS(50));
    
    err = ReadMeasurement(&temperature, &humidity);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement (err=0x%x)", err);
    }
    if (err == ESP_OK && Aht30Sensor_callback_) {
        Aht30Sensor_callback_(temperature, humidity);
    }
}

esp_err_t Aht30Sensor::TriggerMeasurement() {
    // 发送触发测量命令
    uint8_t measure_cmd[3] = {AHT30_CMD_TRIGGER, 0x33, 0x00};
    return i2c_master_transmit(i2c_device_, measure_cmd, 3, 100);
}

esp_err_t Aht30Sensor::ReadMeasurement(float* temperature, float* humidity) {
    // 读取测量数据(7字节)
    uint8_t buffer[7];
    ReadRegs(AHT30_CMD_STATUS, buffer, 7);
    
    // 检查状态
    if (buffer[0] & AHT30_STATUS_BUSY) {
        ESP_LOGW(TAG, "Sensor is still busy");
        return ESP_ERR_TIMEOUT;
    }
    
    // 解析湿度数据
    uint32_t hum_raw = ((uint32_t)buffer[1] << 12) | 
                       ((uint32_t)buffer[2] << 4) | 
                       ((buffer[3] & 0xF0) >> 4);
    *humidity = (float)hum_raw * 100.0f / 0x100000;
    
    // 解析温度数据
    uint32_t temp_raw = ((uint32_t)(buffer[3] & 0x0F) << 16) | 
                        ((uint32_t)buffer[4] << 8) | 
                         (uint32_t)buffer[5];
    *temperature = (float)temp_raw * 200.0f / 0x100000 - 50.0f;
    
    ESP_LOGD(TAG, "Raw values: temp=0x%06lX, hum=0x%06lX", temp_raw, hum_raw);
    ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", *temperature, *humidity);
    
    return ESP_OK;
}    