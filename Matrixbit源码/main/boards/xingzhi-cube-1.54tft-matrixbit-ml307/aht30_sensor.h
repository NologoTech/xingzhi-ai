#ifndef AHT30_SENSOR_H
#define AHT30_SENSOR_H

#include "i2c_device.h"
#include "esp_timer.h"
#include <functional>

class Aht30Sensor : public I2cDevice {
public:
    Aht30Sensor(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x38);
    ~Aht30Sensor();
    
    esp_err_t Initialize();
    esp_err_t StartReading(float interval_ms = 5000);
    esp_err_t StopReading();
    
    esp_err_t TriggerMeasurement();
    esp_err_t ReadMeasurement(float* temperature, float* humidity);
    
    bool IsCalibrated() const { return calibrated_; }
    bool IsReading() const { return reading_; }

    // 加速度数据回调函数类型
    using Aht30SensorCallback = std::function<void(float temp, float hum)>;
    
    // 设置加速度数据回调
    void SetAht30SensorCallback(Aht30SensorCallback callback) {
        Aht30Sensor_callback_ = callback;
    }

private:
    bool calibrated_ = false;
    bool reading_ = false;
    esp_timer_handle_t read_timer_handle_ = nullptr;
    
    static void ReadTimerCallback(void* arg);
    void OnReadTimer();

    Aht30SensorCallback Aht30Sensor_callback_;

};

#endif // AHT30_SENSOR_H    