#ifndef SC7A20H_H
#define SC7A20H_H

#include "i2c_device.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include <functional>

class Sc7a20hSensor : public I2cDevice {
public:
    Sc7a20hSensor(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x19);
    
    esp_err_t Initialize();
    esp_err_t StartReading(float interval_ms = 100);
    esp_err_t StopReading();
    
    esp_err_t ReadAcceleration(int16_t* x, int16_t* y, int16_t* z);
    bool IsReading() const { return reading_; }

    // 加速度数据回调函数类型
    using AccelerationCallback = std::function<void(float x, float y, float z)>;
    
    // 设置加速度数据回调
    void SetAccelerationCallback(AccelerationCallback callback) {
        acceleration_callback_ = callback;
    }

private:
    bool reading_ = false;
    esp_timer_handle_t read_timer_handle_ = nullptr;

    static void ReadTimerCallback(void* arg);
    void OnReadTimer();
    
    AccelerationCallback acceleration_callback_;
    
    // 添加存储加速度值的成员变量
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
};

#endif // SC7A20H_H