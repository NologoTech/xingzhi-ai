#include "sc7a20h.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define TAG "Sc7a20hSensor"

#define WHO_AM_I_REG 0x0F
#define CTRL_REG1  0x20
#define CTRL_REG4  0x23
#define OUT_X_L_REG  0x28
#define OUT_X_H_REG  0x29
#define OUT_Y_L_REG  0x2A
#define OUT_Y_H_REG  0x2B
#define OUT_Z_L_REG  0x2C
#define OUT_Z_H_REG  0x2D

Sc7a20hSensor::Sc7a20hSensor(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr) {
}

esp_err_t Sc7a20hSensor::Initialize() {
    uint8_t who_am_i_buf[1] = {0};
    ReadRegs(WHO_AM_I_REG, who_am_i_buf, 1); // 使用ReadRegs替代ReadReg
    
    uint8_t who_am_i = who_am_i_buf[0];
    if (who_am_i != 0x11) {
        ESP_LOGE(TAG, "无效的WHO_AM_I值: 0x%02X (期望值: 0x11)", who_am_i);
        return ESP_FAIL;
    }

    // 配置加速度计: 100Hz输出数据率, ±4g量程
    WriteReg(CTRL_REG1, 0x57);
    WriteReg(CTRL_REG4, 0x01);
    
    ESP_LOGI(TAG, "SC7A20H初始化成功");
    return ESP_OK;
}

esp_err_t Sc7a20hSensor::StartReading(float interval_ms) {
    if (reading_) {
        return ESP_OK; // 已经在读取中
    }
    
    esp_timer_create_args_t timer_args = {
        .callback = &Sc7a20hSensor::ReadTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sc7a20h_read_timer",
        .skip_unhandled_events = true
    };
    
    esp_err_t err = esp_timer_create(&timer_args, &read_timer_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建定时器失败 (err=0x%x)", err);
        return err;
    }
    
    err = esp_timer_start_periodic(read_timer_handle_, (uint64_t)(interval_ms * 1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动定时器失败 (err=0x%x)", err);
        esp_timer_delete(read_timer_handle_);
        read_timer_handle_ = nullptr;
        return err;
    }
    
    reading_ = true;
    ESP_LOGI(TAG, "开始周期性读取，间隔 %.1f ms", interval_ms);
    return ESP_OK;
}

esp_err_t Sc7a20hSensor::StopReading() {
    if (!reading_) {
        return ESP_OK; // 已经停止
    }
    
    if (read_timer_handle_) {
        esp_timer_stop(read_timer_handle_);
        esp_timer_delete(read_timer_handle_);
        read_timer_handle_ = nullptr;
    }
    
    reading_ = false;
    ESP_LOGI(TAG, "停止周期性读取");
    return ESP_OK;
}

// 静态定时器回调函数
void Sc7a20hSensor::ReadTimerCallback(void* arg) {
    Sc7a20hSensor* sensor = static_cast<Sc7a20hSensor*>(arg);
    sensor->OnReadTimer();
}

// 定时器触发的读取操作
void Sc7a20hSensor::OnReadTimer() {
    int16_t x, y, z;
    esp_err_t err = ReadAcceleration(&x, &y, &z);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取加速度数据失败 (err=0x%x)", err);
    } else if (err == ESP_OK && acceleration_callback_) {
        acceleration_callback_(accel_x, accel_y, accel_z);
    }
}

esp_err_t Sc7a20hSensor::ReadAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    if (x == NULL || y == NULL || z == NULL) {
        ESP_LOGE(TAG, "无效的输出指针");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[6] = {0};
    ReadRegs(OUT_X_L_REG, &data[0], 1);
    ReadRegs(OUT_X_H_REG, &data[1], 1);
    ReadRegs(OUT_Y_L_REG, &data[2], 1);
    ReadRegs(OUT_Y_H_REG, &data[3], 1);
    ReadRegs(OUT_Z_L_REG, &data[4], 1);
    ReadRegs(OUT_Z_H_REG, &data[5], 1);


    // 组合高低字节(16位有符号数)
    // int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    // int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    // int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]) >> 4;
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]) >> 4;
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]) >> 4;


    // 转换为g单位（±4g量程时，1LSB = 4g / 2^14 ≈ 0.000244g）
    accel_x = raw_x * 0.000244f;
    accel_y = raw_y * 0.000244f;
    accel_z = raw_z * 0.000244f;

    // // 打印原始数据用于调试
    // ESP_LOGI(TAG, "原始数据字节: d0=0x%02X d1=0x%02X d2=0x%02X d3=0x%02X d4=0x%02X",
    //          data[0], data[1], data[2], data[3], data[4]);

    // 打印浮点数格式的加速度值
    ESP_LOGI(TAG, "加速度数据 X:%.2fg Y:%.2fg Z:%.2fg", accel_x, accel_y, accel_z);

    // 保存原始值
    *x = raw_x;
    *y = raw_y;
    *z = raw_z;

    return ESP_OK;
}