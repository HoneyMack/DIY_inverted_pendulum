//MPU6050を使ったセンサ
#pragma once

#include <Arduino.h>
#include <math.h>
#include "PoseSensorBase.h"
#include "helper.h"

#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W

#define GYRO_XOUT_HIGH_ADDRESS  0x43
#define ACCEL_XOUT_HIGH_ADDRESS 0x3B

//#define GYRO_XOUT_HIGH_ADDRESS  0x3B
//#define ACCEL_XOUT_HIGH_ADDRESS 0x43


//加速度レンジ
#define MPU6050_ACCEL_FS_2 0x00 //± 2g
#define MPU6050_ACCEL_FS_4 0x08
#define MPU6050_ACCEL_FS_8 0x10
#define MPU6050_ACCEL_FS_16 0x18


//角速度レンジ
#define MPU6050_GYRO_FS_250 0x00 //± 250°/s
#define MPU6050_GYRO_FS_500 0x08
#define MPU6050_GYRO_FS_1000 0x10
#define MPU6050_GYRO_FS_2000 0x18


class PoseSensorMPU6050: public PoseSensorBase {
  public:
    PoseSensorMPU6050(uint8_t slaveAddress /*スレーブアドレス*/, uint8_t accelRange /*加速度レンジ*/, uint8_t angularRange /*角速度レンジ*/);
    void Initialize(); //初期化処理&通信開始
    virtual ~PoseSensorMPU6050(){};
    vector3 GetAccel() override;
    vector3 GetAngularVelocity() override;
  private:
    uint8_t _slaveAddress;
    uint8_t _accelRange;
    uint8_t _angularRange;
    float _accelSSF;
    float _angularSSF;
    vector3 _offsetAccel,_offsetAngular;
};
