#include "PoseSensorMPU6050.h"
#include "helper.h"

const float pi = 3.141592653589793;

PoseSensorMPU6050::PoseSensorMPU6050(uint8_t slaveAddress /*スレーブアドレス*/, uint8_t accelRange /*加速度レンジ*/, uint8_t angularRange /*角速度レンジ*/): PoseSensorBase() {
  _slaveAddress = slaveAddress;
  uint8_t rangeValue = 0;
  //レンジの設定
  _accelRange = accelRange;
  _angularRange = angularRange;

  //binary to value変換係数を計算
  //感度の計算
  int range = 0;
  switch (accelRange) {
    case MPU6050_ACCEL_FS_2:
      range = 2; break;
    case MPU6050_ACCEL_FS_4:
      range = 4; break;
    case MPU6050_ACCEL_FS_8:
      range = 8; break;
    case MPU6050_ACCEL_FS_16:
      range = 16; break;
  }
  _accelSSF = 32768.0f / range;

  range = 0;
  switch (angularRange) {
    case MPU6050_GYRO_FS_250:
      range = 250; break;
    case MPU6050_GYRO_FS_500:
      range = 500; break;
    case MPU6050_GYRO_FS_1000:
      range = 1000; break;
    case MPU6050_GYRO_FS_2000:
      range = 2000; break;
  }
  _angularSSF = 32768.0f / range;

  _offsetAccel = _offsetAngular = vector3{0.0f,0.0f,0.0f};
}

void PoseSensorMPU6050::Initialize() {
  //レンジ設定
  I2c_write(_slaveAddress, MPU6050_ACCEL_CONFIG, _accelRange);
  I2c_write(_slaveAddress,  MPU6050_GYRO_CONFIG, _angularRange);
  //初期化&計測開始
  I2c_write(_slaveAddress, 0x6B, 0x00);
  //I2c_write(_slaveAddress, 0x68, 0x06);
  //初期化時は1秒ほど停止しているとしてジャイロのオフセットを計算
  int num = 20;
  vector3 accumAngular{0.0f, 0.0f, 0.0f};
  for (int i = 0; i < num; i++) {
    delay(5);
    auto angular = GetAngularVelocity();
    accumAngular.x += angular.x;
    accumAngular.y += angular.y;
    accumAngular.z += angular.z;
  }
  _offsetAngular.x = accumAngular.x / num;
  _offsetAngular.y = accumAngular.y / num;
  _offsetAngular.z = accumAngular.z / num;
}


//acceleration [g]
vector3 PoseSensorMPU6050::GetAccel() {
  byte buff[6];
  vector3 accel;

  I2c_read(_slaveAddress, ACCEL_XOUT_HIGH_ADDRESS, 6, buff);
  accel.x = ((int16_t)((buff[0] << 8) | buff[1])) / _accelSSF;
  accel.y = ((int16_t)((buff[2] << 8) | buff[3])) / _accelSSF;
  accel.z = ((int16_t)((buff[4] << 8) | buff[5])) / _accelSSF;

  //あきらかにz方向にゆがみがあるので修正
  accel.z += 0.25;
  return accel;
}


//angular velocity [rad/s]
vector3 PoseSensorMPU6050::GetAngularVelocity() {
  byte buff[6];
  vector3 angularVelocity;

  I2c_read(_slaveAddress, GYRO_XOUT_HIGH_ADDRESS, 6, buff);
  angularVelocity.x = ((int16_t)((buff[0] << 8) | buff[1])) / _angularSSF;
  angularVelocity.y = ((int16_t)((buff[2] << 8) | buff[3])) / _angularSSF;
  angularVelocity.z = ((int16_t)((buff[4] << 8) | buff[5])) / _angularSSF;

  //°->rad変換
  angularVelocity.x *= pi / 180.0f;
  angularVelocity.y *= pi / 180.0f;
  angularVelocity.z *= pi / 180.0f;

  //オフセットを除く
  angularVelocity.x -= _offsetAngular.x;
  angularVelocity.y -= _offsetAngular.y;
  angularVelocity.z -= _offsetAngular.z;

  return angularVelocity;
}
