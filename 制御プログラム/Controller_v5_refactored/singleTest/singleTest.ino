#include "helper.h"
#include "PoseSensorMPU6050.h"
#include "ComplementaryPoseEstimator.h"
#include <ArxSmartPtr.h>
#include <MsTimer2.h>


const uint8_t SLAVE_MPU6050 = 0x68; //AD0接地時
const float cutOffFrequency = 5000.0f; //HPFとLPFのカットオフ周波数


const int INTERVAL = 10; //10[ms]ごとに状態を更新

std::shared_ptr<PoseSensorMPU6050> mpu;
std::shared_ptr<ComplementaryPoseEstimator> poseEstimator;

void Updator() {
  poseEstimator->Update(INTERVAL / 1000.0f);
}

void setup() {
  // put your setup code here, to run once:
  mpu = std::make_shared<PoseSensorMPU6050>(SLAVE_MPU6050, MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_500);
  mpu->Initialize();//MPUの初期化と動作開始
  poseEstimator = std::make_shared<ComplementaryPoseEstimator>(mpu, cutOffFrequency);

  //fast mode plus
  Wire.setClock(1000000);

  MsTimer2::set(INTERVAL, Updator);   // INTERVAL毎,割込み関数を呼び出す様に設定
  MsTimer2::start();                  // タイマー割り込み開始

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  float angle = poseEstimator->GetCurrentAngle();
  Serial.print(180 * angle / 3.141592); Serial.print(',');
  vector3 accel = mpu->GetAccel();
  angle = atan2(accel.z, accel.x);
  Serial.println(180 * angle / 3.141592);
  //  vector3 accel = mpu->GetAccel();
  //  vector3 angular = mpu->GetAngularVelocity();
  //  Serial.print("accel(x,y,z)=("); Serial.print(accel.x); Serial.print(","); Serial.print(accel.y); Serial.print(","); Serial.print(accel.z); Serial.print(")\n");
  //  Serial.print("angular(x,y,z)=("); Serial.print(angular.x); Serial.print(","); Serial.print(angular.y); Serial.print(","); Serial.print(angular.z); Serial.print(")\n\n");
}
