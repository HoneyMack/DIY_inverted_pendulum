#include "ComplementaryPoseEstimator.h"

ComplementaryPoseEstimator::ComplementaryPoseEstimator(std::shared_ptr<PoseSensorBase> poseSensor  /*加速度・角速度などがとれるセンサを扱うクラス*/
    , float cutOffFrequency = 1.0 /*HPFとLPFのカットオフ周波数[Hz]*/): PoseEstimatorBase(poseSensor) {
  _cutOffFrequency = cutOffFrequency;

  //初期姿勢は加速度の身を使って推定
  vector3 accel = _poseSensor->GetAccel();
  _angle = atan2(accel.z,accel.x);
}
void ComplementaryPoseEstimator::Update(float sampleInterval /*データのサンプリング周期[s]*/) {
  interrupts();
  vector3 accel = _poseSensor->GetAccel();
  vector3 angularVelocity = _poseSensor->GetAngularVelocity();
  noInterrupts();
  float K = sampleInterval/(sampleInterval + 1.0f/_cutOffFrequency);

  float angle_accel = atan2(accel.z,accel.x); //要変更
  float angle_next = K*(_angle + sampleInterval*angularVelocity.y) + (1.0f-K)*angle_accel;

  _angularVelocity = (angle_next - _angle)/sampleInterval;
  _angle = angle_next;
}

float ComplementaryPoseEstimator::GetCurrentAngle(){
  return _angle;
}
float ComplementaryPoseEstimator::GetCurrentAngularVelocity(){
  return _angularVelocity;
}
