#pragma once
#include "PoseEstimatorBase.h"

//姿勢推定器
class ComplementaryPoseEstimator: public PoseEstimatorBase{
  public:
    ComplementaryPoseEstimator(std::shared_ptr<PoseSensorBase> poseSensor  /*加速度・角速度などがとれるセンサを扱うクラス*/,float cutOffFrequency /*HPFとLPFのカットオフ周波数[Hz]*/);
    void StartEstimate(); //推定開始
    float GetCurrentAngle() override; //[rad]
    float GetCurrentAngularVelocity() override; //[rad/s]
    void Update(float sampleInterval /*データのサンプリング周期[s]*/) override; /*角度更新*/
  private:
    float _angle,_angularVelocity;
    float _cutOffFrequency;
};
