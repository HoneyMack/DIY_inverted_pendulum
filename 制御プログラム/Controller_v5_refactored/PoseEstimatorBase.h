#pragma once
#include "PoseSensorBase.h"
#include <ArxSmartPtr.h>

//姿勢推定器
class PoseEstimatorBase {
  public:
    PoseEstimatorBase(std::shared_ptr<PoseSensorBase> poseSensor  /*加速度・角速度などがとれるセンサを扱うクラス*/) {
      _poseSensor = poseSensor;
    };
    virtual float GetCurrentAngle() = 0;
    virtual float GetCurrentAngularVelocity() = 0;
  protected:
    std::shared_ptr<PoseSensorBase> _poseSensor; /*センサ値取得用クラス*/

    virtual void Update(float sampleInterval /*データのサンプリング周期[s]*/) = 0; /*角度更新*/
};
