//姿勢にまつわるセンサを管理するクラス
#pragma once

typedef struct vector3_t{
  float x,y,z;
}vector3;

class PoseSensorBase{
  public:
    PoseSensorBase(){};
    virtual ~PoseSensorBase(){};
    virtual vector3 GetAccel() = 0;
    virtual vector3 GetAngularVelocity()=0;
    virtual vector3 GetMagnetics(){return vector3{0.f,0.f,0.f};};
};
