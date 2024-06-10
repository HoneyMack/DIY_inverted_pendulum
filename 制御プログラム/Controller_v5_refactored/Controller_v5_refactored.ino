/*倒立振子の制御プログラム
  仕様器具、Arduino Nano, TA7291 * 2, MPU6050, ADXL345, モーター * 2, ギアボックス, スイッチ 等

            ※ADXL345は不使用
   変更点
   ジャイロセンサーのドリフトをより抑えるように積分修正を行った。→　ロボットが一定時間静止に移動する位置量がより減った。
                                                                     ＝時間がたっても開始時の位置からあまりずれなくなった。

   モータドライバ　TA7291P仕様
   IN1 = 1,IN2 = 1   :ブレーキ
   IN1 = 0,IN2 = 1   :CW
   IN1 = 1,IN2 = 0   :CCW
   IN1 = 0,IN2 = 0   :ストップ
*/

#include <math.h>
#include "PoseSensorMPU6050.h"
#include "ComplementaryPoseEstimator.h"
#include <ArxSmartPtr.h>
#include <MsTimer2.h>

//MPU6050周りの設定
const uint8_t SLAVE_MPU6050 = 0x68; //AD0接地時

//相補フィルタ(COmplementaryPoseEstimator)のカットオフ周波数
const float CUT_OFF_FREQUENCY = 5000.0f; //HPFとLPFのカットオフ周波数

const int UPDATE_INTERVAL = 10; //状態の更新周期 [ms]  i.e. 1000/UPDATE_INTERVAL Hz

const double RAD_TO_DEGREE = 180.0 / M_PI; //radから度に変換

//モータドライバ　制御デジタルピン
#define R_IN1 4
#define R_IN2 5
#define L_IN1 2
#define L_IN2 3
//モータ出力電圧制御PWMピン
#define R_VREF 9
#define L_VREF 10

//アナログ出力の最小と最大
#define V_MIN 40
#define V_MAX 255

//モーター移動量調節用係数 度*係数/秒と考える
#define KEISU 5000

#define RESET_SWITCH 6          //ジャイロ度リセット用スイッチのPIN番号
//重心調整用スイッチ UP&DOWN
#define UP_SWITCH 7
#define DOWN_SWITCH 8
#define SWITCH_CHANGE_VALUE 0.0025

#define SWITCH_INTERVAL 100     //繰り返し押せるようになるまでの時間

//#define GYRO_CORRECTION 0.006    //0.005 移動距離における重心修正の係数
#define GYRO_CORRECTION 0.006    //0.005 移動距離における重心修正の係数

#define LROFFSET 1.18 //左右のモーターの速度差を埋める補正係数


volatile bool isStarted = false; //動作開始スイッチ
const float initialStableAngle = 5.0;
volatile float stableAngle = 5.0; //[deg] 倒立振子が倒れない角度


int i = 0, j = 0;
int switchDownCount = 0;
volatile int moveCountCW = 0, moveCountCCW = 0;

//PID制御用パラメータ
volatile double powerP = 0, powerI = 0, powerD = 0, value, value_before;

std::shared_ptr<PoseSensorMPU6050> mpu;           //センサ値取得・管理クラス
std::shared_ptr<ComplementaryPoseEstimator> poseEstimator;    //位置推定管理用クラス


//モーターに関するすべての値を持つ構造体
typedef struct _motorParam {
  volatile byte r_IN1 = LOW, r_IN2 = LOW , l_IN1 = LOW, l_IN2 = LOW; // HIGH or LOW
  volatile byte r_Vref = 122, l_Vref = 122;          //Vref = 40 ～255  40以下だとモーター回らないor不安定
} motorParam;

motorParam motor;

//モーター制御関数
void motorControl(motorParam m) {
  //モーター制御ピンの出力
  digitalWrite(R_IN1, m.r_IN1);
  digitalWrite(R_IN2, m.r_IN2);
  digitalWrite(L_IN1, m.l_IN1);
  digitalWrite(L_IN2, m.l_IN2);

  //モーター出力電圧制御
  analogWrite(R_VREF , min(max(V_MIN, (int)(m.r_Vref * LROFFSET)), V_MAX));
  analogWrite(L_VREF, m.l_Vref);
}

void Updator() {
  //Serial.println("in Updator");
  poseEstimator->Update(UPDATE_INTERVAL / 1000.0f);

  //動作開始されていた場合PID制御
  if (isStarted) {
    //8周期分が75*10ms =750ms -> 750/8 = 93.75ms
    double Ku = 100.0;
    double Pu = 750/8.0 *0.001*4;
    double Kp = 0.6*Ku;
    double Td = Pu/8, Ti = 0.5*Pu;
    double Kd = Td * Kp, Ki = Kp / Ti;
    double dT = UPDATE_INTERVAL / 1000.0;
    double N = 20.0;
    
//    double Kp = 30.0;
//    double Td = 0.010, Ti = 1.0e3;
//    double Kd = Td * Kp, Ki = Kp / Ti;
//    double dT = UPDATE_INTERVAL / 1000.0;
//    double N = 5.0;
    //Serial.println("inStarted");
    powerP = poseEstimator->GetCurrentAngle() * RAD_TO_DEGREE - stableAngle;                  //比例
    powerD = poseEstimator->GetCurrentAngularVelocity() * RAD_TO_DEGREE;     //微分
    powerI += (abs(powerI) < 1000) ? powerP*dT : 0.0f; //積分:オーバーフロー防止
    value = 0.0;
    if (abs(powerP) < 40) { //角度が制御可能域にあるなら
      //value = powerP * 100.0 + powerI  * 0.1 + powerD * 0.0;

      value_before = value;
      
      //value = (Td / (N * dT) * value_before + Kp * powerP/* + Kd * powerD + Ki * powerI*/) / (1.0 + Td / (N * dT));
      value = (Td / (N * dT) * value_before + Kp * powerP + Kd * powerD /*+ Ki * powerI*/) / (1.0 + Td / (N * dT));
      //value = (Td / (N * dT) * value_before + Kp * powerP + Kd * powerD + Ki * powerI) / (1.0 + Td / (N * dT));
      //Serial.println(poseEstimator->GetCurrentAngle() * RAD_TO_DEGREE);
      Serial.println(value);
      //PIDの数値をモーターの出力に変換
      if (abs(value) < 1.0) {   //値が小さすぎる場合はstop
        //前後移動リセット
        moveCountCW = 0;
        moveCountCCW = 0;

        //ブレーキ
        motor.r_IN1 = HIGH;
        motor.r_IN2 = HIGH;
        motor.l_IN1 = HIGH;
        motor.l_IN2 = HIGH;
        motor.r_Vref = 0;
        motor.l_Vref = 0;
      }
      else if (value > 0) {     //CW or CCW

        //移動量を評価し、二次関数的に重心を調整
        moveCountCW++;
        moveCountCCW = 0;
        stableAngle -= moveCountCW * GYRO_CORRECTION;
//        stableAngle -=  GYRO_CORRECTION;
        
        //CW
        motor.r_IN1 = HIGH;
        motor.r_IN2 = LOW;
        motor.l_IN1 = HIGH;
        motor.l_IN2 = LOW;

        motor.r_Vref = min(V_MIN + (int)value, V_MAX);
        motor.l_Vref = motor.r_Vref;
      }
      else {                    //CCW or CW

        //移動量を評価し、二次関数的に重心を調整
        moveCountCCW++;
        moveCountCW = 0;
        stableAngle += moveCountCCW * GYRO_CORRECTION;
//        stableAngle += GYRO_CORRECTION;

        //CCW

        motor.r_IN1 = LOW;
        motor.r_IN2 = HIGH;
        motor.l_IN1 = LOW;
        motor.l_IN2 = HIGH;
        motor.r_Vref = min(V_MIN - (int)value, V_MAX);
        motor.l_Vref = motor.r_Vref;
      }

    }
    else { //角度が大きくなりすぎたので停止処理

      //前後移動リセット
      moveCountCW = 0;
      moveCountCCW = 0;

      //積分リセット
      powerI = 0.0;

      //動作停止
      isStarted = false;

      Serial.println("error, stop");

      //ブレーキ
      motor.r_IN1 = HIGH;
      motor.r_IN2 = HIGH;
      motor.l_IN1 = HIGH;
      motor.l_IN2 = HIGH;
      //モーター停止
      motor.r_Vref = 0;
      motor.l_Vref = 0;
    }
    //Serial.print("value:"); Serial.println(value);

    //モーターの状態を更新
    //モーター制御ピンの出力
    digitalWrite(R_IN1, motor.r_IN1);
    digitalWrite(R_IN2, motor.r_IN2);
    digitalWrite(L_IN1, motor.l_IN1);
    digitalWrite(L_IN2, motor.l_IN2);

    //モーター出力電圧制御
    analogWrite(R_VREF , min(max(V_MIN, (int)(motor.r_Vref * LROFFSET)), V_MAX));
    analogWrite(L_VREF, motor.l_Vref);
  }
}


void setup() {
  //モータドライバ　制御デジタルピン　出力
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(RESET_SWITCH, INPUT_PULLUP);
  pinMode(UP_SWITCH, INPUT_PULLUP);
  pinMode(DOWN_SWITCH, INPUT_PULLUP);

  //モータ出力電圧制御PWMピン設定
  analogWrite(R_VREF, 0);
  analogWrite(L_VREF, 0);

  //センサ値取得・管理クラス
  mpu = std::make_shared<PoseSensorMPU6050>(SLAVE_MPU6050, MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_500);
  mpu->Initialize();//MPUの初期化と動作開始
  //姿勢推定クラス
  poseEstimator = std::make_shared<ComplementaryPoseEstimator>(mpu, CUT_OFF_FREQUENCY);

  //I2cを高速通信モード(fast mode plus)で実行
  Wire.setClock(1000000);
  Wire.begin();

  MsTimer2::set(UPDATE_INTERVAL, Updator);   // INTERVAL毎,割込み関数を呼び出す様に設定
  MsTimer2::start();                  // タイマー割り込み開始

  Serial.begin(115200);

  delay(10);
}

void loop() {
  delay(10);
  //リセットスイッチが押されたら
  //Serial.println("in loop");
  if (digitalRead(RESET_SWITCH) == LOW && switchDownCount == 0) {
    Serial.println("Start");

    stableAngle = initialStableAngle;
    switchDownCount = SWITCH_INTERVAL;      //繰り返し押せるようになるまでの時間:SWITCH_INTERVAL*10 ms

    //動作開始
    isStarted = true;
  }

  //チャタリング防止用カウンター。0になったら再度認識する方式
  if (switchDownCount > 0)
    switchDownCount--;

  //重心調整用スイッチ制御
  if (digitalRead(UP_SWITCH) == LOW)
    stableAngle += SWITCH_CHANGE_VALUE;
  if (digitalRead(DOWN_SWITCH) == LOW)
    stableAngle -= SWITCH_CHANGE_VALUE;

  //Serial.println(motor.r_Vref);


  //Serial.println(poseEstimator->GetCurrentAngle());
}
