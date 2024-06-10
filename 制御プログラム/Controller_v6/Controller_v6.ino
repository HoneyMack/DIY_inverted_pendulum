/*倒立振子の制御プログラム
  仕様器具、Arduino Nano, TA7291 * 2, MPU6050, ADXL345, モーター * 2, ギアボックス, スイッチ 等
            
            ※ADXL345は不使用
   変更点
   ジャイロセンサーのドリフトをより抑えるように積分修正を行った。→　ロボットが一定時間静止に移動する位置量がより減った。
                                                                     ＝時間がたっても開始時の位置からあまりずれなくなった。
   PIDの値を変更、立てかけた状態から自立できるようにする。
   
   モータドライバ　TA7291P仕様
   IN1 = 1,IN2 = 1   :ブレーキ
   IN1 = 0,IN2 = 1   :CW
   IN1 = 1,IN2 = 0   :CCW
   IN1 = 0,IN2 = 0   :ストップ
*/

#include <MsTimer2.h>
#include <Wire.h>
#include <math.h>

#define Slave_MPU6050 0x68 //AD0接地時
#define Slave_ADXL345 0x53 //SDO接地時
#define WHO_AM_I 0x75
#define NUM 1000            //　オフセット用ジャイロ、加速度のサンプリング量
#define NUM_AVERAGE 3      //ジャイロの一度の角度推定に使うサンプル数
#define NUM_SCALE 5
#define INTERVAL 10         // 1000/INTERVAL Hz

#define DATAX_LOW_ADDRESS 0x32        //ADXL345　accel data lhlhlh
#define GYRO_XOUT_HIGH_ADDRESS  0x43
#define ACCEL_XOUT_HIGH_ADDRESS 0x3B

#define RANGE 500        //ジャイロセンサのレンジ250/500/1000/2000

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

#define RESET_SWITCH 6          //ジャイロ度リセット
//重心調整用スイッチ UP&DOWN
#define UP_SWITCH 7
#define DOWN_SWITCH 8
#define SWITCH_CHANGE_VALUE 0.0025

#define SWITCH_INTERVAL 7000     //繰り返し押せるようになるまでの時間

#define GYRO_CORRECTION 0.006    //0.005 移動距離における重心修正の係数

#define LROFFSET 1.05



bool flag = false,start_flag = false;
int range, i = 0, j = 0, count = 0,switch_down_count,times_count1,times_count2;
int buff[14];                       //I2cデータ受け取り用
double param = 0.0, degPerSec = 0.0;

double PowerP,PowerI,PowerD,value;


//センサの値を持つ構造体
struct senserParam {
  int gx, gy[NUM_AVERAGE + 1], gz, gxb, gyb, gzb;
  int ax, ay, az;
  long buffx, buffy, buffz;         //平均値を保持する変数
  double gyo, gpitch, grole,role,debagg;
  double apitch, arole;
}sens;

//センサのオフセット
struct offset {
  long gx, gy, gz;
  long ax, ay, az;
  double gxcorrect, gycorrect, gzcorrect;
}off;

//モーターに関するすべての値を持つ構造体
struct motorParam {
  byte r_IN1 = LOW, r_IN2 = LOW , l_IN1 = LOW, l_IN2 = LOW; // HIGH or LOW
  byte r_Vref = 122, l_Vref = 122;          //Vref = 40 ～255  40以下だとモーター回らないor不安定
} motor;

//モーター制御関数
void motorControl(struct motorParam m) {
  //モーター制御ピンの出力
  digitalWrite(R_IN1, m.r_IN1);
  digitalWrite(R_IN2, m.r_IN2);
  digitalWrite(L_IN1, m.l_IN1);
  digitalWrite(L_IN2, m.l_IN2);

  //モーター出力電圧制御
  analogWrite(R_VREF  , min(max(V_MIN,((int)m.r_Vref * LROFFSET)),V_MAX));
  analogWrite(L_VREF, m.l_Vref);
}

//I2c書き込み関数
void I2c_write(int device_address, int memory_address, int value)
{
  Wire.beginTransmission(device_address);           //スレーブ指定
  Wire.write(memory_address);                       //アドレス指定
  Wire.write(value);                                //スレーブに書き込む値
  Wire.endTransmission();                           //終了
}

//I2c読み取り関数
void I2c_read(int device_address, int memory_address, int read_length, int *p)
{
  Wire.beginTransmission(device_address);           //スレーブ指定
  Wire.write(memory_address);                       //アドレス指定
  Wire.requestFrom(device_address, read_length);    //スレーブ、受け取るデータの長さを指定
  for (int i = 0; i < read_length; i++)             //データを入手する
  {
    *(p + i) = Wire.read();
  }
  Wire.endTransmission();                           //終了
}

//センサの値更新
void RawToValueUpdate() {
  
  //チャタリング防止用カウンター。オーバーフロー防止ずみ
  if(switch_down_count >-100)
  switch_down_count--;

  sens.buffy -= sens.gy[count + 1];           //平均値更新

  //センサ値入手
  I2c_read(Slave_MPU6050, GYRO_XOUT_HIGH_ADDRESS, 6, buff);
  sens.gy[count + 1]  = ((buff[2] << 8) | buff[3]) - off.gy;

  sens.buffy += sens.gy[count + 1];           //平均値更新

  count = (count + 1) % NUM_AVERAGE;          //平均値更新用カウンター
 
  sens.gy[0] = sens.buffy /NUM_AVERAGE;       //角速度の平均値を得る

  j = (j + 1) % NUM_SCALE;  //積分修正用のインターバルカウンター
//  if(j == 1)
//  sens.role -= ((double)(sens.gy[0] + sens.gyb + off.gycorrect) * param / 2.0);
//  else
//  sens.role -= ((double)(sens.gy[0] + sens.gyb) * param / 2.0);

  if (j == 0) {
    sens.gy[0] -= off.gycorrect;                 //積分誤差修正
  }

  //role値算出(台形積分)
  sens.grole -= ((double)(sens.gy[0] + sens.gyb) * param / 2.0);
//  sens.debagg -= ((double)(sens.gy[0] + sens.gyb) * param / 2.0);
  
  sens.gyb = sens.gy[0];                      //台形積分用

  flag = false;
  return;
}

//周期固定用フラグ
void Flag() {
  flag = true;
  return;
}

void setup() {

  //レンジの設定
  if (RANGE == 250)
    range = 0b00000;
  else if (RANGE == 500)
    range = 0b01000;
  else if (RANGE == 1000)
    range = 0b10000;
  else if (RANGE == 2000)
    range = 0b11000;

  param = ((double)RANGE / 1000 ) * INTERVAL / 32768.0; //値１あたりのdeg（時間あたり）。ジャイロ積分係数
  degPerSec = (double)RANGE / 32768000.0;               // センサ受け取り値からdeg/s への係数

  //モータドライバ　制御デジタルピン　出力
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(RESET_SWITCH,INPUT_PULLUP);
  pinMode(UP_SWITCH,INPUT_PULLUP);
  pinMode(DOWN_SWITCH,INPUT_PULLUP);

  //モータ出力電圧制御PWMピン設定
  analogWrite(R_VREF, 0);
  analogWrite(L_VREF, 0);

  Serial.begin(115200);
  Wire.begin();
  delay(100);
  I2c_write(Slave_MPU6050, 0x1B, range);            //レンジ設定
  I2c_write(Slave_MPU6050, 0x6B, 0x00);             //初期化&計測開始
//  I2c_write(Slave_MPU6050, 0x37, 0x02);
//  I2c_write(Slave_ADXL345, 0x31, 0b00000000);       //レンジ設定
//  I2c_write(Slave_ADXL345, 0x2D, 0b00001000);       //測定開始

  delay(200);

  //オフセットの設定
  for (int i = 0; i < NUM; i++) {
    I2c_read(Slave_MPU6050, GYRO_XOUT_HIGH_ADDRESS, 6, buff);
    off.gx += ((buff[0] << 8) | buff[1]);
    off.gy += ((buff[2] << 8) | buff[3]);
    off.gz += ((buff[4] << 8) | buff[5]);

//    I2c_read(Slave_ADXL345, DATAX_LOW_ADDRESS, 6, buff);
//    off.ax += ((buff[1] << 8) | buff[0]);
//    off.ay += ((buff[3] << 8) | buff[2]);
//    off.az += ((buff[5] << 8) | buff[4]);
  }

  off.gx /= NUM;
  off.gy /= NUM;
  off.gz /= NUM;

  MsTimer2::set(INTERVAL, Flag);    // INTERVAL毎,割込み関数を呼び出す様に設定
  MsTimer2::start();             // タイマー割り込み開始

  delay(300);

  //積分誤差修正用の値
  while (j < NUM) {

    if (flag == true) {
      //センサ値入手、正規化
      I2c_read(Slave_MPU6050, GYRO_XOUT_HIGH_ADDRESS, 6, buff);
      sens.gx = ((buff[0] << 8) | buff[1]) - off.gx;
      sens.gy[0] = ((buff[2] << 8) | buff[3]) - off.gy;
      sens.gz = ((buff[4] << 8) | buff[5]) - off.gz;

      //時間積分（インターバル一定のため)
      off.gxcorrect += ((sens.gx + sens.gxb) / 2.0);
      off.gycorrect += ((sens.gy[0] + sens.gyb) / 2.0);
      off.gzcorrect += ((sens.gz + sens.gzb) / 2.0);

      sens.gxb = sens.gx;
      sens.gyb = sens.gy[0];
      sens.gzb = sens.gz;

      j = j + 1;

      flag == false;
    }
  }

  //一定時間（NUM/NUM_SCALE）msごとに修正するために正規化
  off.gxcorrect /= (NUM / NUM_SCALE);
  off.gycorrect /= (NUM / NUM_SCALE);
  off.gzcorrect /= (NUM / NUM_SCALE);

  j = 0;
  i = 0;
  delay(100);
}

void loop() {
  //スタート判定用フラグ
  if(digitalRead(RESET_SWITCH) == LOW){
    Serial.println("start");
    start_flag =true;
  }

  //INTERVAL msごとに処理
  if (flag == true &&start_flag)
  {
    RawToValueUpdate();                     //センサ値更新

    PowerP = sens.grole;                    //比例
    PowerD = -sens.gy[0]*degPerSec;         //微分
//    PowerI += sens.grole;                   //積分

    //オーバーフロー防止 足した絶対値1000以下
    if(abs(PowerI + sens.grole) < 1000){
       PowerI += sens.grole;
    }

    //積分量が倒れている角度を修正する方向に働いていない場合、0にする
//    if((PowerI >0 && sens.grole < 0)||(PowerI < 0 && sens.grole > 0))
//      PowerI = 0.0;

    if(abs(sens.grole) <=0.3 &&abs(PowerD) <=0.01){
      value = 0.0;
    }    
    else if(abs(sens.grole)<20){
      //PowerDかPowerPのどちらかが負ならture
      if(abs(PowerD - PowerP) > abs(PowerD + PowerP))
        value = PowerP * 36.0 + PowerI  *5.35 + PowerD * 1200.0;        //36.0, 5.35, 1000.0 2 3
      else
        value = PowerP * 48.0 + PowerI * 5.35 + PowerD * 6500.0;        //48.0, 5.35, 5000.0 7 8

      value *= 1.04;
    }
    else{
      MsTimer2::stop();
      Serial.println("error, stop");
      
      value = 0.0;
      motor.r_IN1 = HIGH;
      motor.r_IN2 = HIGH;
      motor.l_IN1 = HIGH;
      motor.l_IN2 = HIGH;
      motor.r_Vref = V_MAX;
      motor.l_Vref = motor.r_Vref;
      
      motorControl(motor);  //モーターの状態を更新
      
      while(1){
        delay(1000);        
      }
      
//      if(abs(PowerD - PowerP) > abs(PowerD + PowerP))
//        value = PowerP * 30.0 + PowerI * 2.05 + PowerD * 5000.0;
//      else
//        value = PowerP * 40.0 + PowerI * 2.05 + PowerD * 9000.0;
    }

    //PIDの数値をモーターの出力に変換
    if(abs(value) <1.0){      //stop

      times_count1 = 0;
      times_count2 = 0;
      
      motor.r_IN1 = HIGH;
      motor.r_IN2 = HIGH;
      motor.l_IN1 = HIGH;
      motor.l_IN2 = HIGH;
      motor.r_Vref = V_MAX;
      motor.l_Vref = motor.r_Vref;
    }
    else if(value >0){        //CW or CCW
      
      //移動量を評価し、二次関数的に重心を調整
      times_count1++;
      times_count2 = 0;
      sens.grole += times_count1 * GYRO_CORRECTION;                         
        
      motor.r_IN1 = LOW;
      motor.r_IN2 = HIGH;
      motor.l_IN1 = LOW;
      motor.l_IN2 = HIGH;
      motor.r_Vref = min(max(V_MIN,((int)value)),V_MAX);
      motor.l_Vref = motor.r_Vref;
    }
    else{                     //CCW or CW

      //移動量を評価し、二次関数的に重心を調整
      times_count2++;
      times_count1 = 0;
      sens.grole -= times_count2 * GYRO_CORRECTION;                          
      
      motor.r_IN1 = HIGH;
      motor.r_IN2 = LOW;
      motor.l_IN1 = HIGH;
      motor.l_IN2 = LOW;
      motor.r_Vref = min(max(V_MIN,-(int)value),V_MAX);
      motor.l_Vref = motor.r_Vref;      
    }
    
    motorControl(motor);  //モーターの状態を更新
    
    //重心調整用スイッチ制御
    if(digitalRead(UP_SWITCH)==LOW)
      sens.grole += SWITCH_CHANGE_VALUE;
    if(digitalRead(DOWN_SWITCH)==LOW)
      sens.grole -= SWITCH_CHANGE_VALUE;
            
    i = (i + 1) % 10;       //シリアル表示インターバル用カウンター      
    //デバッグ用シリアル表示
    if (i == 0) 
    {
//      Serial.print("r_IN1:");
//      Serial.print(motor.r_IN1);
//      Serial.print("\tr_IN2:");
//      Serial.print(motor.r_IN2);
//      Serial.print("\tl_IN1:");
//      Serial.print(motor.l_IN1);
//      Serial.print("\tl_IN2:");
//      Serial.print(motor.l_IN2);
//      Serial.print("\tVref:");
//      Serial.print(motor.l_Vref);
//      Serial.print("\tPowerP:");
//      Serial.print(PowerP);
//      Serial.print("\tPowerI:");
//      Serial.print(PowerI);
//      Serial.print("\tPowerD:");
//      Serial.print(PowerD);
//      Serial.print("\trole:");
//      Serial.print(sens.role);
//      Serial.print("\tdebagg:");
//      Serial.print(sens.debagg);
//      Serial.print("\tgrole:");
//      Serial.println(sens.grole);
    }
  }
  //重心リセット （重要性はかなり低い）
  if(digitalRead(RESET_SWITCH) == LOW&&switch_down_count<0){
    Serial.println("LOW");
    sens.grole =0.0;
    sens.role = 0.0;
    sens.debagg = 0.0;
    switch_down_count = SWITCH_INTERVAL;      //繰り返し押せるようになるまでの時間:SWITCH_INTERVAL*INTERVAL ms
  }
}
