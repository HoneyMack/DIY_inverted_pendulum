/*倒立振子の制御プログラム

   モータドライバ　TA7291P仕様
   IN1 = 1,IN2 = 1   :ブレーキ
   IN1 = 0,IN2 = 1   :CW/CCW
   IN1 = 1,IN2 = 0   :CCW/CW
   IN1 = 0,IN2 = 0   :ストップ
*/

#include <MsTimer2.h>
#include <Wire.h>
#include <math.h>
#include <Kalman.h>

#define Slave_MPU6050 0x68
#define WHO_AM_I 0x75
#define NUM 1000            //　オフセット用ジャイロ、加速度のサンプリング量
#define NUM_AVERAGE 10      //加速度の一度の角度推定に使うサンプル数
#define NUM_SCALE 10
#define INTERVAL 5         // 1000/INTERVAL Hz

#define GYRO_XOUT_HIGH_ADDRESS  0x43
#define ACCEL_XOUT_HIGH_ADDRESS 0x3B

#define RANGE 500        //ジャイロセンサのレンジ250/500/1000/2000

//モータドライバ　制御デジタルピン
#define R_IN1 4
#define R_IN2 5
#define L_IN1 2
#define L_IN2 3
//モータ出力電圧制御PWMピン
#define R_VREF 11
#define L_VREF 10

//アナログ出力の最小と最大
#define V_MIN 40
#define V_MAX 255

bool flag = false;
int range, i = 0, j = 0, count = 0;
int buff[14];                       //I2cデータ受け取り用
double param = 0.0, degPerSec = 0.0;

//センサの値を持つ構造体
struct senserParam {
  int gx, gy, gz, gxb, gyb, gzb;
  int ax[(NUM_AVERAGE + 1)], ay[(NUM_AVERAGE + 1)], az[(NUM_AVERAGE + 1)];
  long buffx, buffy, buffz; //平均値を保持する変数
  double gyo, gpitch, grole;
  double apitch, arole;
};

//センサのオフセット
struct offset {
  long gx, gy, gz;
  long ax, ay, az;
  double gxcorrect, gycorrect, gzcorrect;
};

struct senserParam sens;
struct offset off;

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
  analogWrite(R_VREF, m.r_Vref);
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

  j = (j + 1) % NUM_SCALE;  //積分修正用のインターバルカウンター

  sens.buffx -= sens.ax[count + 1];
  sens.buffy -= sens.ay[count + 1];
  sens.buffz -= sens.az[count + 1];

  I2c_read(Slave_MPU6050, ACCEL_XOUT_HIGH_ADDRESS, 14, buff);
  sens.ax[count + 1] = ((buff[0] << 8) | buff[1]);
  sens.ay[count + 1] = ((buff[2] << 8) | buff[3]);
  sens.az[count + 1] = ((buff[4] << 8) | buff[5]);
  sens.gx  = ((buff[8] << 8) | buff[9]) - off.gx;
  sens.gy  = ((buff[10] << 8) | buff[11]) - off.gy;
  sens.gz  = ((buff[12] << 8) | buff[13]) - off.gz;

  sens.buffx += sens.ax[count + 1];
  sens.buffy += sens.ay[count + 1];
  sens.buffz += sens.az[count + 1];

  count = (count + 1) % NUM_AVERAGE;

  if (j == 0) {
    sens.gpitch -= off.gxcorrect;
    sens.grole -= off.gycorrect;
    sens.gyo -= off.gzcorrect;
  }

  //加速度の平均値をとる
  sens.ax[0] = sens.buffx / NUM_AVERAGE;
  sens.ay[0] = sens.buffy / NUM_AVERAGE;
  sens.az[0] = sens.buffz / NUM_AVERAGE;

  //yo,pitch,role値算出(台形積分)
  sens.arole = 180 * atan2(sens.ax[0], sens.az[0]) / PI;
  sens.apitch = 180 * atan2(sens.ay[0], sens.az[0]) / PI;
  sens.gyo += ((double)(sens.gz + sens.gzb) * param / 2.0);
  sens.grole += ((double)(sens.gy + sens.gyb) * param / 2.0);
  sens.gpitch += ((double)(sens.gx + sens.gxb) * param / 2.0);

  sens.gxb = sens.gx;
  sens.gyb = sens.gy;
  sens.gzb = sens.gz;

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
  degPerSec = (double)RANGE / 32768000.0;        // センサ受け取り値からdeg/s への係数

  //モータドライバ　制御デジタルピン　出力
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);

  //モータ出力電圧制御PWMピン設定
  analogWrite(R_VREF, 0);
  analogWrite(L_VREF, 0);

  Serial.begin(115200);
  Wire.begin();
  delay(100);
  I2c_write(Slave_MPU6050, 0x1B, range);            //レンジ設定
  I2c_write(Slave_MPU6050, 0x6B, 0x00);             //初期化&計測開始
  I2c_write(Slave_MPU6050, 0x37, 0x02);

  delay(200);

  //オフセットの設定
  for (int i = 0; i < NUM; i++) {
    I2c_read(Slave_MPU6050, GYRO_XOUT_HIGH_ADDRESS, 6, buff);
    off.gx += ((buff[0] << 8) | buff[1]);
    off.gy += ((buff[2] << 8) | buff[3]);
    off.gz += ((buff[4] << 8) | buff[5]);
  }

  off.gx /= NUM;
  off.gy /= NUM;
  off.gz /= NUM;

  MsTimer2::set(INTERVAL, Flag);    // INTERVAL毎,割込み関数を呼び出す様に設定
  MsTimer2::start();             // タイマー割り込み開始

  j = 0;

  delay(100);
  
  //積分誤差修正用の値
  while (j < NUM) {
//    Serial.println("past");
    if (flag == true) {
      //センサ値入手、正規化
      I2c_read(Slave_MPU6050, GYRO_XOUT_HIGH_ADDRESS, 6, buff);
      sens.gx = ((buff[0] << 8) | buff[1]) - off.gx;
      sens.gy = ((buff[2] << 8) | buff[3]) - off.gy;
      sens.gz = ((buff[4] << 8) | buff[5]) - off.gz;

      //時間積分（インターバル一定のため)
      off.gxcorrect += ((sens.gx + sens.gxb) / 2.0) * param;
      off.gycorrect += ((sens.gy + sens.gyb) / 2.0) * param;
      off.gzcorrect += ((sens.gz + sens.gzb) / 2.0) * param;

      sens.gxb = sens.gx;
      sens.gyb = sens.gy;
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
}

void loop() {
  //role,pitch,yo軸シリアル表示

  if (flag == true)
  {
    RawToValueUpdate();
    i = (i + 1) % 20;       //シリアル表示インターバル用カウンター

    if (i == 0) {
      Serial.print("gyro_yo:");
      Serial.print(sens.gyo);
      Serial.print("\taccel_pitch:");
      Serial.print(sens.apitch);
      Serial.print("\tgyro_pitch:");
      Serial.print(sens.gpitch);
      Serial.print("\taccel_role:");
      Serial.print(sens.arole);
      Serial.print("\tgyro_role:");
      Serial.println(sens.grole);
    }
  }

  if (Serial.read() == 'r') {
    sens.gyo = 0.0;
    sens.gpitch = 0.0;
    sens.grole = 0.0;
  }
  /* //モータテスト用
    motor.r_IN1 = LOW;
    motor.r_IN2 = HIGH;
    motor.l_IN1 = LOW;
    motor.l_IN2 = HIGH;
    motor.r_Vref = 40;
    motor.l_Vref = 40;
    motorControl(motor);
    delay(1000);
    motor.r_IN1 = HIGH;
    motor.r_IN2 = LOW;
    motor.l_IN1 = HIGH;
    motor.l_IN2 = LOW;
    motor.r_Vref = 40;
    motor.l_Vref = 40;
    motorControl(motor);
    delay(1000);
  */
}
