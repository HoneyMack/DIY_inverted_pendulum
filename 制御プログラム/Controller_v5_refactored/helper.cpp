#include "helper.h"

//I2c書き込み関数
void I2c_write(uint8_t device_address, int memory_address, byte value, bool is_continue=false)
{
  Wire.beginTransmission(device_address);           //スレーブ指定
  Wire.write(memory_address);                       //アドレス指定
  Wire.write(value);                                //スレーブに書き込む値
  Wire.endTransmission(!is_continue);                           //終了
}

//I2c読み取り関数
void I2c_read(uint8_t device_address, int memory_address, int read_length, byte *p, bool is_continue=false)
{
  Wire.beginTransmission(device_address);           //スレーブ指定
  Wire.write(memory_address);                       //アドレス指定
  Wire.endTransmission(false);                      //接続続行
  Wire.requestFrom((int)device_address, read_length);    //スレーブ、受け取るデータの長さを指定
  for (int i = 0; i < read_length; i++)             //データを入手する
  {
    *(p + i) = Wire.read();
  }
  Wire.endTransmission(!is_continue);                           //終了
}
