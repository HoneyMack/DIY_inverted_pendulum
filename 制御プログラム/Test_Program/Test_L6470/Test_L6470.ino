void setup() {
  send1_L6470(0x07);
  send2_L6470(600);
}
void send1_L6470(int8_t x) {
  digitalWrite(10, LOW);
  SPI.transfer(x);
  digitalWrite(10, HIGH);
}
void send2_L6470(int16_t x) {
  int8_t buf[2];
  buf[0] = x >> 8;
  buf[1] = x & 0xff;
  digitalWrite(10, LOW);
  SPI.transfer(buf[0]);
  digitalWrite(10, HIGH);
  digitalWrite(10, LOW);
  SPI.transfer(buf[1]);
  digitalWrite(10, HIGH);
}
