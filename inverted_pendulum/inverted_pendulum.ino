/*2017.1.13*/
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <SoftwareSerial.h>
#define g 9.832
/****some presetting for MPU6050*************/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float aax = 0, aay = 0, aaz = 0, agx = 0, agy = 0, agz = 0; //角度变量
long axo = 0, ayo = 0, azo = 0; //加速度计偏移量
long gxo = 0, gyo = 0, gzo = 0; //陀螺仪偏移量
float pi = 3.1415926535897;
float AcceRatio = 16384.0; //加速度计比例系数
float GyroRatio = 131.0; //陀螺仪比例系数 float AcceRatio = 16384.0; //加速度计比例系数
float thetax = 0;
//h for arduino uno
float h = 0.02; //time interval used in calculating the angle
//h for arduino yun
//float h = 0.0015; //time interval used in calculating the angle
float omgxn0, omgxn1, omgxn2 = 0;
#define OUTPUT_READABLE_ACCELGYRO
/*********************************************/
/***L298N presetting***/
int motorIn1 = 5;
int motorIn2 = 6;
int motorIn3 = 9;
int motorIn4 = 10;
/******************/
/***Bluetooth presetting*********/
SoftwareSerial BT(12, 13); // rx, tx
char val;
int Kmode = 0; //Kp or Kp or Kd
int Wmode = 0; // record val into inputbuff or not
String inputBuff = "";
/*****************************************************/
/****some presetting for PID control**********/
uint8_t n_sample = 8;
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0}; //x,y轴采样队列
long aax_sum, aay_sum, aaz_sum; //x,y轴采样和
float a_x[10] = {0}, a_y[10] = {0}, a_z[10] = {0} , g_x[10] = {0} , g_y[10] = {0}, g_z[10] = {0}; //加速度计协方差计算队列
float Px = 1, Rx, Kx, Sx, Vx, Qx; //x轴卡尔曼变量
//float Py = 1, Ry, Ky, Sy, Vy, Qy; //y轴卡尔曼变量
//float Pz = 1, Rz, Kz, Sz, Vz, Qz; //z轴卡尔曼变
int command1 = 0;
int command2 = 0;
float Kp =240.69 ; //240.66//228//65//52; // <=====================PID here============
float Ki =3.47; //3.5//4//2.8//0.1325;
float Kd =39.48; //39.5//29.3//1.5//1.068;
float c = 1; //adjust voltage
float angle = 0;
float reference = 0;
float turn=0;
float error1 = 0;
float previousError = 0;
float derivative = 0;
int integral = 0;
float vcc = 8.5;
float vcc_now = 0;
unsigned long time_now, time_last;
float dt = 0;
bool run = false;
/********************************************/
void setup() {
  pinMode(8, OUTPUT);
  /***Bluetooth setup**************************/
  Serial.begin(9600); // arduiono-pc
  Serial.println("BT is ready!");
  BT.begin(9600); // if HC-05嚗se 38400
  /***********************************************/
  /***L298N setup***/
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);
  /******************/
  /***MPU6050 setup***********************************************/
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  unsigned short times = 200; //采样次数
  while(run == false){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if(atan(-(float)ay/(float)az)*180/3.14159265>-0.001 && atan(-(float)ay/(float)az)*180/3.14159265<0.001){
      digitalWrite(8, HIGH);
      run = true;
      delay(500);
      break;
      }
  }
  delay(500);
  for (int i = 0; i < times; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
    axo += ax; ayo += ay; azo += az; //采样和
    gxo += gx; gyo += gy; gzo += gz;
  }
  axo /= times; ayo /= times; azo /= times; //计算加速度计偏移
  gxo /= times; gyo /= times; gzo /= times; //计算陀螺仪偏移
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  /*********************************************************************************************************/
  digitalWrite(8, HIGH);digitalWrite(8, LOW);
  delay(1000);
  digitalWrite(8, HIGH);digitalWrite(8, HIGH);
}
void loop() {
  bluetooth();
  balancing();
}
void forward(int cmd)
{
  analogWrite(motorIn1, 0);
  analogWrite(motorIn2, cmd+turn);
  analogWrite(motorIn3, 0);
  analogWrite(motorIn4, cmd-turn);
}
void backward(int cmd)
{
  analogWrite(motorIn1, cmd+turn);
  analogWrite(motorIn2, 0);
  analogWrite(motorIn3, cmd-turn);
  analogWrite(motorIn4, 0);
}
void hold()
{
  while (1) {
    analogWrite(motorIn1, 0);
    analogWrite(motorIn2, 0);
    analogWrite(motorIn3, 0);
    analogWrite(motorIn4, 0);
  }
}
void balancing()
{
  unsigned long time_now = millis(); //当前时间(ms)
  dt = (time_now - time_last) / 1000.0; //微分时间(s)
  time_last = time_now; //上一次采样时间(ms)
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accx = ax / AcceRatio; //x轴加速度
  float accy = ay / AcceRatio; //y轴加速度
  float accz = az / AcceRatio; //z轴加速度
  aax = atan(accy / accz) * (-180) / pi; //y轴对于z轴的夹角
  //aay = atan(accx / accz) * 180 / pi; //x轴对于z轴的夹角
  //aaz = atan(accz / accy) * 180 / pi; //z轴对于y轴的夹角
  aax_sum = 0; // 对于加速度计原始数据的滑动加权滤波算法
  //aay_sum = 0;
  //aaz_sum = 0;
  for (int i = 1; i < n_sample; i++)
  {
    aaxs[i - 1] = aaxs[i];
    aax_sum += aaxs[i] * i;
    /*
    aays[i - 1] = aays[i];
    aay_sum += aays[i] * i;
    aazs[i - 1] = aazs[i];
    aaz_sum += aazs[i] * i;*/
  }
  aaxs[n_sample - 1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11 * n_sample / 2.0)) * 9 / 7.0; //角度调幅至0-90°
  /*
  aays[n_sample - 1] = aay; //此处应用实验法取得合适的系数
  aay_sum += aay * n_sample; //本例系数为9/7
  aay = (aay_sum / (11 * n_sample / 2.0)) * 9 / 7.0;
  aazs[n_sample - 1] = aaz;
  aaz_sum += aaz * n_sample;
  aaz = (aaz_sum / (11 * n_sample / 2.0)) * 9 / 7.0;*/
  float gyrox = - (gx - gxo) / GyroRatio * dt; //x轴角速度
  //float gyroy = - (gy - gyo) / GyroRatio * dt; //y轴角速度
  //float gyroz = - (gz - gzo) / GyroRatio * dt; //z轴角速度
  agx += gyrox; //x轴角速度积分
  //agy += gyroy; //x轴角速度积分
  //agz += gyroz;
  /* kalman start */
  Sx = 0; Rx = 0;
  //Sy = 0; Ry = 0;
  //Sz = 0; Rz = 0;
  for (int i = 1; i < 10; i++)
  { //测量值平均值运算
    a_x[i - 1] = a_x[i]; //即加速度平均值
    Sx += a_x[i];
    /*a_y[i - 1] = a_y[i];
    Sy += a_y[i];
    a_z[i - 1] = a_z[i];
    Sz += a_z[i];*/
  }
  a_x[9] = aax;
  Sx += aax;
  Sx /= 10; //x轴加速度平均值
  /*a_y[9] = aay;
  Sy += aay;
  Sy /= 10; //y轴加速度平均值
  a_z[9] = aaz;
  Sz += aaz;
  Sz /= 10;*/
  for (int i = 0; i < 10; i++)
  {
    Rx += sq(a_x[i] - Sx);
    //Ry += sq(a_y[i] - Sy);
    //Rz += sq(a_z[i] - Sz);
  }
  Rx = Rx / 9; //得到方差
  //Ry = Ry / 9;
  //Rz = Rz / 9;
  Px = Px + 0.0025; // 0.0025在下面有说明...
  Kx = Px / (Px + Rx); //计算卡尔曼增益
  agx = agx + Kx * (aax - agx); //陀螺仪角度与加速度计速度叠加
  Px = (1 - Kx) * Px; //更新p值
  /*Py = Py + 0.0025;
  Ky = Py / (Py + Ry);
  agy = agy + Ky * (aay - agy);
  Py = (1 - Ky) * Py;
  Pz = Pz + 0.0025;
  Kz = Pz / (Pz + Rz);
  agz = agz + Kz * (aaz - agz);
  Pz = (1 - Kz) * Pz;*/
  /* kalman end */
  angle = agx;
  previousError = error1;
  error1 = angle - reference;
  derivative = error1 - previousError;
  integral += error1;
  command1 = Kp * error1 + Ki * integral + Kd * (derivative);
  //vcc_now = (float)analogRead(A0) * (5.0 / 1023.0) * 2;
 // command2 = command1 * c * vcc / vcc_now;
  if (error1 > 0.0) {
    command1 = constrain(abs(command1), 0, 1023);
    command1 = command1/4;
    forward((int)command1);
  }
  else if (error1 <= 0.0) {
    command1 = constrain(abs(command1), 0, 1023);
    command1 = command1/4;
    backward((int)command1);
  }
  //Serial.print("cmd/angle/gx/gy/gz/ax/ay/az = ");
  Serial.print(time_now / 1000.0);Serial.print("\t");
  Serial.print(command1);Serial.print("\t");
  //Serial.print(atan(-(float)az/(float)ay)*180/3.14159265);Serial.print("\t");
  BT.print(angle); BT.print(" ");
  BT.print(time_now / 1000.0); BT.print(";\n");
  //Serial.print(angle);Serial.print("\t");
  //Serial.print("vcc_now : ");Serial.println("\t");
  //Serial.print(vcc_now);Serial.println("\t");
  /*Serial.print(gx/131.0-0.4);Serial.print("\t");
    Serial.print(gy/131.0-0.2);Serial.print("\t");
    Serial.print(gz/131.0+0.6);Serial.print("\t");
    Serial.print(ax/8192.0*g/2+0.2);Serial.print("\t");
    Serial.print(ay/8192.0*g/2+0.04);Serial.print("\t");
    Serial.println(az/8192.0*g/2-0.2); //8192*g 
  Serial.print(angle); Serial.print(",");
  Serial.print(agx); Serial.print(",");
  Serial.print(agy); Serial.print(",");
  Serial.print(agz); Serial.println();*/
}
void bluetooth()
{
  if (Serial.available()) {
    //val = Serial.read();
  }
  if (BT.available()) {
    val = BT.read();
    if(val=='N'){
      Serial.print("N");
        hold();
    }
    if(val=='Z'){
      Serial.print("Z");
        reference=0;
        turn=0;
    }
    switch(val){
      case 2:
      reference++;
      case 3:
      turn--;
      case 5:
      turn++;
      case 6:
      reference--;
    }
  }// if BT.avilible()
}
