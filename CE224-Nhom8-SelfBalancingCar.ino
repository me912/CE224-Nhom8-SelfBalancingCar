#include    <Kalman.h>
#include    <Servo.h>
#include    <Wire.h>
#include    <math.h>
Kalman kalmanX;
//IMU 6050====================================================
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;
float accXangle;
float gyroXangel;
float kalAngelX;
unsigned long timer;
uint8_t i2cData[14];
float CurrentAngle;
// MOTOR====================================================
int AIN1 = 10;
int AIN2 = 11;
int BIN1 = 12;
int BIN2 = 13;
int CIN1 = 9;
int CIN2 = 14;
int speed;
// PID====================================================
const double Kp = 8;
const double Ki = 0;
const double Kd = 0.9;
double pTerm, iTerm, dTerm, integrated_error, last_error, error;
const double K = 0.96;
const double K_diff = 0.96;
#define GUARD_GAIN 8.0
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))
void setup() 
{
  Wire.setClock(400000);
 Wire.setSDA(0);
 Wire.setSCL(1);
pinMode(AIN1, OUTPUT); 
pinMode(AIN2, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);
Serial.begin(115200);
Serial.println("begin");
Wire.begin();
i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz 
i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling 
i2cData[2] = 0x00;
i2cData[3] = 0x00;
while(i2cWrite(0x19,i2cData,4,false)); 
while(i2cWrite(0x6B,0x01,true));
while(i2cRead(0x75,i2cData,1));
if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
Serial.print(F("Error reading sensor"));
while(1);
 }
delay(100); 
//Kalman====================================================
while(i2cRead(0x3B,i2cData,6));
accX = ((i2cData[0] << 8) | i2cData[1]);
accY = ((i2cData[2] << 8) | i2cData[3]);
accZ = ((i2cData[4] << 8) | i2cData[5]);
accXangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
kalmanX.setAngle(accXangle); 
gyroXangel = accXangle; 
timer = micros();
}
void loop()
{
////  Serial.println("1");
//delay(100);
//// Serial.println(accY);
//// Serial.println(accZ);
//Serial.println(accXangle);
//Serial.println(CurrentAngle);
  
dof();
//Pid(K);
//goc_to_speed();
//Motors();
//Serial.println(error);
//Serial.println(speed);
if(CurrentAngle <= 190 && CurrentAngle >=170)
 {
Pid(K_diff);
Motors(speed);
 }
else
 {
if(CurrentAngle < 280 && CurrentAngle >80)
 {
Pid(K);
Motors(speed);
 }
else
 {
stop();
 }
 }
 
}

void Motors(int new_speed)
{
  Serial.println(new_speed);
if(new_speed > 0)
 {
analogWrite(CIN1, new_speed);
digitalWrite(AIN1, LOW);
digitalWrite(AIN2, HIGH);
analogWrite(CIN2, new_speed);
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, HIGH);
 }
else
 {
new_speed = map(new_speed,0,-255,0,255);
analogWrite(CIN1, new_speed);
digitalWrite(AIN1, HIGH);
digitalWrite(AIN2, LOW);
analogWrite(CIN2, new_speed);
digitalWrite(BIN1, HIGH);
digitalWrite(BIN2, LOW);
 }
}
void stop()
{
speed = map(speed,0,-150,0,150);
analogWrite(CIN1, 0);
digitalWrite(AIN1, LOW);
digitalWrite(AIN2, LOW);
analogWrite(CIN2, 0);
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, LOW);
}
void Pid(float Knew)
{
error = 180 - CurrentAngle; // 180 = level
Serial.println(error);
//pTerm = Kp * speed_error;
// integrated_error += speed_error;
//iTerm = Ki*constrain(integrated_error, -GUARD_GAIN, 
//GUARD_GAIN);
//dTerm = Kd*(speed_error - last_error);
//last_error = speed_error;
//speed = constrain((pTerm + iTerm + dTerm), -255, 255);
//Serial.println(180 - CurrentAngle);
//Serial.println(error);
//Serial.println(new_speed);
//Motors(speed);
pTerm = Kp * error;
 integrated_error += error;
iTerm = Ki*constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
dTerm = Kd*(error - last_error);
last_error = error;
speed = constrain(Knew*(pTerm + iTerm + dTerm), -255, 255);
speed = pow(cosh(speed/30),-2)*30*(error<0 ? -1 : 1) + speed;
}
void dof()
{
while(i2cRead(0x3B,i2cData,14));
accX = ((i2cData[0] << 8) | i2cData[1]);
accY = ((i2cData[2] << 8) | i2cData[3]);
accZ = ((i2cData[4] << 8) | i2cData[5]);
tempRaw = ((i2cData[6] << 8) | i2cData[7]); 
gyroX = ((i2cData[8] << 8) | i2cData[9]);
gyroY = ((i2cData[10] << 8) | i2cData[11]);
gyroZ = ((i2cData[12] << 8) | i2cData[13]);
accXangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
double gyroXrate = (double)gyroY/131.0;
 CurrentAngle = kalmanX.getAngle(accXangle, gyroXrate, 
(double)(micros()-timer)/100000*10);
timer = micros();
}

//int goc_to_speed(float error){
//  int speed;
//  if(abs(error)<0.3){
//    speed = 255*0;
//  }
//  else if(abs(error)<8){
//    speed = 255*0.0;
//  }
//  else if(abs(error)<5){
//    speed = 255*0.2;
//  }else if(abs(error)<8){
//    speed = 255*0.3;
//  }else if(abs(error)<12){
//    speed = 255*0.6;
//  }
//  else if(abs(error)<15){
//    speed = 255*0.65;
//  }
//  else if(abs(error)<18){
//    speed = 255*0.45;
//  }
//  else if(abs(error)<20){
//    speed = 255*0.5;
//  }else if(abs(error)<22){
//    speed = 255*0.55;
//  }else if(abs(error)<25){
//    speed = 255*0.65;
//  }else if(abs(error)<30){
//    speed = 255*0.72;
//  }else if(abs(error)<35){
//    speed = 255*0.78;
//  }else if(abs(error)<40){
//    speed = 255*0.86;
//  }else if(abs(error)<45){
//    speed = 255*0.9;
//  }else if(abs(error)<50){
//    speed = 255*0.92;
//  }else if(abs(error)<55){
//    speed = 255*0.92;
//  }else if(abs(error)<60){
//    speed = 255*0.98;
//  }else{
//    speed = 0;
//  }
//  speed = speed*get_sign(error);
//  return speed;
//}
//int get_sign(double number){
//  return (number < 0)? -1 : 1;
//}
