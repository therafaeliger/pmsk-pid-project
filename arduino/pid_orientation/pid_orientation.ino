// Arduino Uno/Nano, MPU6050 address 0x68, servo signal D9.
// PID resides in Python. Boot with the IMU stationary for gyro calibration.
#include <Wire.h>
#include <Servo.h>
#include <stdlib.h>
#include <math.h>
Servo servo;
const uint8_t MPU = 0x68;
const float SERVO_SIGN = 1.0; // Change to -1 if a positive command gives negative yaw.
// Mount Z parallel to the vertical servo shaft. Relative gyro yaw can drift.
float roll=0, pitch=0, yaw=0, biasX=0, biasY=0, biasZ=0;
float q0=1, q1=0, q2=0, q3=0, previousYaw=0;
unsigned long lastSample = 0, lastCommand = 0;
char command[40]; uint8_t used = 0;
bool overflow = false;

void regWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(value); Wire.endTransmission();
}
bool readIMU(int16_t *v) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (uint8_t i=0; i<7; i++) v[i] = (int16_t)((Wire.read()<<8) | Wire.read());
  return true;
}
void neutral() { servo.write(90); }
void writeServo(float relativeAngle) {
  relativeAngle=constrain(relativeAngle,-90.0f,90.0f);
  int absoluteAngle=(int)round(90.0f+SERVO_SIGN*relativeAngle);
  servo.write(constrain(absoluteAngle,0,180)); // SG90 absolute mechanical range.
}
void updateOrientation(int16_t *v,float dt) {
  float gx=(v[4]/131.0f-biasX)*PI/180.0f;
  float gy=(v[5]/131.0f-biasY)*PI/180.0f;
  float gz=(v[6]/131.0f-biasZ)*PI/180.0f;
  float ax=v[0],ay=v[1],az=v[2];
  float norm=sqrt(ax*ax+ay*ay+az*az);
  // Quaternion complementary filter: gravity corrects tilt, not absolute yaw.
  // Reject gravity correction during large linear accelerations (+/-2g scale).
  if (norm>12000 && norm<20000) {
    ax/=norm; ay/=norm; az/=norm;
    float vx=2*(q1*q3-q0*q2), vy=2*(q0*q1+q2*q3);
    float vz=q0*q0-q1*q1-q2*q2+q3*q3;
    gx+=2*(ay*vz-az*vy); gy+=2*(az*vx-ax*vz); gz+=2*(ax*vy-ay*vx);
  }
  float a=q0,b=q1,c=q2,d=q3;
  q0+=0.5f*(-b*gx-c*gy-d*gz)*dt;
  q1+=0.5f*(a*gx+c*gz-d*gy)*dt;
  q2+=0.5f*(a*gy-b*gz+d*gx)*dt;
  q3+=0.5f*(a*gz+b*gy-c*gx)*dt;
  norm=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0/=norm; q1/=norm; q2/=norm; q3/=norm;
  roll=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*180/PI;
  pitch=asin(constrain(2*(q0*q2-q3*q1),-1.0f,1.0f))*180/PI;
  float rawYaw=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*180/PI;
  float delta=rawYaw-previousYaw;
  if (delta>180) delta-=360;
  if (delta<-180) delta+=360;
  yaw+=delta; previousYaw=rawYaw; // Continuous relative yaw avoids +/-180 jumps.
}
void setup() {
  Serial.begin(115200); Wire.begin(); Wire.setClock(400000);
  servo.attach(9); neutral();
  regWrite(0x6B, 0); regWrite(0x1B, 0); regWrite(0x1C, 0);
  regWrite(0x1A, 3); // DLPF ~44 Hz gyro, 42 Hz accel; +/-250 deg/s and +/-2g.
  delay(500);
  int16_t v[7]; int valid=0;
  for (int i=0; i<200; i++) {
    if (readIMU(v)) { biasX+=v[4]/131.0; biasY+=v[5]/131.0; biasZ+=v[6]/131.0; valid++; }
    delay(5);
  }
  if (!valid) { Serial.println("ERR,IMU_NOT_FOUND"); while(true) { neutral(); delay(100); } }
  biasX/=valid; biasY/=valid; biasZ/=valid;
  if (readIMU(v)) {
    float r=atan2((float)v[1],(float)v[2]);
    float p=atan2(-(float)v[0],sqrt((float)v[1]*v[1]+(float)v[2]*v[2]));
    q0=cos(r/2)*cos(p/2); q1=sin(r/2)*cos(p/2);
    q2=cos(r/2)*sin(p/2); q3=-sin(r/2)*sin(p/2);
  }
  yaw = 0; // Stationary mechanical neutral becomes the relative yaw origin.
  lastSample = millis(); lastCommand = millis();
}
void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      command[used] = 0;
      if (!overflow && strncmp(command,"SET,",4)==0) {
        char *end; float angle = strtod(command+4, &end);
        if (end != command+4 && *end == 0 && isfinite(angle)) {
          writeServo(angle); lastCommand = millis();
        }
      }
      used=0; overflow=false;
    } else if (ch != '\r') {
      if (used < sizeof(command)-1 && !overflow) command[used++]=ch;
      else overflow=true;
    }
  }
  unsigned long now = millis();
  if (now-lastCommand > 300) neutral(); // USB/software watchdog.
  if (now-lastSample >= 10) {
    float dt = (now-lastSample)/1000.0; lastSample=now;
    int16_t v[7];
    if (!readIMU(v)) { neutral(); return; }
    if (dt > 0.1) { neutral(); return; }
    updateOrientation(v,dt);
    Serial.print("IMU,"); Serial.print(now); Serial.print(',');
    Serial.print(roll,3); Serial.print(','); Serial.print(pitch,3); Serial.print(',');
    Serial.println(yaw,3);
  }
}
