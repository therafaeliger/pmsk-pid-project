// Library: Adafruit MPU6050, Servo, dll.
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

// Initialize Servo, MPU6050
Servo servo;
Adafruit_MPU6050 mpu;

// Gloval Var Def.
const float SERVO_SIGN = 1.0;
float roll=0, pitch=0, yaw=0;
float biasX=0, biasY=0, biasZ=0;
float q0=1, q1=0, q2=0, q3=0, previousYaw=0;
unsigned long lastSample = 0, lastCommand = 0;
char cmd[40];
uint8_t used = 0;
bool overflow = false;
const float GYRO_DEADBAND = 0.005; // 0.005 ~ 0.28

// setServo function, def 90 deg
void setServo(float relativeAngle=0)
{
    relativeAngle = constrain(relativeAngle, -90.0f, 90.0f);
    int absoluteAngle = (int)round(90.0f + SERVO_SIGN * relativeAngle);
    servo.write(constrain(absoluteAngle, 0, 180));
}

// Update MPU6050 Orientation Data
void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    gx -= biasX;
    gy -= biasY;
    gz -= biasZ;

    // Deadband Filter
    if (abs(gx) < GYRO_DEADBAND) gx = 0;
    if (abs(gy) < GYRO_DEADBAND) gy = 0;
    if (abs(gz) < GYRO_DEADBAND) gz = 0;
  
    float norm = sqrt(ax*ax + ay*ay + az*az);
  
    // Complementary Filter
    if (norm > 7.1f && norm < 12.0f) {
        ax /= norm; ay /= norm; az /= norm;
        float vx = 2*(q1*q3 - q0*q2);
        float vy = 2*(q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        gx += 2*(ay*vz - az*vy); 
        gy += 2*(az*vx - ax*vz); 
        gz += 2*(ax*vy - ay*vx);
    }
  
    // Integrasi Quaternion
    float a=q0, b=q1, c=q2, d=q3;
    q0 += 0.5f*(-b*gx - c*gy - d*gz)*dt;
    q1 += 0.5f*( a*gx + c*gz - d*gy)*dt;
    q2 += 0.5f*( a*gy - b*gz + d*gx)*dt;
    q3 += 0.5f*( a*gz + b*gy - c*gx)*dt;
    
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
  
    // Euler Angles
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180/PI;
    pitch = asin(constrain(2*(q0*q2 - q3*q1), -1.0f, 1.0f)) * 180/PI;
    float rawYaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180/PI;

    float delta = rawYaw - previousYaw;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    yaw += delta; 
    previousYaw = rawYaw; 
}

void setup()
{
    Serial.begin(115200);

    servo.attach(9);
    setServo();

    if (!mpu.begin()) {
        Serial.println("ERR,IMU_NOT_FOUND");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050_FOUND");

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    delay(500);

    // Kalibrasi:
    int valid = 0;
    sensors_event_t a, g, temp;

    for (int i = 0; i < 500; i++) {
        if (mpu.getEvent(&a, &g, &temp)) {
            biasX += g.gyro.x; biasY += g.gyro.y; biasZ += g.gyro.z;
            valid++;
        }
        delay(4);
    }

    if(!valid) {
        Serial.println("ERR,IMU_READ_FAILED");
        while (true) {
            delay(100);
        }
    }

    biasX /= valid; biasY /= valid; biasZ /= valid;

    if (mpu.getEvent(&a, &g, &temp)) {
        float r = atan2(a.acceleration.y, a.acceleration.z);
        float p = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));
        q0 = cos(r/2)*cos(p/2); 
        q1 = sin(r/2)*cos(p/2);
        q2 = cos(r/2)*sin(p/2); 
        q3 = -sin(r/2)*sin(p/2);
    }

    yaw = 0;
    lastSample = lastCommand = millis();
}

void loop()
{
    while(Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n') {
            cmd[used] = 0;
            if (!overflow && strncmp(cmd, "SET,", 4) == 0) {
                char *end; 
                float angle = strtod(cmd + 4, &end);
                if (end != cmd + 4 && *end == 0 && isfinite(angle)) {
                    setServo(angle); 
                    lastCommand = millis();
                }
            }
            used = 0; 
            overflow = false;
        } else if (ch != '\r') {
            if (used < sizeof(cmd)-1 && !overflow) cmd[used++] = ch;
            else overflow = true;
        }
    }

    unsigned long now = millis();

    if (now - lastCommand > 300) setServo(); 

    if (now - lastSample >= 10) {
        float dt = (now - lastSample) / 1000.0; 
        lastSample = now;

        if (dt > 0.1) {
            setServo();
            return;
        }

        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp)) {
            updateOrientation(a.acceleration.x, a.acceleration.y, a.acceleration.z, 
                            g.gyro.x, g.gyro.y, g.gyro.z, dt);
            
            Serial.print("IMU,"); Serial.print(now); Serial.print(',');
            Serial.print(roll, 3); Serial.print(','); 
            Serial.print(pitch, 3); Serial.print(',');
            Serial.println(yaw, 3);
        }
        else {
            setServo();
        }
    }
}