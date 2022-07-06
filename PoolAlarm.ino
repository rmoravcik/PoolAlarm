#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INTERRUPT_PIN     2
#define LED_PIN           3
#define BUZZER_PIN        9

#define X_GYRO_OFFSET    62
#define Y_GYRO_OFFSET   -20
#define Z_GYRO_OFFSET    18

#define X_ACCEL_OFFSET -459
#define Y_ACCEL_OFFSET 1615
#define Z_ACCEL_OFFSET  941

#define LED_HEART_BEAT_TIMEOUT 2000

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool resetToZero = false;
float yprZero[3];
float accelZeroZ;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  Wire.begin();

  Serial.begin(38400);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, 0);

  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);

  mpu.setXAccelOffset(X_ACCEL_OFFSET);
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);
  
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
        
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    digitalWrite(BUZZER_PIN, 1);
    delay(100);  
    digitalWrite(BUZZER_PIN, 0);
    delay(900);  
  }
}

uint64_t lastMillis = 0;

void loop() {
  if (!dmpReady)
    return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    if (!resetToZero) {
      yprZero[0] = ypr[0];
      yprZero[1] = ypr[1];
      yprZero[2] = ypr[2];
      accelZeroZ = aaReal.z;
      digitalWrite(LED_PIN, 0);
      resetToZero = true;
    }

    float pitch = (ypr[1] - yprZero[1]) * 180/M_PI;
    float roll = (ypr[2] - yprZero[2]) * 180/M_PI;
    float accelZ = aaReal.z - accelZeroZ;

    if (abs(accelZ) >= 50.0) {
    Serial.print("Pitch/Roll/AccelZ:\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(roll);
    Serial.print("\t");
    Serial.println(accelZ);
    }

    if ((abs(pitch) >= 2.0) || (abs(roll) >= 2.0)) {
      Serial.println("Alarm!!!");
      while (1) {
        digitalWrite(BUZZER_PIN, 1);
        delay(500);
        digitalWrite(BUZZER_PIN, 0);
        delay(500);
      }
    }
  }

  if (millis() > (lastMillis + LED_HEART_BEAT_TIMEOUT)) {
    if (resetToZero) {
      digitalWrite(LED_PIN, 1);
      delay(200);
      digitalWrite(LED_PIN, 0);
      delay(2000);
    }
    lastMillis = millis();
  }
}
