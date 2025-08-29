#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#define IMU_SCL 22
#define IMU_SDA 21
#define IMU_INT 27

#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1

#define LEFT_MOTOR_ENA 2
#define LEFT_MOTOR_IN1 12
#define LEFT_MOTOR_IN2 13
#define RIGHT_MOTOR_ENA 4
#define RIGHT_MOTOR_IN1 14
#define RIGHT_MOTOR_IN2 15

// Cek GPIO
#define LEFT_ENC_A 32
#define LEFT_ENC_B 33
#define RIGHT_ENC_A 34
#define RIGHT_ENC_B 35

const char* esp_blu_MAC = "a0:b7:65:14:b7:ae";

double Kp = 12;
double Ki = 0;
double Kd = 0;

double setpoint = 0;
float deadband = 3.0;
double input, output;
volatile bool dmp_ready;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64] = {0};

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

MPU6050 mpu;

void onConnect() {
  Serial.println("PS4 Controller Connected");
}
void onDisconnect() {
  Serial.println("PS4 Controller Disconnected");
}
void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}
void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
}
void notify(){
  Serial.printf("lx:%4d,ly:%4d,rx:%4d,ry:%4d\n",
                  PS4.LStickX(),
                  PS4.LStickY(),
                  PS4.RStickX(),
                  PS4.RStickY());
}

void IRAM_ATTR dmpDataReady() {
  dmp_ready = true;
}

class Motor {
  public:
    bool _isReversed = false;
    uint8_t _ena_pin, _in1_pin, _in2_pin;

    Motor(bool reversed, uint8_t ena_pin, uint8_t in1_pin, uint8_t in2_pin) {
      _isReversed = reversed;
      _ena_pin = ena_pin;
      _in1_pin = in1_pin;
      _in2_pin = in2_pin;

      if(ena_pin == LEFT_MOTOR_ENA){
        ledcSetup(LEFT_PWM_CHANNEL, 1000, 8);
        ledcAttachPin(ena_pin, LEFT_PWM_CHANNEL);
      }else if(ena_pin == RIGHT_MOTOR_ENA){
        ledcSetup(RIGHT_PWM_CHANNEL, 1000, 8);
        ledcAttachPin(ena_pin, RIGHT_PWM_CHANNEL);
      }

      pinMode(_in1_pin, OUTPUT);
      pinMode(_in2_pin, OUTPUT);
    }

    void MoveForward(uint8_t speed){
      if (_isReversed) {
        digitalWrite(_in1_pin, LOW);
        digitalWrite(_in2_pin, HIGH);
      } else {
        digitalWrite(_in1_pin, HIGH);
        digitalWrite(_in2_pin, LOW);
      }
      if (_ena_pin == LEFT_MOTOR_ENA)
        ledcWrite(LEFT_PWM_CHANNEL, speed);
      else if (_ena_pin == RIGHT_MOTOR_ENA)
        ledcWrite(RIGHT_PWM_CHANNEL, speed);
    }

    void MoveBackward(uint8_t speed){
      if (_isReversed) {
        digitalWrite(_in1_pin, HIGH);
        digitalWrite(_in2_pin, LOW);
      } else {
        digitalWrite(_in1_pin, LOW);
        digitalWrite(_in2_pin, HIGH);
      }
      if (_ena_pin == LEFT_MOTOR_ENA)
        ledcWrite(LEFT_PWM_CHANNEL, speed);
      else if (_ena_pin == RIGHT_MOTOR_ENA)
        ledcWrite(RIGHT_PWM_CHANNEL, speed);
    }

    void MotorStop(){
      if (_ena_pin == LEFT_MOTOR_ENA)
        ledcWrite(LEFT_PWM_CHANNEL, 0);
      else if (_ena_pin == RIGHT_MOTOR_ENA)
        ledcWrite(RIGHT_PWM_CHANNEL, 0);

      digitalWrite(_in1_pin, LOW);
      digitalWrite(_in2_pin, LOW);
    }
};

Motor leftMotor(true, LEFT_MOTOR_ENA, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2);
Motor rightMotor(false, RIGHT_MOTOR_ENA, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2);

void setup() {
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);

  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  Serial.begin(115200);
  Wire.begin(IMU_SDA, IMU_SCL);

  Serial.println("Inisialisasi MPU6050...");
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 Connected");
  } else {
    Serial.println("[Error] MPU6050 Failed");
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    Serial.println("DMP Ready!");
    mpu.setDMPEnabled(true);
    pinMode(IMU_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
  }else {
    Serial.print("[Error] DMP init failed : ");
    Serial.print(devStatus);
    while (1);
  }

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);
  bool ps_status = PS4.begin(esp_blu_MAC);
  Serial.println(ps_status);
  removePairedDevices();
  Serial.print("This device MAC is: ");
  printDeviceAddress();
  Serial.println("");
}

void loop() {
  if(dmp_ready){
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion q;
      VectorFloat gravity;
      float ypr[3];

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      float robot_angle = ypr[1] * 180/M_PI; // pitch angle
      input = robot_angle;
    }
    dmp_ready = false;
  }else{
    pid.Compute();
    if (input > -50 && input < 50){
      if(input < setpoint + deadband){
        leftMotor.MoveBackward(abs(output));
        rightMotor.MoveBackward(abs(output));
      }else if(input > setpoint - deadband){
        leftMotor.MoveForward(abs(output));
        rightMotor.MoveForward(abs(output));
      }else{
        leftMotor.MotorStop();
        rightMotor.MotorStop();
      }
    }else{
      leftMotor.MotorStop();
      rightMotor.MotorStop();
    }
  }
}
