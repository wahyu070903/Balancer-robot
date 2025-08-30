#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <Bluepad32.h>

#define IMU_SCL 22
#define IMU_SDA 21
#define IMU_INT 27

#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1

#define RIGHT_MOTOR_ENA 2
#define RIGHT_MOTOR_IN1 12
#define RIGHT_MOTOR_IN2 13
#define LEFT_MOTOR_ENA 4
#define LEFT_MOTOR_IN1 14
#define LEFT_MOTOR_IN2 15

// Cek GPIO
#define LEFT_ENC_A 32
#define LEFT_ENC_B 33
#define RIGHT_ENC_A 34
#define RIGHT_ENC_B 35

const char* esp_blu_MAC = "a0:b7:65:14:b7:ae";

double Kp = 1.2;
double Ki = 0.4;
double Kd = 0;

double setpoint = 0;
float deadband = 3.0;
double input, output;
volatile bool dmp_ready;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64] = {0};

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
MPU6050 mpu;

void IRAM_ATTR dmpDataReady() {
  dmp_ready = true;
}

// Virtual parameter for IRAM_ATTR
volatile long* ITR_enc_count_R;
volatile long* ITR_enc_count_L;

void IRAM_ATTR handleEncoderAR() {
    if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) {
    // Clockwise
    (*ITR_enc_count_R)++;
  } else {
    // Counter-clockwise
    (*ITR_enc_count_R)--;
  }
}

void IRAM_ATTR handleEncoderBR() {
  if (digitalRead(RIGHT_ENC_A) != digitalRead(RIGHT_ENC_B)) {
    // Clockwise
    (*ITR_enc_count_R)++;
  } else {
    // Counter-clockwise
    (*ITR_enc_count_R)--;
  }
}

void IRAM_ATTR handleEncoderAL() {
    if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) {
    // Clockwise
    (*ITR_enc_count_L)++;
  } else {
    // Counter-clockwise
    (*ITR_enc_count_L)--;
  }
}

void IRAM_ATTR handleEncoderBL() {
  if (digitalRead(LEFT_ENC_A) != digitalRead(LEFT_ENC_B)) {
    // Clockwise
    (*ITR_enc_count_L)++;
  } else {
    // Counter-clockwise
    (*ITR_enc_count_L)--;
  }
}

class Motor {
  public:
    bool _isReversed = false;
    uint8_t _ena_pin, _in1_pin, _in2_pin;
    uint8_t _enc_a_pin, _enc_b_pin;
    volatile long enc_count;
    long prevCount = 0;
    unsigned long prevTime = 0;
    bool isEncInterrupt = false;
    float motor_rpm = 0;
    double _pid_input, _pid_output, _pid_setpoint = 0;
    double _Kp, _Ki, _Kd;

    Motor(bool reversed, uint8_t ena_pin, uint8_t in1_pin, uint8_t in2_pin, uint8_t enc_a_pin, uint8_t enc_b_pin) {
      _isReversed = reversed;
      _ena_pin = ena_pin;
      _in1_pin = in1_pin;
      _in2_pin = in2_pin;
      _enc_a_pin = enc_a_pin;
      _enc_b_pin = enc_b_pin;  
      // Reset value on startup
      enc_count = 0; 
      

      if(ena_pin == LEFT_MOTOR_ENA){
        ledcSetup(LEFT_PWM_CHANNEL, 1000, 8);
        ledcAttachPin(ena_pin, LEFT_PWM_CHANNEL);
      }else if(ena_pin == RIGHT_MOTOR_ENA){
        ledcSetup(RIGHT_PWM_CHANNEL, 1000, 8);
        ledcAttachPin(ena_pin, RIGHT_PWM_CHANNEL);
      }

      pinMode(_in1_pin, OUTPUT);
      pinMode(_in2_pin, OUTPUT);
      pinMode(_enc_a_pin, INPUT_PULLUP);
      pinMode(_enc_b_pin, INPUT_PULLUP);

      PID pid(&_pid_input, &_pid_output, &_pid_setpoint, _Kp, _Ki, _Kd, DIRECT);
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(10);
      pid.SetOutputLimits(0, 5000); // Max RPM motor
    }

    float ReadMotorSpeed(){
      if(!isEncInterrupt){
        if(_enc_a_pin == LEFT_ENC_A){
          attachInterrupt(digitalPinToInterrupt(_enc_a_pin), handleEncoderAL, CHANGE);
          attachInterrupt(digitalPinToInterrupt(_enc_b_pin), handleEncoderBL, CHANGE);
          ITR_enc_count_L = &enc_count;
        } else {
          attachInterrupt(digitalPinToInterrupt(_enc_a_pin), handleEncoderAR, CHANGE);
          attachInterrupt(digitalPinToInterrupt(_enc_b_pin), handleEncoderBR, CHANGE);
          ITR_enc_count_R = &enc_count;
        }
        isEncInterrupt = true;
      }
      // Read every 100 ms
      unsigned long now = millis();
      if (now - prevTime >= 100){
        long delta = enc_count - prevCount;
        float cps = (delta * 1000.0) / (now - prevTime);
        // CPR = 11PPR x 4 (Quadrature reading)
        const int CPR = 44;
        motor_rpm = (cps * 60.0) / CPR;

        prevCount = enc_count;
        prevTime = now;
      }
      return motor_rpm;
    }

    void SetMotorSpeed(float _speed){
      _pid_setpoint = _speed;
      _pid_input = ReadMotorSpeed();
      pid.Compute();
      if (_ena_pin == LEFT_MOTOR_ENA)
        ledcWrite(LEFT_PWM_CHANNEL, _pid_output);
      else if (_ena_pin == RIGHT_MOTOR_ENA)
        ledcWrite(RIGHT_PWM_CHANNEL, _pid_output);
    }

    void MoveForward(float _speed){
      if (_isReversed) {
        digitalWrite(_in1_pin, LOW);
        digitalWrite(_in2_pin, HIGH);
      } else {
        digitalWrite(_in1_pin, HIGH);
        digitalWrite(_in2_pin, LOW);
      }
      SetMotorSpeed(_speed);
    }

    void MoveBackward(float _speed){
      if (_isReversed) {
        digitalWrite(_in1_pin, HIGH);
        digitalWrite(_in2_pin, LOW);
      } else {
        digitalWrite(_in1_pin, LOW);
        digitalWrite(_in2_pin, HIGH);
      }
      SetMotorSpeed(_speed);
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

Motor leftMotor(true, LEFT_MOTOR_ENA, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_ENC_A, LEFT_ENC_B);
Motor rightMotor(true, RIGHT_MOTOR_ENA, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_ENC_A, RIGHT_ENC_B);

uint8_t* setMotorMoveRatio(uint8_t _ma_ratio, uint8_t _mb_ratio, uint8_t _speed, Motor& rightMotor, Motor& leftMotor) {
  static uint8_t motor_speed[2] = {0, 0};

  float mr_speed = abs(rightMotor.ReadMotorSpeed());
  float ml_speed = abs(leftMotor.ReadMotorSpeed());

  // target = rata-rata biar motor sinkron
  float target = (mr_speed + ml_speed) / 2.0;

  float Kp = 5;  // coba kecil dulu
  float error_r = target - mr_speed;
  float error_l = target - ml_speed;

  int pwm_r = _speed + (int)(Kp * error_r);
  int pwm_l = _speed + (int)(Kp * error_l);

  pwm_r = constrain(abs(pwm_r), 0, 255);
  pwm_l = constrain(abs(pwm_l), 0, 255);

  motor_speed[0] = (uint8_t)pwm_r; // kanan
  motor_speed[1] = (uint8_t)pwm_l; // kiri

  return motor_speed;
}



void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);

      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// Dump all values to serial monitor (Debuging Only)
void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
 
  //== PS4 X button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
    // code for when X button is pushed
  }
  if (ctl->buttons() != 0x0001) {
    // code for when X button is released
  }

  //== PS4 Square button = 0x0004 ==//
  if (ctl->buttons() == 0x0004) {
    // code for when square button is pushed
  }
  if (ctl->buttons() != 0x0004) {
  // code for when square button is released
  }

  //== PS4 Triangle button = 0x0008 ==//
  if (ctl->buttons() == 0x0008) {
    // code for when triangle button is pushed
  }
  if (ctl->buttons() != 0x0008) {
    // code for when triangle button is released
  }

  //== PS4 Circle button = 0x0002 ==//
  if (ctl->buttons() == 0x0002) {
    // code for when circle button is pushed
  }
  if (ctl->buttons() != 0x0002) {
    // code for when circle button is released
  }

  //== PS4 Dpad UP button = 0x01 ==//
  if (ctl->buttons() == 0x01) {
    // code for when dpad up button is pushed
  }
  if (ctl->buttons() != 0x01) {
    // code for when dpad up button is released
  }

  //==PS4 Dpad DOWN button = 0x02==//
  if (ctl->buttons() == 0x02) {
    // code for when dpad down button is pushed
  }
  if (ctl->buttons() != 0x02) {
    // code for when dpad down button is released
  }

  //== PS4 Dpad LEFT button = 0x08 ==//
  if (ctl->buttons() == 0x08) {
    // code for when dpad left button is pushed
  }
  if (ctl->buttons() != 0x08) {
    // code for when dpad left button is released
  }

  //== PS4 Dpad RIGHT button = 0x04 ==//
  if (ctl->buttons() == 0x04) {
    // code for when dpad right button is pushed
  }
  if (ctl->buttons() != 0x04) {
    // code for when dpad right button is released
  }

  //== PS4 R1 trigger button = 0x0020 ==//
  if (ctl->buttons() == 0x0020) {
    // code for when R1 button is pushed
  }
  if (ctl->buttons() != 0x0020) {
    // code for when R1 button is released
  }

  //== PS4 R2 trigger button = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    // code for when R2 button is pushed
  }
  if (ctl->buttons() != 0x0080) {
    // code for when R2 button is released
  }

  //== PS4 L1 trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
    // code for when L1 button is pushed
  }
  if (ctl->buttons() != 0x0010) {
    // code for when L1 button is released
  }

  //== PS4 L2 trigger button = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    // code for when L2 button is pushed
  }
  if (ctl->buttons() != 0x0040) {
    // code for when L2 button is released
  }

  //== LEFT JOYSTICK - UP ==//
  if (ctl->axisY() <= -25) {
    // code for when left joystick is pushed up
    }

  //== LEFT JOYSTICK - DOWN ==//
  if (ctl->axisY() >= 25) {
    // code for when left joystick is pushed down
  }

  //== LEFT JOYSTICK - LEFT ==//
  if (ctl->axisX() <= -25) {
    // code for when left joystick is pushed left
  }

  //== LEFT JOYSTICK - RIGHT ==//
  if (ctl->axisX() >= 25) {
    // code for when left joystick is pushed right
  }

  //== LEFT JOYSTICK DEADZONE ==//
  if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25) {
    // code for when left joystick is at idle
  }

  //== RIGHT JOYSTICK - X AXIS ==//
  if (ctl->axisRX()) {
    // code for when right joystick moves along x-axis
  }

  //== RIGHT JOYSTICK - Y AXIS ==//
  if (ctl->axisRY()) {
  // code for when right joystick moves along y-axis
  }
  // Only for debugging
  // dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(IMU_SDA, IMU_SCL);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

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
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) processControllers();
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
    // Safety
    if (input > -100 && input < 100){
      // Speeds [0] = Right [1] = Left
      uint8_t* speeds = setMotorMoveRatio(1, 1, output, rightMotor, leftMotor);
      if(input < setpoint + deadband){
        rightMotor.MoveBackward(abs(speeds[0]));
        leftMotor.MoveBackward(abs(speeds[1]));
      }else if(input > setpoint - deadband){
        rightMotor.MoveForward(abs(speeds[0]));
        leftMotor.MoveForward(abs(speeds[1]));
      }else{
        rightMotor.MotorStop();
        leftMotor.MotorStop();
      }
    }else{
      rightMotor.MotorStop();
      leftMotor.MotorStop();
    }
  }

  // For diagnostis using serial plotter
  Serial.print(leftMotor.ReadMotorSpeed());
  Serial.print("\t");
  Serial.println(rightMotor.ReadMotorSpeed());
}