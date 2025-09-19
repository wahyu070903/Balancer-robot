#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include "driver/pcnt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define IMU_SCL 22
#define IMU_SDA 21
#define IMU_INT 27

#define RIGHT_PWM_CHANNEL_R 2
#define RIGHT_PWM_CHANNEL_L 3
#define LEFT_PWM_CHANNEL_R 4
#define LEFT_PWM_CHANNEL_L 5

#define RIGHT_MOTOR_RPWM 4
#define RIGHT_MOTOR_LPWM 2
#define LEFT_MOTOR_RPWM 5
#define LEFT_MOTOR_LPWM 19

#define LEFT_ENC_A 34
#define LEFT_ENC_B 35
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33

#define LED_1 23

#define SERVO_1 16
#define SERVO_2 17

// Task Handle
TaskHandle_t TaskControlHandle;
TaskHandle_t TaskRemoteHandle;
void TaskControl(void *pvParameters);
void TaskRemote(void *pvParameters);

// MUtex variable
SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t pidDataMutex;

const char* esp_blu_MAC = "a0:b7:65:14:b7:ae";

double Kp = 10; //80
double Ki = 0;
double Kd = 0;

double Kp_orient = 0; //1.2
double Ki_orient = 0; // 6.4
double Kd_orient = 0; //0.528

double setpoint = 0;
float deadband = 2;
double input, output;
volatile bool dmp_ready;
double orientation = 0;
double target_orient = 0;
double output_orient = 0;
bool init_orient = false;
uint8_t serial_tuneOpt = 0;
bool isRemoteConnected = false;
bool isRobotFall = false;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64] = {0};

pcnt_unit_t pcnt_unit_left = PCNT_UNIT_0;
pcnt_unit_t pcnt_unit_right = PCNT_UNIT_1;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
PID pid_orient(&orientation, &output_orient, &target_orient, Kp_orient, Ki_orient, Kd_orient, DIRECT);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
MPU6050 mpu;

void ServoMoveUp();
void ServoMoveDown();

void IRAM_ATTR dmpDataReady() {
  dmp_ready = true;
}

class Motor {
  public:
    bool _isReversed = false;
    uint8_t _rpwm_pin, _lpwm_pin;
    uint8_t _rpwm_channel = 0, _lpwm_channel = 0;
    uint8_t _enc_a_pin, _enc_b_pin;
    volatile long enc_count;
    long prevCount = 0;
    unsigned long prevTime = 0;
    unsigned long lastPidTime = 0;
    bool isEncInterrupt = false;
    float motor_rpm = 0;
    double _pid_input, _pid_output, _pid_setpoint = 0;
    double _Kp = 0, _Ki = 0, _Kd = 0;
    PID* Motor_pid;

    Motor(bool reversed, uint8_t rpwm_pin, uint8_t lpwm_pin, uint8_t enc_a_pin, uint8_t enc_b_pin) {
      _isReversed = reversed;
      _rpwm_pin = rpwm_pin;
      _lpwm_pin = lpwm_pin;
      _enc_a_pin = enc_a_pin;
      _enc_b_pin = enc_b_pin;  
      // Reset value on startup
      enc_count = 0;

      if(_rpwm_pin == LEFT_MOTOR_RPWM){
        ledcSetup(LEFT_PWM_CHANNEL_R, 5000, 8); // edit: tambah frekuensi 1000->5000
        ledcSetup(LEFT_PWM_CHANNEL_L, 5000, 8);
        ledcAttachPin(_rpwm_pin, LEFT_PWM_CHANNEL_R);
        ledcAttachPin(_lpwm_pin, LEFT_PWM_CHANNEL_L);
        _rpwm_channel = LEFT_PWM_CHANNEL_R;
        _lpwm_channel = LEFT_PWM_CHANNEL_L;

        // Configure PCNT
        pcnt_config_t pcnt_config_left = {
          .pulse_gpio_num = _enc_a_pin,
          .ctrl_gpio_num = _enc_b_pin,
          .lctrl_mode = PCNT_MODE_REVERSE,   // edit
          .hctrl_mode = PCNT_MODE_KEEP,
          .pos_mode = PCNT_COUNT_INC,
          .neg_mode = PCNT_COUNT_DEC,
          .counter_h_lim = 32767,
          .counter_l_lim = -32768,
          .unit = pcnt_unit_left,
          .channel = PCNT_CHANNEL_0
        };
        pcnt_unit_config(&pcnt_config_left);
        pcnt_counter_pause(pcnt_unit_left);
        pcnt_counter_clear(pcnt_unit_left);
        pcnt_counter_resume(pcnt_unit_left);

      }else if(_rpwm_pin == RIGHT_MOTOR_RPWM){
        ledcSetup(RIGHT_PWM_CHANNEL_R, 5000, 8);  // edit frekuensi
        ledcSetup(RIGHT_PWM_CHANNEL_L, 5000, 8);
        ledcAttachPin(_rpwm_pin, RIGHT_PWM_CHANNEL_R);
        ledcAttachPin(_lpwm_pin, RIGHT_PWM_CHANNEL_L);
        _rpwm_channel = RIGHT_PWM_CHANNEL_R;
        _lpwm_channel = RIGHT_PWM_CHANNEL_L;

        // Configure PCNT
        pcnt_config_t pcnt_config_right = {
          .pulse_gpio_num = _enc_a_pin,
          .ctrl_gpio_num = _enc_b_pin,
          .lctrl_mode = PCNT_MODE_REVERSE,
          .hctrl_mode = PCNT_MODE_KEEP,
          .pos_mode = PCNT_COUNT_INC,
          .neg_mode = PCNT_COUNT_DEC,
          .counter_h_lim = 32767,
          .counter_l_lim = -32768,
          .unit = pcnt_unit_right,
          .channel = PCNT_CHANNEL_0
        };
        pcnt_unit_config(&pcnt_config_right);
        pcnt_counter_pause(pcnt_unit_right);
        pcnt_counter_clear(pcnt_unit_right);
        pcnt_counter_resume(pcnt_unit_right);
      }
    
      pinMode(_enc_a_pin, INPUT_PULLUP);
      pinMode(_enc_b_pin, INPUT_PULLUP);

      Motor_pid = new PID(&_pid_input, &_pid_output, &_pid_setpoint, _Kp, _Ki, _Kd, DIRECT);
      Motor_pid->SetMode(AUTOMATIC);
      Motor_pid->SetSampleTime(10); // 50ms same as encoder pooling rate
      Motor_pid->SetOutputLimits(0, 255);
    }

    float ReadMotorSpeed(){
      int16_t count = 0;
      if(_enc_a_pin == LEFT_ENC_A){
        pcnt_get_counter_value(pcnt_unit_left, &count);
      } else {
        pcnt_get_counter_value(pcnt_unit_right, &count);
      }
      
      // Read every 10 ms
      unsigned long now = millis();
      unsigned long elapsed = now - prevTime; 
      if (elapsed >= 10) {
        long delta = count - prevCount;
        if (delta > 32767) delta -= 65536;
        else if (delta < -32767) delta += 65536;

        float rpm = (delta * 60000.0) / (elapsed * 44.0);
        motor_rpm = rpm;

        prevCount = count;
        prevTime = now;
      }
      return motor_rpm;
    }

    void SetMotorSpeed(float _speed){
      _pid_setpoint = abs(_speed);
      
      // Only compute PID at appropriate intervals
      unsigned long now = millis();
      if (now - lastPidTime >= 10) { // Match PID sample time
        _pid_input = abs(ReadMotorSpeed());
        Motor_pid->Compute();
        lastPidTime = now;
      }

      int pwm_output = constrain((int)_pid_output, 0, 255);
      if (_speed > 0) {
        MoveForward(pwm_output);
      } else if (_speed < 0) {
        MoveBackward(pwm_output);
      } else {
        MotorStop();
      }
    }

    void MoveForward(int _pwm){
      if (_isReversed) {
        ledcWrite(_rpwm_channel, _pwm);
        ledcWrite(_lpwm_channel, 0);
      } else {
        ledcWrite(_rpwm_channel, 0);
        ledcWrite(_lpwm_channel, _pwm);
      }
    }

    void MoveBackward(int _pwm){
      if (_isReversed) {
        ledcWrite(_rpwm_channel, 0);
        ledcWrite(_lpwm_channel, _pwm);
      } else {
        ledcWrite(_rpwm_channel, _pwm);
        ledcWrite(_lpwm_channel, 0);
      }
    }

    void MotorStop(){
      ledcWrite(_rpwm_channel, 0);
      ledcWrite(_lpwm_channel, 0);
      Motor_pid->SetMode(MANUAL);
      _pid_output = 0;
      Motor_pid->SetMode(AUTOMATIC);
    }

    void SetKpid(double Kp, double Ki, double Kd) {
      _Kp = Kp;
      _Ki = Ki;
      _Kd = Kd;
      Motor_pid->SetTunings(_Kp, _Ki, _Kd);
    }

  private:
    float Median3(float a, float b, float c){
      if (a > b) { float temp=a; a=b; b=temp; }
      if (b > c) { float temp=b; b=c; c=temp; }
      if (a > b) { float temp=a; a=b; b=temp; }

      return b;
    }

    float FilterRPM(float raw){
      static float v1 = 0, v2 = 0;
      static float ema = 0;
      static bool init = false;

      float med = Median3(raw, v1, v2);
      v2 = v1; v1 = raw;
      // EMA smoothing
      float alpha = 0.25; // 0.1 = smoother, 0.3 = faster response
      if (!init) { ema = med; init = true; }
      ema += alpha * (med - ema);

      return ema;
    }
};

Motor leftMotor(true, LEFT_MOTOR_RPWM, LEFT_MOTOR_LPWM, LEFT_ENC_A, LEFT_ENC_B);
Motor rightMotor(false, RIGHT_MOTOR_RPWM, RIGHT_MOTOR_LPWM, RIGHT_ENC_A, RIGHT_ENC_B);

class LedRuntime {
  public:
    uint8_t led = 0;
    long lastFlashing = 0;
    bool lastState = false;

    LedRuntime(uint8_t _gpio){
      this->led = _gpio;
      pinMode(this->led, OUTPUT);
    }
    void TurnOn(){
      digitalWrite(this->led, HIGH);
    }
    void TurnOFF(){
      digitalWrite(this->led, LOW);
    }
    void FlashingLed(unsigned long _period){
      unsigned long now = millis();
      if (now - lastFlashing >= _period) {
        lastFlashing = now;
        lastState = !lastState;
        digitalWrite(this->led, lastState ? HIGH : LOW);
      }
    }
};

LedRuntime Led_1(LED_1);

// Servo object
Servo Servo_1;
Servo Servo_2;
Servo Servo_3;

// uint8_t* setMotorMoveRatio(uint8_t _ma_ratio, uint8_t _mb_ratio, uint8_t _speed, Motor& rightMotor, Motor& leftMotor) {
//   static uint8_t motor_speed[2] = {0, 0};

//   float mr_speed = abs(rightMotor.ReadMotorSpeed());
//   float ml_speed = abs(leftMotor.ReadMotorSpeed());

//   // target = rata-rata biar motor sinkron
//   float target = (mr_speed + ml_speed) / 2.0;

//   float Kp = 5;  // coba kecil dulu
//   float error_r = target - mr_speed;
//   float error_l = target - ml_speed;

//   int pwm_r = _speed + (int)(Kp * error_r);
//   int pwm_l = _speed + (int)(Kp * error_l);

//   pwm_r = constrain(abs(pwm_r), 0, 255);
//   pwm_l = constrain(abs(pwm_l), 0, 255);

//   motor_speed[0] = (uint8_t)pwm_r; // kanan
//   motor_speed[1] = (uint8_t)pwm_l; // kiri

//   return motor_speed;
// }

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
  isRemoteConnected = true;
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
  isRemoteConnected = false;
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
  if (ctl->buttons() & 0x0010) {
    // code for when L1 button is pushed
    ServoMoveUp();
  }
  if (ctl->buttons() != 0x0010) {
    // code for when L1 button is released
  }

  //== PS4 L2 trigger button = 0x0040 ==//
  if (ctl->buttons() & 0x0040) {
    // code for when L2 button is pushed
    ServoMoveDown();
  }
  if (ctl->buttons() != 0x0040) {
    // code for when L2 button is released
  }

  //== LEFT JOYSTICK - UP ==//
  if (ctl->axisY() <= -25) {
    // code for when left joystick is pushed up
    setpoint = 2; 
  }

  //== LEFT JOYSTICK - DOWN ==//
  if (ctl->axisY() >= 25) {
    // code for when left joystick is pushed down
    setpoint = -2;
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
    setpoint = 0;
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

String serialInput = "";
bool newCommand = false;

void SerialPIDTune() {
  // Read all available characters without blocking
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      newCommand = true;
      break;
    } else {
      serialInput += c;
    }
    
    // Small delay to allow buffer to fill
    delay(1);
  }

  // Process the command if we have a complete one
  if (newCommand) {
    serialInput.trim();
    
    if(serialInput.startsWith("Orient")){
      serial_tuneOpt = 100;
    }else if(serialInput.startsWith("Balance")){
      serial_tuneOpt = 101;
    }
    if (serialInput.startsWith("Kp=")) {
      if(serial_tuneOpt == 101){
        Kp = serialInput.substring(3).toFloat();
        pid.SetTunings(Kp, Ki, Kd);
        Serial.print("Updated Kp = "); Serial.println(Kp);
      }else if (serial_tuneOpt == 100){
        Kp_orient = serialInput.substring(3).toFloat();
        pid_orient.SetTunings(Kp_orient, Ki_orient, Kd_orient);
        Serial.print("Updated Kp = "); Serial.println(Kp_orient);
      }
    } 
    else if (serialInput.startsWith("Ki=")) {
      if(serial_tuneOpt == 101){
        Ki = serialInput.substring(3).toFloat();
        pid.SetTunings(Kp, Ki, Kd);
        Serial.print("Updated Ki = "); Serial.println(Ki);
      }else if (serial_tuneOpt == 100){
        Ki_orient = serialInput.substring(3).toFloat();
        pid_orient.SetTunings(Kp_orient, Ki_orient, Kd_orient);
        Serial.print("Updated Ki = "); Serial.println(Ki_orient);
      }
    } 
    else if (serialInput.startsWith("Kd=")) {
      if(serial_tuneOpt == 101){
        Kd = serialInput.substring(3).toFloat();
        pid.SetTunings(Kp, Ki, Kd);
        Serial.print("Updated Kd = "); Serial.println(Kd);
      }else if (serial_tuneOpt == 100){
        Kd_orient = serialInput.substring(3).toFloat();
        pid_orient.SetTunings(Kp_orient, Ki_orient, Kd_orient);
        Serial.print("Updated Kd = "); Serial.println(Kd_orient);
      }
    }
    
    serialInput = "";
    newCommand = false;
  }
}

float NormalizeYawAngle(float angle){
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

float YawAngleDifferece(float a, float b){
  float diff = a - b;
  return NormalizeYawAngle(diff);
}

void setup() {
  // Mutex init
  imuDataMutex = xSemaphoreCreateMutex();
  pidDataMutex = xSemaphoreCreateMutex();
  
  // Task init
  xTaskCreatePinnedToCore(
    TaskControl,         
    "TaskControl",       
    4096,                
    NULL,          
    2,                   
    &TaskControlHandle,  
    1                    // Core 1
  );

  xTaskCreatePinnedToCore(
    TaskRemote,
    "TaskRemote",
    4096,
    NULL,
    1,                 
    &TaskRemoteHandle,
    0                    // Core 0
  );

  Serial.begin(115200);
  Wire.begin(IMU_SDA, IMU_SCL);
  isRemoteConnected = false;

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  Serial.println("Inisialisasi MPU6050...");
  mpu.initialize();
  mpu.setXGyroOffset(157);
  mpu.setYGyroOffset(70);
  mpu.setZGyroOffset(-107);
  mpu.setXAccelOffset(342);
  mpu.setYAccelOffset(154);
  mpu.setZAccelOffset(216);

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
  pid.SetOutputLimits(-2200, 2200); // Max Motor Speed

  pid_orient.SetMode(AUTOMATIC);
  pid_orient.SetSampleTime(10);
  pid_orient.SetOutputLimits(-1500, 1500);

  rightMotor.SetKpid(0.14, 0.8, 0.002);
  leftMotor.SetKpid(0.14, 0.8, 0.002);

  Servo_1.attach(SERVO_1);
  Servo_2.attach(SERVO_2);
}

void ServoMoveUp(){
  Servo_1.write(180);
}

void ServoMoveDown(){
  Servo_1.write(0);
}

void TaskControl(void *pvParameters){
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS; // 200 Hz
  double lastLog = 0;
  for(;;){
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
    if(dmp_ready){
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float robot_angle = ypr[1] * 180/M_PI; // pitch
        orientation = ypr[0] * 180/M_PI;

        input = robot_angle;
        if(init_orient == false){
          target_orient = orientation;
          init_orient = true;
        }
      }
      dmp_ready = false;
    }else{
      pid.Compute();
      pid_orient.Compute();
      if (xSemaphoreTake(pidDataMutex, portMAX_DELAY) == pdTRUE){
        if (input > -50 && input < 50){
          isRobotFall = false;
          float left_speed = output;
          float right_speed = output;

          float orient_error = YawAngleDifferece(target_orient, orientation);
          if(fabs(orient_error) > deadband){
            if(orient_error > 0){
              left_speed -= output_orient;
              right_speed += output_orient;
            }else{
              left_speed += output_orient;
              right_speed -= output_orient;
            }
          }
          rightMotor.SetMotorSpeed(right_speed);
          leftMotor.SetMotorSpeed(left_speed);
        }else{
          rightMotor.MotorStop();
          leftMotor.MotorStop();
          // Reset orientation after fall
          init_orient = false;
          isRobotFall = true;
        }

        xSemaphoreGive(pidDataMutex);
      }
    }

    // rightMotor.SetMotorSpeed(1500);
    // leftMotor.SetMotorSpeed(1500);
    // Serial debugging here
    // Serial.print(millis());
    // Serial.print(",");
    // Serial.print(rightMotor._pid_output);
    // Serial.print(",");
    // Serial.println(leftMotor._pid_output);
    
    Serial.print(millis());
    Serial.print(",");
    Serial.print(leftMotor._pid_input);
    Serial.print(",");
    Serial.println(rightMotor._pid_input);

    // Serial.print(target_orient);
    // Serial.print(",");
    // Serial.println(orientation);

  }
}

void TaskRemote(void *pvParameters){
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // 100 Hz

  for(;;){
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    bool dataUpdated = BP32.update();
    if (dataUpdated) processControllers();

    if(isRobotFall){
      Led_1.FlashingLed(200);
    }else{
      Led_1.TurnOFF();
      if(isRemoteConnected){
        Led_1.TurnOn();
      }else{
        Led_1.FlashingLed(1000);
      }
    }
    SerialPIDTune(); 
  } 
}

void loop() {
  vTaskDelay(portMAX_DELAY); 
}