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

#define LMOTOR_KP 0.12
#define LMOTOR_KI 4
#define LMOTOR_KD 0.0008
#define RMOTOR_KP 0.06
#define RMOTOR_KI 4
#define RMOTOR_KD 0.0004

// Task Handle
TaskHandle_t TaskControlHandle;
TaskHandle_t TaskRemoteHandle;
void TaskControl(void *pvParameters);
void TaskRemote(void *pvParameters);

// MUtex variable
SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t pidDataMutex;

const char* esp_blu_MAC = "a0:b7:65:14:b7:ae";

double Kp = 20; //80
double Ki = 240;
double Kd = 1.2;

double Kp_orient = 1.2; //1.2
double Ki_orient = 0; // 6.4
double Kd_orient = 0; //0.528

double Kp_position = 0.2;
double Ki_position = 0.05;
double Kd_position = 0.01;

const float WHEEL_DIAMETER = 68; // in mm
const float MM_PER_COUNT = (3.14159 * WHEEL_DIAMETER) / 44.0; // 44 counts per revolution

double setpoint = 0;
float deadband = 1.25;
double input, output;
volatile bool dmp_ready;
double orientation = 0;
double orientation_unwrapped = 0;
double last_orientation = 0;
long rotation_count = 0;
double target_orient = 0;
double output_orient = 0;
double orient_error = 0;
bool init_orient = false;
uint8_t serial_tuneOpt = 0;
bool isRemoteConnected = false;
bool isRobotFall = false;
bool isLifterUp = false;
bool isLifterDown = false;
float angleCorrection = -1.8;
float maxAngleOffset = 4.2;
float targetAngleOffset = 0.0;
double position_x= 0;
double target_position = 0;
double position_error = 0;
double position_output = 0;
bool position_hold_enabled = false;
float maxOrientationOffset = 16.0;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64] = {0};

pcnt_unit_t pcnt_unit_left = PCNT_UNIT_0;
pcnt_unit_t pcnt_unit_right = PCNT_UNIT_1;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
PID pid_orient(&orientation_unwrapped, &output_orient, &target_orient, Kp_orient, Ki_orient, Kd_orient, DIRECT);
PID pid_position(&position_x, &position_output, &target_position, Kp_position, Ki_position, Kd_position, DIRECT);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
MPU6050 mpu;

void ServoMoveUp();
void ServoMoveDown();
float NormalizeYawAngle(float);
float ExponentialResponse(int, float);

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
    volatile long absolute_position = 0;
    long prevCount = 0;
    unsigned long prevTime = 0;
    unsigned long lastPidTime = 0;
    bool isEncInterrupt = false;
    float motor_rpm = 0;
    double _pid_input, _pid_output, _pid_setpoint = 0;
    double _Kp = 0, _Ki = 0, _Kd = 0;
    long enc_ext = 0;
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
        ledcSetup(LEFT_PWM_CHANNEL_R, 5000, 8); // edit: tambah frekuensi 100->20000
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
        pcnt_counter_clear(pcnt_unit_left);
      } else {
        pcnt_get_counter_value(pcnt_unit_right, &count);
        pcnt_counter_clear(pcnt_unit_right);
      }

      enc_ext += count;
      absolute_position += count;
      
      // Read every 10 ms
      unsigned long now = millis();
      unsigned long elapsed = now - prevTime; 
      if (elapsed >= 10) {
        long delta = enc_ext - prevCount;
        if (delta > 32767) delta -= 65536;
        else if (delta < -32767) delta += 65536;

        float rpm = (delta * 60000.0) / (elapsed * 44.0);
        motor_rpm = rpm;

        prevCount = enc_ext;
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

    float GetTravelDistance(){
      if(_isReversed){
        return -absolute_position * MM_PER_COUNT;
      }else{
        return absolute_position * MM_PER_COUNT;
      }
    }

    void ResetTravelDistance(){
      absolute_position = 0;
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
  //== PS4 X button = 0x0001 ==//
  if (ctl->buttons() & 0x0001) {
    // X button pressed
    position_hold_enabled = true;
    target_position = position_x;
    Serial.println("Position hold ENGAGED");
  } else {
    // X button released
  }

  //== PS4 Square button = 0x0004 ==//
  if (ctl->buttons() & 0x0004) {
    // Square button pressed
    position_hold_enabled = false;
    Serial.println("Position hold DISENGAGED");
  } else {
    // Square button released
  }

  //== PS4 Triangle button = 0x0008 ==//
  if (ctl->buttons() & 0x0008) {
    // Triangle button pressed
  } else {
    // Triangle button released
  }

  //== PS4 Circle button = 0x0002 ==//
  if (ctl->buttons() & 0x0002) {
    // Circle button pressed
  } else {
    // Circle button released
  }

  //== Dpad UP = 0x01 ==//
  if (ctl->dpad() == DPAD_UP) {
    // Dpad up pressed
  }

  //== Dpad DOWN = 0x02 ==//
  if (ctl->dpad() == DPAD_DOWN) {
    // Dpad down pressed
  }

  //== Dpad LEFT = 0x08 ==//
  if (ctl->dpad() == DPAD_LEFT) {
    // Dpad left pressed
  }

  //== Dpad RIGHT = 0x04 ==//
  if (ctl->dpad() == DPAD_RIGHT) {
    // Dpad right pressed
  }

  //== PS4 R1 trigger = 0x0020 ==//
  if (ctl->buttons() & 0x0020) {
    // R1 pressed
  } else {
    // R1 released
  }

  //== PS4 R2 trigger = 0x0080 ==//
  if (ctl->buttons() & 0x0080) {
    // R2 pressed
  } else {
    // R2 released
  }

  //== PS4 L1 trigger = 0x0010 ==//
  if (ctl->buttons() & 0x0010) {
    // L1 pressed
    isLifterDown = false;
    isLifterUp = true;
  } else {
    // L1 released
  }

  //== PS4 L2 trigger = 0x0040 ==//
  if (ctl->buttons() & 0x0040) {
    // L2 pressed
    isLifterUp = false;
    isLifterDown = true;
  } else {
    // L2 released
  }

  //== LEFT JOYSTICK ==//
  int ly = ctl->axisY();
  if(abs(ly) > 25){
    targetAngleOffset = ExponentialResponse(ly, maxAngleOffset);
  } else {
    targetAngleOffset = 0.0;
  }

  //== RIGHT JOYSTICK ==//
  int rx = ctl->axisRX();
  if (rx != 0) {
    if(rx <= -50){
      target_orient -= ExponentialResponse(rx, maxOrientationOffset); // Turn Left
    }else if(rx >= 50){
      target_orient += ExponentialResponse(rx, maxOrientationOffset); // Turn Right
    }
  }
  if (ctl->axisRY() != 0) {
    // Right stick Y
  }
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
    }else if(serialInput.startsWith("Correct")){
      serial_tuneOpt = 102;
    }else if(serialInput == "Motor_r"){
      serial_tuneOpt = 103;
    }else if(serialInput == "Motor_l"){
      serial_tuneOpt = 104;
    }else if(serialInput.startsWith("Position")){
      serial_tuneOpt = 105; 
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
      }else if (serial_tuneOpt == 103){
        float motor1_kp = serialInput.substring(3).toFloat();
        rightMotor.SetKpid(motor1_kp, rightMotor._Ki, rightMotor._Kd);
        Serial.print("Updated Motor 1 Kp = "); Serial.println(motor1_kp, 4);
      }else if (serial_tuneOpt == 104){
        float motor2_kp = serialInput.substring(3).toFloat();
        leftMotor.SetKpid(motor2_kp, leftMotor._Ki, leftMotor._Kd);
        Serial.print("Updated Motor 2 Kp = "); Serial.println(motor2_kp, 4);
      }else if (serial_tuneOpt == 105){
        Kp_position = serialInput.substring(3).toFloat();
        pid_position.SetTunings(Kp_position, Ki_position, Kd_position);
        Serial.print("Updated Position Kp = "); Serial.println(Kp_position);
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
      }else if (serial_tuneOpt == 103){
        float motor1_ki = serialInput.substring(3).toFloat();
        rightMotor.SetKpid(rightMotor._Kp, motor1_ki, rightMotor._Kd);
        Serial.print("Updated Motor 1 Ki = "); Serial.println(motor1_ki, 4); 
      }else if (serial_tuneOpt == 104){
        float motor2_ki = serialInput.substring(3).toFloat();
        leftMotor.SetKpid(leftMotor._Kp, motor2_ki, leftMotor._Kd);
        Serial.print("Updated Motor 2 Ki = "); Serial.println(motor2_ki, 4);
      }else if (serial_tuneOpt == 105){
        Ki_position = serialInput.substring(3).toFloat();
        pid_position.SetTunings(Kp_position, Ki_position, Kd_position);
        Serial.print("Updated Position Ki = "); Serial.println(Ki_position);
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
      }else if (serial_tuneOpt == 103){
        float motor1_kd = serialInput.substring(3).toFloat();
        rightMotor.SetKpid(rightMotor._Kp, rightMotor._Ki, motor1_kd);
        Serial.print("Updated Motor 1 Kd = "); Serial.println(motor1_kd, 4);
      }else if (serial_tuneOpt == 104){
        float motor2_kd = serialInput.substring(3).toFloat(); 
        leftMotor.SetKpid(leftMotor._Kp, leftMotor._Ki, motor2_kd);
        Serial.print("Updated Motor 2 Kd = "); Serial.println(motor2_kd, 4);
      }else if (serial_tuneOpt == 105){
        Kd_position = serialInput.substring(3).toFloat();
        pid_position.SetTunings(Kp_position, Ki_position, Kd_position);
        Serial.print("Updated Position Kd = "); Serial.println(Kd_position);
      }
    }else if(serial_tuneOpt == 102){
      angleCorrection = serialInput.toFloat();
      Serial.print("Updated Angle Correction = "); Serial.println(angleCorrection);
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

float UnwrapAngle(double* current_angle, double* last_angle){
  if (*current_angle - *last_angle > 180) {
      rotation_count--;
  } else if (*current_angle - *last_angle < -180) {
      rotation_count++;
  }
  *last_angle = *current_angle;

  return *current_angle + (rotation_count * 360.0);
}

float ExponentialResponse(int input, float maxValue){
  float normalized = input / 512.0;
  float sign = normalized < 0 ? -1 : 1;
  float exponential = sign * pow(abs(normalized), 1.5);
  return exponential * maxValue;
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
  mpu.setXGyroOffset(139);
  mpu.setYGyroOffset(82);
  mpu.setZGyroOffset(-156);
  mpu.setXAccelOffset(-427);
  mpu.setYAccelOffset(185);
  mpu.setZAccelOffset(73);

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
  pid_orient.SetOutputLimits(-2200, 2200);

  rightMotor.SetKpid(RMOTOR_KP, RMOTOR_KI, RMOTOR_KD);
  leftMotor.SetKpid(LMOTOR_KP, LMOTOR_KI, LMOTOR_KD);

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
  pid_position.SetMode(AUTOMATIC);
  pid_position.SetSampleTime(50);
  pid_position.SetOutputLimits(-500, 500);
  for(;;){
    xTaskDelayUntil(&xLastWakeTime, xFrequency);

    float left_distance = leftMotor.GetTravelDistance();
    float right_distance = rightMotor.GetTravelDistance();
    position_x = (left_distance + right_distance) / 2.0;

    if (position_hold_enabled && !isRobotFall) {
      // Compute position correction
      pid_position.Compute();
      
      // Apply position correction to balance setpoint
      setpoint = targetAngleOffset + (position_output * 0.01); // Scale position correction
    } else {
      // Reset position integral when not in position hold mode
      pid_position.SetMode(MANUAL);
      pid_position.SetMode(AUTOMATIC);
      target_position = position_x; // Update target to current position
    }

    if(dmp_ready){
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float robot_angle = ypr[1] * 180/M_PI; // pitch
        orientation = ypr[0] * 180/M_PI; // yaw
        orientation_unwrapped = UnwrapAngle(&orientation, &last_orientation);

        input = robot_angle + angleCorrection;
        if (!position_hold_enabled) {
          setpoint = targetAngleOffset;
        }
        if(init_orient == false){
          target_orient = orientation_unwrapped;
          init_orient = true;
        }
      }
      dmp_ready = false;
    }else{
      pid.Compute();
      orient_error = target_orient - orientation_unwrapped;
      pid_orient.Compute();
      if (xSemaphoreTake(pidDataMutex, portMAX_DELAY) == pdTRUE){
        if (input > -50 && input < 50){
          isRobotFall = false;
          float left_speed = output;
          float right_speed = output;

          if(fabs(orient_error) > deadband){
            left_speed += output_orient;
            right_speed -= output_orient;
          }
          
          if (position_hold_enabled) {
            left_speed += position_output;
            right_speed += position_output;
          }

          rightMotor.SetMotorSpeed(right_speed);
          leftMotor.SetMotorSpeed(left_speed);
        }else{
          rightMotor.MotorStop();
          leftMotor.MotorStop();
          // Reset orientation after fall
          init_orient = false;
          isRobotFall = true;

          position_hold_enabled = false;
        }

        if(isLifterUp){
          ServoMoveUp();
          isLifterUp = false;
        }else if(isLifterDown){
          ServoMoveDown();
          isLifterDown = false;
        }
        xSemaphoreGive(pidDataMutex);
      }
    }

    // rightMotor.SetMotorSpeed(2000);
    // leftMotor.SetMotorSpeed(2000);
    
    // When debugging motor PID tuning
    // Serial.print(millis());
    // Serial.print(",");
    // Serial.print(rightMotor._pid_input);
    // Serial.print(",");
    // Serial.println(leftMotor._pid_input);
    
    // When debugging Balance PID tuning
    // Serial.print(millis());
    // Serial.print(",");
    // Serial.print(setpoint);
    // Serial.print(",");
    // Serial.println(input);

    // When debugging Orientation PID tuning
    Serial.print(orientation_unwrapped);
    Serial.print(",");
    Serial.println(target_orient);

    // When debugging Position PID tuning
    // Serial.print(left_distance);
    // Serial.print(",");
    // Serial.print(right_distance);
    // Serial.print(",");
    // Serial.println(position_x);
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