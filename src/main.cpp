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
#define RIGHT_ENC_A 33
#define RIGHT_ENC_B 32

#define LED_1 23

#define SERVO_1 16
#define SERVO_2 17

#define LMOTOR_KP 2280
#define LMOTOR_KI 62645
#define LMOTOR_KD 19.15

#define RMOTOR_KP 2280
#define RMOTOR_KI 62645
#define RMOTOR_KD 20.8

#define RMOTOR_ALPHA 0.2
#define LMOTOR_ALPHA 0.2

// Task Handle
TaskHandle_t TaskControlHandle;
TaskHandle_t TaskRemoteHandle;
void TaskControl(void *pvParameters);
void TaskRemote(void *pvParameters);

// MUtex variable
SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t pidDataMutex;

const char* esp_blu_MAC = "a0:b7:65:14:b7:ae";

double Kp_balance = 0.008;                                                       // 20
double Ki_balance = 0;                                                      // 240
double Kd_balance = 0;                                                      // 1.2

double Kp_speed = 1;
double Ki_speed = 0;
double Kd_speed = 0;

const float WHEEL_DIAMETER = 68;                                              // in mm
const float MM_PER_COUNT = (3.14159 * WHEEL_DIAMETER) / 44.0;                 // 44 counts per revolution
const float MOTOR_L_RATIO = 9.70;                                             // NOTE: I check it from count the physical gear teeth
const float MOTOR_R_RATIO = 9.70;

double setpoint = 0;
float deadband = 1.25;
double input_balance, output_balance;
double output_speed;
int16_t estimated_speed = 0;
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

double throttle = 0;
float max_throttle = 1.0;
float steering = 0;
float max_steering = 180;

double target_angle = 0;
float left_motor_speed = 0;
float right_motor_speed = 0;
double actual_robot_speed = 0;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64] = {0};

pcnt_unit_t pcnt_unit_left = PCNT_UNIT_0;
pcnt_unit_t pcnt_unit_right = PCNT_UNIT_1;

PID pid_balance(&input_balance, &output_balance, &target_angle, Kp_balance, Ki_balance, Kd_balance, DIRECT);
PID pid_speed(&actual_robot_speed, &output_speed, &throttle, Kp_speed, Ki_speed, Kd_speed, DIRECT);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
MPU6050 mpu;

void ServoMoveUp();
void ServoMoveDown();
float NormalizeYawAngle(float);
float ExponentialResponse(int, float, float);
float SmoothInput(float, float, float);

void IRAM_ATTR dmpDataReady() {
  dmp_ready = true;
}

class Motor {
  public:
    bool _isReversed = false;
    uint8_t _rpwm_pin, _lpwm_pin;
    uint8_t _rpwm_channel = 0, _lpwm_channel = 0;
    uint8_t _enc_a_pin, _enc_b_pin;
    volatile long enc_ext = 0;
    volatile long absolute_position = 0;
    long prevCount = 0; 
    unsigned long prevTime = 0;
    unsigned long lastPidTime = 0;
    bool isEncInterrupt = false;
    float motor_rpm = 0;                                  // raw rpm from encoder
    float shaft_rpm = 0;                                  // rpm after gear reduction
    float wheel_speed = 0;                                // m/s
    double _pid_input, _pid_output, _pid_setpoint = 0;
    double _Kp = 0, _Ki = 0, _Kd = 0;
    float motor_rpm_filtered = 0;
    bool rpm_filter_init = false;
    float wheel_diameter_mm = 0;
    float filter_alpha = 0.1;
    float gear_ratio = 1.0;
    PID* Motor_pid;

    Motor(bool reversed, uint8_t rpwm_pin, uint8_t lpwm_pin, uint8_t enc_a_pin, uint8_t enc_b_pin) {
      _isReversed = reversed;
      _rpwm_pin = rpwm_pin;
      _lpwm_pin = lpwm_pin;
      _enc_a_pin = enc_a_pin;
      _enc_b_pin = enc_b_pin;  
      // Reset value on startup
      enc_ext = 0;

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
          .lctrl_mode = PCNT_MODE_REVERSE,  
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
        ledcSetup(RIGHT_PWM_CHANNEL_R, 5000, 8); 
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

    float ReadMotorSpeed() {
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
        motor_rpm = FilterRPM(rpm, filter_alpha);
        shaft_rpm = GearRatio(motor_rpm, gear_ratio);
        wheel_speed = rpm_to_ms(shaft_rpm, wheel_diameter_mm / 2.0);

        prevCount = enc_ext;
        prevTime = now;
      }
      return wheel_speed;
    }

    void SetMotorSpeed(float _speed){
      _pid_setpoint = abs(_speed);
      
      // Only compute PID at appropriate intervals
      unsigned long now = millis();
      if (now - lastPidTime >= 10) { // Match PID sample time
        _pid_input = abs(this->wheel_speed);
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

    void SetWheelDiameter(float diameter_mm) {
      wheel_diameter_mm = diameter_mm;
    }

    void SetGearRatio(float ratio) {
      gear_ratio = ratio;
    }

    void SetFilterAlpha(float alpha) {
      filter_alpha = alpha;
    }

  private:
  float FilterRPM(float new_rpm, float alpha) {
    if (!rpm_filter_init) {
        motor_rpm_filtered = new_rpm; 
        rpm_filter_init = true;
    } else {
        motor_rpm_filtered = alpha * new_rpm + (1 - alpha) * motor_rpm_filtered;
    }
    return motor_rpm_filtered;
  }

    float rpm_to_ms(float rpm, float radius_mm) {
      return (rpm / 60.0f) * (2.0f * PI * (radius_mm / 1000.0f));
    }

    float GearRatio(float motor_rpm, float out_ratio) {
      if (motor_rpm == 0) return 0;
      return motor_rpm / out_ratio;
    }
};

Motor leftMotor(true, LEFT_MOTOR_RPWM, LEFT_MOTOR_LPWM, LEFT_ENC_A, LEFT_ENC_B);
Motor rightMotor(true, RIGHT_MOTOR_RPWM, RIGHT_MOTOR_LPWM, RIGHT_ENC_A, RIGHT_ENC_B);

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
  }

  //== PS4 Square button = 0x0004 ==//
  if (ctl->buttons() & 0x0004) {
    // Square button pressed
    position_hold_enabled = false;
    Serial.println("Position hold DISENGAGED");
  }

  //== PS4 L1 trigger = 0x0010 ==//
  if (ctl->buttons() & 0x0010) {
    // L1 pressed
    isLifterDown = false;
    isLifterUp = true;
  } 

  //== LEFT JOYSTICK ==//
  int ly = ctl->axisY();
  if(abs(ly) > 25){
    float throttle_raw = ExponentialResponse(ly, max_throttle, 1.6);
    throttle = SmoothInput(throttle_raw, throttle, 0.2);
  } else {
    throttle = 0;
  }

  //== RIGHT JOYSTICK ==//
  int rx = ctl->axisRX();
  if (rx != 0) {
    if(abs(rx) > 25){
      float steering_raw = ExponentialResponse(rx, max_steering, 1.2);
      steering = SmoothInput(steering_raw, steering, 0.2);
    } else {
      steering = 0;
    }
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
    if(serialInput.startsWith("Balance")){
      serial_tuneOpt = 100;
    }else if(serialInput.startsWith("Speed")){
      serial_tuneOpt = 101;
    }else if(serialInput == "Motor_r"){
      serial_tuneOpt = 102;
    }else if(serialInput == "Motor_l"){
      serial_tuneOpt = 103;
    }

    if (serialInput.startsWith("Kp=")) {
      if(serial_tuneOpt == 100){
        Kp_balance = serialInput.substring(3).toFloat();
        pid_balance.SetTunings(Kp_balance, Ki_balance, Kd_balance);
        Serial.print("Updated Kp = "); Serial.println(Kp_balance);
      }else if(serial_tuneOpt == 101){
        Kp_speed = serialInput.substring(3).toFloat();
        pid_speed.SetTunings(Kp_speed, Ki_speed, Kd_speed);
        Serial.print("Updated Kp = "); Serial.println(Kp_speed);
      }
      else if (serial_tuneOpt == 102){
        float motor1_kp = serialInput.substring(3).toFloat();
        rightMotor.SetKpid(motor1_kp, rightMotor._Ki, rightMotor._Kd);
        Serial.print("Updated Motor 1 Kp = "); Serial.println(motor1_kp);
      }else if (serial_tuneOpt == 103){
        float motor2_kp = serialInput.substring(3).toFloat();
        leftMotor.SetKpid(motor2_kp, leftMotor._Ki, leftMotor._Kd);
        Serial.print("Updated Motor 2 Kp = "); Serial.println(motor2_kp);
      }
    } 
    else if (serialInput.startsWith("Ki=")) {
      if(serial_tuneOpt == 100){
        Ki_balance = serialInput.substring(3).toFloat();
        pid_balance.SetTunings(Kp_balance, Ki_balance, Kd_balance);
        Serial.print("Updated Ki = "); Serial.println(Ki_balance);
      }else if(serial_tuneOpt == 101){
        Ki_speed = serialInput.substring(3).toFloat();
        pid_speed.SetTunings(Kp_speed, Ki_speed, Kd_speed);
        Serial.print("Updated Ki = "); Serial.println(Ki_speed);
      }
      else if (serial_tuneOpt == 102){
        float motor1_ki = serialInput.substring(3).toFloat();
        rightMotor.SetKpid(rightMotor._Kp, motor1_ki, rightMotor._Kd);
        Serial.print("Updated Motor 1 Ki = "); Serial.println(motor1_ki); 
      }else if (serial_tuneOpt == 103){
        float motor2_ki = serialInput.substring(3).toFloat();
        leftMotor.SetKpid(leftMotor._Kp, motor2_ki, leftMotor._Kd);
        Serial.print("Updated Motor 2 Ki = "); Serial.println(motor2_ki);
      }
    } 
    else if (serialInput.startsWith("Kd=")) {
      if(serial_tuneOpt == 100){
        Kd_balance = serialInput.substring(3).toFloat();
        pid_balance.SetTunings(Kp_balance, Ki_balance, Kd_balance);
        Serial.print("Updated Kd = "); Serial.println(Kd_balance);
      }else if(serial_tuneOpt == 101){
        Kd_speed = serialInput.substring(3).toFloat();
        pid_speed.SetTunings(Kp_speed, Ki_speed, Kd_speed);
        Serial.print("Updated Kd = "); Serial.println(Kd_speed);
      }
      else if (serial_tuneOpt == 102){
        float motor1_kd = serialInput.substring(3).toFloat();
        rightMotor.SetKpid(rightMotor._Kp, rightMotor._Ki, motor1_kd);
        Serial.print("Updated Motor 1 Kd = "); Serial.println(motor1_kd);
      }else if (serial_tuneOpt == 103){
        float motor2_kd = serialInput.substring(3).toFloat(); 
        leftMotor.SetKpid(leftMotor._Kp, leftMotor._Ki, motor2_kd);
        Serial.print("Updated Motor 2 Kd = "); Serial.println(motor2_kd);
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

float UnwrapAngle(double* current_angle, double* last_angle){
  if (*current_angle - *last_angle > 180) {
      rotation_count--;
  } else if (*current_angle - *last_angle < -180) {
      rotation_count++;
  }
  *last_angle = *current_angle;

  return *current_angle + (rotation_count * 360.0);
}

float ExponentialResponse(int input, float maxValue, float expVal = 1.5) {
  float normalized = input / 512.0;
  float sign = normalized < 0 ? -1 : 1;
  float exponential = sign * pow(abs(normalized), expVal);
  return exponential * maxValue;
}

float SmoothInput(float input, float prev, float alpha = 0.2) {
  return alpha * input + (1 - alpha) * prev;
}

void setup() {
  // Mutex init
  imuDataMutex = xSemaphoreCreateMutex();
  pidDataMutex = xSemaphoreCreateMutex();
  
  // Task init (Core 1)
  xTaskCreatePinnedToCore(
    TaskControl,         
    "TaskControl",       
    4096,                
    NULL,          
    2,                   
    &TaskControlHandle,  
    1                    
  );

  // Task init (Core 0)
  xTaskCreatePinnedToCore(
    TaskRemote,
    "TaskRemote",
    4096,
    NULL,
    1,                 
    &TaskRemoteHandle,
    0                   
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
  pid_speed.SetMode(AUTOMATIC);
  pid_speed.SetSampleTime(10);
  pid_speed.SetOutputLimits(-1000, 1000);
  pid_balance.SetMode(AUTOMATIC);
  pid_balance.SetSampleTime(10);
  pid_balance.SetOutputLimits(-1, 1);

  rightMotor.SetGearRatio(MOTOR_R_RATIO);
  rightMotor.SetWheelDiameter(WHEEL_DIAMETER);
  rightMotor.SetFilterAlpha(RMOTOR_ALPHA);
  leftMotor.SetGearRatio(MOTOR_L_RATIO);
  leftMotor.SetWheelDiameter(WHEEL_DIAMETER);
  leftMotor.SetFilterAlpha(LMOTOR_ALPHA);

  unsigned long last_toggle = 0;
  float curr_speed = 0.2;
  for(;;){
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
    // Speed calculated from encoders 
    rightMotor.ReadMotorSpeed();
    leftMotor.ReadMotorSpeed();
    right_motor_speed = rightMotor.wheel_speed;
    left_motor_speed = leftMotor.wheel_speed;
    float vel_enc = (right_motor_speed + left_motor_speed) / 2.0;
    actual_robot_speed = vel_enc;

    pid_speed.Compute();
    pid_balance.Compute();
    target_angle = 0;

    if(dmp_ready){
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];
        VectorInt16 raw_accel;
        VectorFloat linear_accel;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float robot_angle = ypr[1] * 180/M_PI; // pitch
        orientation = ypr[0] * 180/M_PI; // yaw
        orientation_unwrapped = UnwrapAngle(&orientation, &last_orientation);

        input_balance = robot_angle + angleCorrection;
      }
      dmp_ready = false;
    }else{
      if (xSemaphoreTake(pidDataMutex, portMAX_DELAY) == pdTRUE){
        if (input_balance > -50 && input_balance < 50){
          isRobotFall = false;
          float left_speed = output_balance;
          float right_speed = output_balance;

          rightMotor.SetMotorSpeed(left_speed);
          leftMotor.SetMotorSpeed(right_speed);
        }else{
          rightMotor.MotorStop();
          leftMotor.MotorStop();
          // Reset orientation after fall
          init_orient = false;
          isRobotFall = true;
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

    Serial.print(input_balance);
    Serial.print(",");
    Serial.print(output_balance);
    Serial.print(",");
    Serial.println(rightMotor.wheel_speed);

  }
}

void TaskRemote(void *pvParameters){
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // 200 Hz

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