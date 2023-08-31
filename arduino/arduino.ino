#include <CarPWMMotorControl.hpp>
#include <PID_v1.h>

#define PWM_MIN 0
#define PWMRANGE 255

const int lfEncoderPin = 21;  // Encoder pin for left-front motor
const int rfEncoderPin = 20;  // Encoder pin for right-front motor
const int lbEncoderPin = 19;  // Encoder pin for left-back motor
const int rbEncoderPin = 18;  // Encoder pin for right-back motor

volatile int lfEncoderCount = 0;  // Encoder count for left-front motor
volatile int rfEncoderCount = 0;  // Encoder count for right-front motor
volatile int lbEncoderCount = 0;  // Encoder count for left-back motor
volatile int rbEncoderCount = 0;  // Encoder count for right-back motor

unsigned long lastUpdateTime = 0;  // Time of last speed update
volatile float lfSpeed = 0.0;      // Left-front motor speed in rotations per second
volatile float rfSpeed = 0.0;      // Right-front motor speed in rotations per second
volatile float lbSpeed = 0.0;      // Left-back motor speed in rotations per second
volatile float rbSpeed = 0.0;      // Right-back motor speed in rotations per second

// PID parameters
double Kp = 1.6;  // Proportional gain
double Ki = 3.2;  // Integral gain
double Kd = 0;  // Derivative gain

// Variables for PID control
double lfInput, lfOutput;
double rfInput, rfOutput;
double lbInput, lbOutput;
double rbInput, rbOutput;


// Target speeds
double targetLfSpeed = 0;  // Target speed for left-front motor (rotations per second)
double targetRfSpeed = 0;  // Target speed for right-front motor (rotations per second)
double targetLbSpeed = 0;  // Target speed for left-back motor (rotations per second)
double targetRbSpeed = 0;  // Target speed for right-back motor (rotations per second)


PID lfPid(&lfInput, &lfOutput, &targetLfSpeed, Kp, Ki, Kd, DIRECT);
PID rfPid(&rfInput, &rfOutput, &targetRfSpeed, Kp, Ki, Kd, DIRECT);
PID lbPid(&lbInput, &lbOutput, &targetLbSpeed, Kp, Ki, Kd, DIRECT);
PID rbPid(&rbInput, &rbOutput, &targetRbSpeed, Kp, Ki, Kd, DIRECT);



PWMDcMotor lfMotor;
PWMDcMotor rfMotor;
PWMDcMotor lbMotor;
PWMDcMotor rbMotor;

struct twist {
  float x_speed;
  float y_speed;
  float r_speed;
};

twist current_twist = { 0, 0, 0 };

void setup() {
  // forward pin, back pin, pwm pin
  lfMotor.init(5, 6, 7);
  rfMotor.init(13, 11, 12);
  lbMotor.init(3, 4, 2);
  rbMotor.init(9, 10, 8);

  Serial3.begin(9600);
  Serial.begin(9600);

  pinMode(lfEncoderPin, INPUT_PULLUP);
  pinMode(rfEncoderPin, INPUT_PULLUP);
  pinMode(lbEncoderPin, INPUT_PULLUP);
  pinMode(rbEncoderPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(lfEncoderPin), updateLfEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rfEncoderPin), updateRfEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(lbEncoderPin), updateLbEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rbEncoderPin), updateRbEncoder, RISING);

  lfPid.SetMode(AUTOMATIC);
  rfPid.SetMode(AUTOMATIC);
  lbPid.SetMode(AUTOMATIC);
  rbPid.SetMode(AUTOMATIC);
}


void loop() {
  if (millis() - lastUpdateTime >= 10) {
    lfSpeed = (lfEncoderCount / 360.0) / ((millis() - lastUpdateTime) / 1000.0);
    rfSpeed = (rfEncoderCount / 360.0) / ((millis() - lastUpdateTime) / 1000.0);
    lbSpeed = (lbEncoderCount / 360.0) / ((millis() - lastUpdateTime) / 1000.0);
    rbSpeed = (rbEncoderCount / 360.0) / ((millis() - lastUpdateTime) / 1000.0);

    lastUpdateTime = millis();
    lfEncoderCount = 0;
    rfEncoderCount = 0;
    lbEncoderCount = 0;
    rbEncoderCount = 0;
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read the incoming line

    float values[3];
    int count = 0;
    int index = 0;

    while (count < 3 && index < input.length()) {
      if (input[index] == ',') {
        values[count] = input.substring(0, index).toFloat();
        input = input.substring(index + 1);
        count++;
        index = 0;
      } else {
        index++;
      }
    }

    if (count == 2 && index <= input.length()) {
      values[count] = input.substring(0, index).toFloat();
      count++;
    }

    if (count == 3) {

      // // Do something with the parsed values
      // // For example, print them to the serial monitor
      for (int i = 0; i < 3; i++) {
        Serial.print("Value ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(values[i]);
      }
      current_twist.x_speed = values[0];
      current_twist.y_speed = values[1];
      current_twist.r_speed = values[2];
    } else {
      // Invalid input, print an error message
      Serial.println("Invalid input! Expecting: <x_speed>,<y_speed>,<r_speed>");
    }
  }
  handleMovement();
  // printDebugInfo();
}



void handleMovement() {
  // if (!movement || updating)
  //   return;

  // Mecanum drive:
  // ------------------------

  // Taken and simplified from: http://robotsforroboticists.com/drive-kinematics/
  float lf = current_twist.x_speed - current_twist.y_speed - current_twist.r_speed * 0.5;
  float rf = current_twist.x_speed + current_twist.y_speed + current_twist.r_speed * 0.5;
  float lb = current_twist.x_speed + current_twist.y_speed - current_twist.r_speed * 0.5;
  float rb = current_twist.x_speed - current_twist.y_speed + current_twist.r_speed * 0.5;

  // Map values to PWM intensities.
  // PWMRANGE = full speed, PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lfPwm = mapPwm(fabs(lf), PWM_MIN, PWMRANGE);
  uint16_t rfPwm = mapPwm(fabs(rf), PWM_MIN, PWMRANGE);
  uint16_t lbPwm = mapPwm(fabs(lb), PWM_MIN, PWMRANGE);
  uint16_t rbPwm = mapPwm(fabs(rb), PWM_MIN, PWMRANGE);

  targetLfSpeed = lfPwm;
  targetRfSpeed = rfPwm;
  targetLbSpeed = lbPwm;
  targetRbSpeed = rbPwm;

  lfInput = lfSpeed * 64; // map(lfSpeed, 0, 4, PWM_MIN, PWMRANGE);
  rfInput = rfSpeed * 64; // map(rfSpeed, 0, 4, PWM_MIN, PWMRANGE);
  lbInput = lbSpeed * 64; // map(lbSpeed, 0, 4, PWM_MIN, PWMRANGE);
  rbInput = rbSpeed * 64; // map(rbSpeed, 0, 4, PWM_MIN, PWMRANGE);
  
  // Compute PID outputs
  lfPid.Compute();
  rfPid.Compute();
  lbPid.Compute();
  rbPid.Compute();

  lfMotor.setSpeedPWMAndDirection(lfOutput, (lf > 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD);
  rfMotor.setSpeedPWMAndDirection(rfOutput, (rf > 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD);
  lbMotor.setSpeedPWMAndDirection(lbOutput, (lb > 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD);
  rbMotor.setSpeedPWMAndDirection(rbOutput, (rb > 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD);


  // movement = false;
}

void updateLfEncoder() {
  lfEncoderCount++;
}

void updateRfEncoder() {
  rfEncoderCount++;
}

void updateLbEncoder() {
  lbEncoderCount++;
}

void updateRbEncoder() {
  rbEncoderCount++;
}

void printDebugInfo() {
  // Serial.print(lfEncoderCount);
  // Serial.print(",");
  // Serial.print(rfEncoderCount);
  // Serial.print(",");
  // Serial.print(lbEncoderCount);
  // Serial.print(",");
  // Serial.print(rbEncoderCount);
  // Serial.print(",");
  // Serial.print("lfInput: ");
  Serial.print(lfInput);
  // Serial.print(",");
  // Serial.print(rfInput);
  // Serial.print(",");
  // Serial.print(lbInput);
  // Serial.print(",");
  // Serial.print(rbInput);
  Serial.print(",");
  Serial.print(lfOutput);
  Serial.print(",");
  Serial.print(targetLfSpeed);
  // Serial.print(",");
  // Serial.print(rfOutput);
  // Serial.print(",");
  // Serial.print(lbOutput);
  // Serial.print(",");
  // Serial.print(rbOutput);
  Serial.println();
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max) {
  return x * (out_max - out_min) + out_min;
}