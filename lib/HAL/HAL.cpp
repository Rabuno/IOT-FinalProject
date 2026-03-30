#include "HAL.h"

namespace {

constexpr uint32_t servoPeriodUs() { return 1000000UL / SERVO_PWM_FREQ_HZ; }

uint32_t servoPulseUsFromAngle(uint8_t angleDeg) {
  if (angleDeg > 180) angleDeg = 180;
  const uint32_t span = static_cast<uint32_t>(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
  return static_cast<uint32_t>(SERVO_PULSE_MIN_US) + (span * angleDeg) / 180UL;
}

uint32_t servoDutyFromPulseUs(uint32_t pulseUs) {
  const uint32_t period = servoPeriodUs();
  const uint32_t maxDuty = (1UL << SERVO_PWM_RES_BITS) - 1UL;
  if (pulseUs > period) pulseUs = period;
  return (pulseUs * maxDuty) / period;
}

void setMotorDirPins(DriveCmd cmd) {
  switch (cmd) {
    case DriveCmd::FORWARD:
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
      digitalWrite(PIN_MOTOR_IN3, HIGH);
      digitalWrite(PIN_MOTOR_IN4, LOW);
      break;
    case DriveCmd::BACKWARD:
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
      digitalWrite(PIN_MOTOR_IN3, LOW);
      digitalWrite(PIN_MOTOR_IN4, HIGH);
      break;
    case DriveCmd::LEFT:
      // Hard left: left wheels backward, right wheels forward
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
      digitalWrite(PIN_MOTOR_IN3, HIGH);
      digitalWrite(PIN_MOTOR_IN4, LOW);
      break;
    case DriveCmd::RIGHT:
      // Hard right: left wheels forward, right wheels backward
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
      digitalWrite(PIN_MOTOR_IN3, LOW);
      digitalWrite(PIN_MOTOR_IN4, HIGH);
      break;
    case DriveCmd::STOP:
    default:
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, LOW);
      digitalWrite(PIN_MOTOR_IN3, LOW);
      digitalWrite(PIN_MOTOR_IN4, LOW);
      break;
  }
}

}  // namespace

namespace HAL {

void initPins() {
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);

  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);

  pinMode(PIN_LORA_AUX, INPUT);

  digitalWrite(PIN_US_TRIG, LOW);
  motorStop();
}

void initPwm() {
  // Motors
  ledcSetup(MOTOR_LEDC_CH_ENA, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
  ledcSetup(MOTOR_LEDC_CH_ENB, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_ENA, MOTOR_LEDC_CH_ENA);
  ledcAttachPin(PIN_MOTOR_ENB, MOTOR_LEDC_CH_ENB);
  ledcWrite(MOTOR_LEDC_CH_ENA, 0);
  ledcWrite(MOTOR_LEDC_CH_ENB, 0);

  // Servo
  ledcSetup(SERVO_LEDC_CH, SERVO_PWM_FREQ_HZ, SERVO_PWM_RES_BITS);
  ledcAttachPin(PIN_SERVO, SERVO_LEDC_CH);
  servoWriteAngle(SERVO_CENTER_DEG);
}

void motorApply(DriveCmd cmd, uint8_t duty) {
  if (duty == 0 || cmd == DriveCmd::STOP) {
    motorStop();
    return;
  }

  setMotorDirPins(cmd);
  ledcWrite(MOTOR_LEDC_CH_ENA, duty);
  ledcWrite(MOTOR_LEDC_CH_ENB, duty);
}

void motorStop() {
  setMotorDirPins(DriveCmd::STOP);
  ledcWrite(MOTOR_LEDC_CH_ENA, 0);
  ledcWrite(MOTOR_LEDC_CH_ENB, 0);
}

void servoWriteAngle(uint8_t angleDeg) {
  const uint32_t pulseUs = servoPulseUsFromAngle(angleDeg);
  const uint32_t duty = servoDutyFromPulseUs(pulseUs);
  ledcWrite(SERVO_LEDC_CH, duty);
}

uint16_t ultrasonicReadCm(uint32_t timeoutUs) {
  // Ensure a clean trigger pulse.
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  const uint32_t dur = pulseIn(PIN_US_ECHO, HIGH, timeoutUs);
  if (dur == 0) return 0;

  // Speed of sound approximation: cm = dur / 58 (for HC-SR04 compatible timings)
  const uint32_t cm = dur / 58UL;
  if (cm > 5000UL) return 0;
  return static_cast<uint16_t>(cm);
}

}  // namespace HAL
