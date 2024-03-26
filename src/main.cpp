#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_MAX 120
#define SERVO_MIN 60
#define SERVO_MID ((SERVO_MAX + SERVO_MIN) / 2)
#define MOTOR_PIN 14

// Connecion with master
HardwareSerial hs(1);

#define BATTERY_REPORT_RATE 500
unsigned long last_battery_report = 0;

enum SerialCommands {
  SerialMotor,
  SerialServo,
  SerialEncoder,
  SerialBattery
};

Servo servo;

float target_speed = 0;
float current_speed = 0;
float elapsed_time;
unsigned long current_time, previous_time;

unsigned long next_motor_write = 0;

// PID parameters
float Kp = 2, Ki = 1, Kd = 1;

// PID variables
float error, last_error, cum_error, rate_error;

volatile unsigned int encoder_diff = 0;

void encoderInt() {
  encoder_diff++;
}

float computeMotorSpeed() {
  error = target_speed - current_speed;
  cum_error += error * elapsed_time;
  rate_error = (error - last_error) / elapsed_time;

  float output = Kp * error + Ki * cum_error + Kd * rate_error;

  last_error = error;

  return output;
}

void setup() {
  attachInterrupt(32, encoderInt, RISING);
  hs.begin(1000000, SERIAL_8N1, 4, 2);
  Serial.begin(9600);
  servo.attach(13);

  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(14, OUTPUT);

  // Controller direction and enable
  digitalWrite(25, HIGH);
  digitalWrite(26, HIGH);
  digitalWrite(27, LOW);
}

void loop() {
  // receive data from master
  if (hs.available()) {
    int header = hs.read();
    switch (header) {
      case SerialMotor:
        {
          int speed = 0;
          hs.readBytes((uint8_t *)&speed, sizeof(int));
          target_speed = speed / 100.0f;
        }
      case SerialServo:
        {
          int angle = 0;
          hs.readBytes((uint8_t *)&angle, sizeof(int));
          servo.write(constrain(angle, SERVO_MIN, SERVO_MAX));  // Shouldn't this be executed periodically along motor_speed?
        }
    }
  }

  // update motor speed
  if (millis() >= next_motor_write) {
    // calculate speed
    current_time = millis();
    elapsed_time = (float)(current_time - previous_time) + 0.00001F;  // calculate elapsed time
    previous_time = current_time;

    current_speed = encoder_diff / elapsed_time;

    hs.write(SerialEncoder);
    hs.write((uint8_t *)&encoder_diff, sizeof(int));

    encoder_diff = 0;
    int motor_speed = computeMotorSpeed();
    analogWrite(14, motor_speed);
    next_motor_write = next_motor_write + 10;
  }

  if ((millis() - last_battery_report) > BATTERY_REPORT_RATE) {
    hs.write(SerialBattery);
    float voltage = analogRead(36) / 430.0f;  // WARN: This conversion factor isn't very precise
    hs.write((uint8_t *)&voltage, sizeof(float));
  }
}
