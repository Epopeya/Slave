#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_MAX 120
#define SERVO_MIN 60
#define SERVO_MID ((SERVO_MAX + SERVO_MIN) / 2)
#define MOTOR_PIN 14

// Connecion with master
HardwareSerial hs(1);

Servo servo;

volatile unsigned int encoder_diff = 0;

void encoderInt() {
    encoder_diff++;
}

void setup() {
    attachInterrupt(32, encoderInt, RISING);
    hs.begin(115200, SERIAL_8N1, 4, 2);
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
    if (hs.available())
    {
        String recv_data = hs.readStringUntil('\n');
        if (recv_data[0] == 'M')
        {
            analogWrite(14, recv_data.substring(1).toInt());
        }
        else if (recv_data[0] == 'S')
        {
            int angle = recv_data.substring(1).toInt();
            servo.write(constrain(angle, SERVO_MIN, SERVO_MAX));
        }
    }
}