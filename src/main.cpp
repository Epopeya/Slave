#include <Arduino.h>
#include <ESP32Servo.h>
#include "HUSKYLENS.h"
#include "HardwareSerial.h"
#include "pid.h"
#include "timer.h"

// servo
#define MOTOR_PIN 14

// connections
HardwareSerial hs(1);
HUSKYLENS huskylens;
HardwareSerial camSerial(2);  // RX, TX

// serial
enum SerialCommands {
  SerialMotor,
  SerialServo,
  SerialEncoder,
  SerialBattery,
  SerialBlocks
};

// camera
TaskHandle_t camera_processing;

#define greenId 2
#define redID 1

int greenX = 0;
int greenY = 0;
bool greenInFrame = false;
int redX = 0;
int redY = 0;
bool redInFrame = false;

// battery
#define BATTERY_REPORT_RATE 500000
unsigned long last_battery_report = 0;

// servo
#define SERVO_MAX 140
#define SERVO_MIN 80
#define SERVO_MID 110
Servo servo;


// PID parameters
//PID motorPID(40, 25, 10);
PID motorPID(0.3, 0.2, 0);

volatile unsigned int total_encoders = 0;

void encoderInt() {
  total_encoders++;
}

void sendSerialBlocks(int color, bool inFrame) {
  const unsigned char leavesScene[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
  unsigned char buf[11] = {
    0x16,
    SerialBlocks,
    0,
  };

  if (color == greenId) {
    buf[2] = 0;
  } else if (color == redID) {
    buf[2] = 1;
  }
  if (inFrame) {
    if (color == greenId) {
      buf[3] = greenX & 0xFF;
      buf[4] = (greenX >> 8) & 0xFF;
      buf[5] = (greenX >> 16) & 0xFF;
      buf[6] = (greenX >> 24) & 0xFF;

      buf[7] = greenY & 0xFF;
      buf[8] = (greenY >> 8) & 0xFF;
      buf[9] = (greenY >> 16) & 0xFF;
      buf[10] = (greenY >> 24) & 0xFF;
    } else if (color == redID) {
      buf[3] = redX & 0xFF;
      buf[4] = (redX >> 8) & 0xFF;
      buf[5] = (redX >> 16) & 0xFF;
      buf[6] = (redX >> 24) & 0xFF;

      buf[7] = redY & 0xFF;
      buf[8] = (redY >> 8) & 0xFF;
      buf[9] = (redY >> 16) & 0xFF;
      buf[10] = (redY >> 24) & 0xFF;
    }
  } else {
    memcpy(buf + 3, leavesScene, sizeof(leavesScene));
    memcpy(buf + 7, leavesScene, sizeof(leavesScene));
  }
  hs.write(buf, sizeof(buf));
}

void cameraProcessing(void *pvParameters) {
  for (;;) {
    vTaskDelay(10);
    // detect blocks
    int32_t error;
    greenInFrame = false;
    redInFrame = false;
    if (!huskylens.request()) {
    } else if (!huskylens.isLearned()) {
    } else if (!huskylens.available()) {
    } else {
      if (huskylens.countBlocks(greenId)) {
        HUSKYLENSResult result = huskylens.getBlock(greenId, 0);
        greenInFrame = true;
        greenX = result.xCenter;
        greenY = result.yCenter;
      }
      if (huskylens.countBlocks(redID)) {
        HUSKYLENSResult result = huskylens.getBlock(redID, 0);
        redInFrame = true;
        redX = result.xCenter;
        redY = result.yCenter;
      }
      // HUSKYLENSResult result = huskylens.read();

      // Serial.println(String()+
      // F("Block:xCenter=")+result.xCenter+
      // F(",yCenter=")+result.yCenter+
      // F(",width=")+result.width+
      // F(",height=")+result.height+
      // F(",frame=")+result.frameNum+
      // F(",ID=")+result.ID
      // );
    }

    // check if blocks should be out of frame
    if (greenInFrame) {
      sendSerialBlocks(greenId, true);
    } else {
      sendSerialBlocks(greenId, false);
    }
    if (redInFrame) {
      sendSerialBlocks(redID, true);
    } else {
      sendSerialBlocks(redID, false);
    }
  }
}

void setup() {
  attachInterrupt(32, encoderInt, CHANGE);
  hs.begin(921600, SERIAL_8N1, 4, 2);
  Serial.begin(921600);
  servo.attach(13);

  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(14, OUTPUT);

  // Controller direction and enable
  digitalWrite(25, HIGH);
  digitalWrite(26, HIGH);
  digitalWrite(27, LOW);

  camSerial.begin(9600, SERIAL_8N1, 19, 23);
  huskylens.begin(camSerial);

  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);  //Switch the algorithm to object tracking.
  xTaskCreatePinnedToCore(cameraProcessing, "camera", 100000, NULL, 10, &camera_processing, 0);
}


float motor_speed = 0;
#define MOTOR_UPDATE_RATE 20000
Timer motorTimer(MOTOR_UPDATE_RATE);
Timer batteryTimer(BATTERY_REPORT_RATE);

int serial_encoder = 0;
int motor_encoder = 0;

void loop() {
  // for(int i = 80; i < 120; i++) {
  //   servo.write(i);
  // Serial.println(i);
  // delay(500);

  // }
  
  // return;
  // for (int i = SERVO_MIN; i < SERVO_MAX; i++) {
  //   servo.write(i);
  //   if(SERVO_MID == i) {
  //     delay(1000);
  //   }
  //   delay(20);
  // }
  // for (int i = SERVO_MAX; i > SERVO_MIN; i--) {
  //   servo.write(i);
  //   delay(20);
  // }
  // return;
  // receive data from master
  if (hs.available()) {
    int start = hs.read();
    if (start == 0x16) {
      int header = hs.read();
      // Serial.println(header);
      switch (header) {
        case SerialMotor:
          {
            uint8_t buf[4] = { 0 };
            hs.readBytes(buf, sizeof(buf));
            int speed = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
            motorPID.target = speed;
            break;
          }
        case SerialServo:
          {
            uint8_t buf[4] = { 0 };
            hs.readBytes(buf, sizeof(buf));
            float angle = 0;
            memcpy(&angle, buf, sizeof(float));
            angle = angle * (360.0f / (2.0f * PI)) + SERVO_MID;
            angle = constrain(angle, SERVO_MIN, SERVO_MAX);
            // Serial.println(angle);
            servo.write(angle);  // Shouldn't this be executed periodically along motor_speed?
            break;
          }
      }
    }
  }

  // update motor speed
  if (motorTimer.isPrimed()) {
    float speed = (total_encoders - motor_encoder) / (MOTOR_UPDATE_RATE * 1e-6);
    float motor_speed = motorPID.update(speed);
    analogWrite(MOTOR_PIN, motor_speed);
    // Serial.printf("total: %d, motor: %d, speed: %f, motor_speed: %f\n", total_encoders, motor_encoder, speed, motor_speed);
    motor_encoder = total_encoders;
  }

  if ((total_encoders - serial_encoder) > 0) {
    uint8_t buf[] = {
      0x16,
      SerialEncoder,
      total_encoders - serial_encoder
    };
    hs.write(buf, sizeof(buf));
    serial_encoder = total_encoders;
  }

  if (batteryTimer.isPrimed()) {
    uint16_t voltage = analogRead(36);
    //Serial.printf("battery: %d\n", voltage);
    uint8_t buf[] = { 0x16, SerialBattery, voltage & 0xFF, (voltage >> 8) & 0xFF };
    //Serial.printf("battery buf: %d %d\n", buf[2], buf[3]);
    hs.write(buf, sizeof(buf));
  }
}
