#include <Arduino.h>
#include <ESP32Servo.h>
#include "HUSKYLENS.h"
#include "HardwareSerial.h"


#define SERVO_MAX 115
#define SERVO_MIN 80
#define MOTOR_PIN 14

// Connecion with master
HardwareSerial hs(1);
HUSKYLENS huskylens;
HardwareSerial camSerial(2); // RX, TX


#define greenId 2
#define redID 1

// origin is TOP LEFT
int greenX = 0;
int greenY = 0;
bool greenInFrame = false;

int redX = 0;
int redY = 0;
bool redInFrame = false;

#define BATTERY_REPORT_RATE 50
unsigned long last_battery_report = 0;

enum SerialCommands {
  SerialMotor,
  SerialServo,
  SerialEncoder,
  SerialBattery,
  SerialBlocks
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

  cum_error = min(cum_error, 100.0f);

  float output = Kp * error + Ki * cum_error + Kd * rate_error;

  last_error = error;

  return output;
}

void sendSerialBlocks(int color, bool inFrame) {
  const int negativeOne = -1;
  if (color == greenId) { // 0 green, 1 red
    hs.write(SerialBlocks);
    hs.write(0); 
    if (inFrame) {
      Serial.printf("Green is in frame: x -> %i, y -> %i\n", greenX, greenY);
      hs.write((uint8_t *)&greenX, sizeof(int));
      hs.write((uint8_t *)&greenY, sizeof(int));
          } else {
      hs.write((uint8_t *)&negativeOne, sizeof(int));
        hs.write((uint8_t *)&negativeOne, sizeof(int));

    }
  }
   else if (color == redID) {
    hs.write(SerialBlocks);
    hs.write(1);
    if (inFrame) {
      hs.write((uint8_t *)&redX, sizeof(int));
      hs.write((uint8_t *)&redY, sizeof(int));
      Serial.printf("Red is in frame: x -> %i, y -> %i\n", redX, redY);
    } else {
      hs.write((uint8_t *)&negativeOne, sizeof(int));
      hs.write((uint8_t *)&negativeOne, sizeof(int));
    }
  }  
}
TaskHandle_t camera_processing;
void cameraProcessing(void *pvParameters) {
  for (;;) {
    vTaskDelay(10);
    // detect blocks
    int32_t error;
    greenInFrame = false;
    redInFrame = false;
    if (!huskylens.request()) Serial.println("Fail to request objects from HUSKYLENS!");
    else if(!huskylens.isLearned()) {}
    else if(!huskylens.available()) {}
    else
    {
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
    if (greenInFrame) { sendSerialBlocks(greenId, true); }
    else { sendSerialBlocks(greenId, false); }
    if (redInFrame) { sendSerialBlocks(redID, true); }
    else { sendSerialBlocks(redID, false); }

  }
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

  camSerial.begin(9600, SERIAL_8N1, 19, 23);
  huskylens.begin(camSerial);
  // while (!huskylens.begin(camSerial))
  // {
  //     Serial.println("Begin failed!");
  //     Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
  //     Serial.println(F("2.Please recheck the connection."));
  //     delay(100);
  // }

  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION); //Switch the algorithm to object tracking.


  xTaskCreatePinnedToCore(cameraProcessing, "camera", 100000, NULL, 10, &camera_processing, 0);

}



void loop() {

  // if (greenInFrame && redInFrame) {
  //   if (greenY > redY) {
  //     if (greenX < 130) {
  //       servo.write(70); // right turn
  //     } else {
  //       servo.write(90);
  //     }
  //   } else {
  //     if (redX > 180) {
  //       servo.write(110); // left turn
  //     } else {
  //       servo.write(90);
  //     }
  //   }
  // }
  // if (greenInFrame){
  //   if (greenX < 130) {
  //     servo.write(70); // right turn
  //   } else {
  //     servo.write(90);
  //   }
  //   Serial.printf("Green is in frame: x -> %i, y -> %i\n", greenX, greenY);
  // }
  // else if (redInFrame){
  //   if (redX > 180) {
  //     servo.write(110); // left turn
  //   } else {
  //     servo.write(90);
  //   }
  //   Serial.printf("Red is in frame: x -> %i, y -> %i\n", redX, redY);
  // } else {
  //   servo.write(90);
  // }

  // receive data from master
  if (hs.available()) {
    int header = hs.read();
    switch (header) {
      case SerialMotor:
        {
          int speed = 0;
          hs.readBytes((uint8_t *)&speed, sizeof(int));
          target_speed = speed / 100.0f;
          break;
        }
      case SerialServo:
        {
          int angle = 0;
          hs.readBytes((uint8_t *)&angle, sizeof(int));
          servo.write(constrain(angle, SERVO_MIN, SERVO_MAX));  // Shouldn't this be executed periodically along motor_speed?
          break;
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
    analogWrite(14, min(motor_speed, 120));
    // analogWrite(14, 0);
    next_motor_write = next_motor_write + 10;
  }

  if ((millis() - last_battery_report) > BATTERY_REPORT_RATE) {
    uint16_t voltage = analogRead(36);
    hs.write(SerialBattery);
    hs.write((uint8_t *)&voltage, sizeof(uint16_t));
    last_battery_report = millis();
  }
}
