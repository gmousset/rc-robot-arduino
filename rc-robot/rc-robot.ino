#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/*
 * Motors
 */
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

/*
 * Serial inputs
 */
byte command[4];
boolean hasNewCommand = false;

void setup() {
  // Serial
  Serial.begin(115200);
  //inputString.reserve(200);
  // Motors
  AFMS.begin();
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  myMotor3->setSpeed(0);
  myMotor4->setSpeed(0);
  Serial.println("setup ok");
}

void loop() {
  if (hasNewCommand) {
    Serial.println("new input command");
    handleCommand();
  }
}

void handleCommand() {
    byte type = command[0];
    switch (type) {
      case 0x00: 
        Serial.println("REBOOT"); 
        break;
      case 0x01: 
        Serial.println("ENGINE"); 
        handleEngineCommand();
        break;
    }
}

void handleEngineCommand() {
  byte loc = command[1];
  byte dir = command[2];
  byte powr = command[3];
  updateMotors(loc, dir, powr);
}

void updateMotors(byte loc, byte dir, byte powr) {
  Adafruit_DCMotor *m1;
  Adafruit_DCMotor *m2;

  if (loc == 0x00) {  // left
    m1 = myMotor1;
    m2 = myMotor4;
  } else { //if (loc == 0x01) { // right
    m1 = myMotor2;
    m2 = myMotor3;
  }

  int cDir;
  if (dir == 0x00) {
    cDir = FORWARD;
  } else { //if (dir == 0x01) {
    cDir = BACKWARD;
  }

  m1->run(cDir);
  m2->run(cDir);
  m1->setSpeed(powr);
  m2->setSpeed(powr);
}

void serialEvent() {
  Serial.readBytes(command, 4);
  hasNewCommand = true;
}

