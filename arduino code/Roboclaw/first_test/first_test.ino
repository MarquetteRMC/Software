#include <SoftwareSerial.h>
#include "RoboClaw.h"

//Roboclaw address can be configured using IonMotion
#define address 0x80

//roboclaw is currently connected to pins Rx and Tx on UDOO
//From documentation Rx and Tx are pins 0 and 1, respectively
RoboClaw roboclaw(&Serial1,10000);

void setup() {
  //roboclaw currently is setup to run with baudrate 38400
  roboclaw.begin(2400);
  delay(2000);
  roboclaw.ForwardMixed(address, 0);
  Serial.println("starting");
}

void loop() {
  roboclaw.ForwardM1(address,127); //running at full speed
  delay(2000);
  roboclaw.BackwardM1(address,127);
  delay(2000);
  roboclaw.BackwardM1(address,0);
  Serial.println("looping");
}
