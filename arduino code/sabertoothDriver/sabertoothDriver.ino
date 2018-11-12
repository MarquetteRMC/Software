#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#define STOPPED 0
#define FORWARD 1
#define RIGHT 2
#define BACK 3
#define LEFT 4

#define RWHEEL 1
#define LWHEEL 2

#define MAX_SPEED 127

void input_handler(void);
void update_wheels(void);
void set_motors(int, int);

SoftwareSerial RSerial(NOT_A_PIN, 11); //connect to S1
SabertoothSimplified ST(RSerial);

int robot_speed = 100;
float turning_speed = 0.5;
int r_trim = 0, l_trim = 0;
long int stop_time = 0;
int robot_direction = STOPPED;


void setup() {
  
  RSerial.begin(9600);
  Serial.begin(9600);
  Serial.write("Starting up\r\n");

  robot_direction = STOPPED;
  
}

void loop() {

  // Checks for a character in the serial buffer and if there is one calls the handler
  if(Serial.available()) {
    stop_time = millis();
    input_handler();
  }

  // If no character has been received in 1/10 of a second, then it stops the robot
  if(millis() > stop_time+100) {
    robot_direction = STOPPED;
  }

  // Updates the wheel speeds based on the current robot_direction
  // TODO: maybe make this an interrupt handler so that it is only called when robot_direction is changed
  update_wheels();

}



void input_handler(void) {
  // reads in a character and sets the direction
  // TODO: make it so we can pass in packets with Vr and Vl and have the RPi handle kinematics
  // TODO: I want to make this work with a joystick or something but I still need to figure out how that will work
  
  switch (Serial.read()) {
    case '.':
      robot_speed = constrain(robot_speed+10, 1, MAX_SPEED);
      break;
    case ',':
      robot_speed = constrain(robot_speed-10, 1, MAX_SPEED);
      break;
    case 'w':
      robot_direction = FORWARD;
      break;
    case 'd':
      robot_direction = RIGHT;
      break;
    case 's':
      robot_direction = BACK;
      break;
    case 'a':
      robot_direction = LEFT;
      break;
    case ' ':
      robot_direction = STOPPED;
      update_wheels();
      break;
  }
}

void update_wheels(void) {
  // Based on the direction, tells the wheels what to do
  
  switch (robot_direction) {
    case STOPPED:
      set_motors(0, 0);
      break;
    case FORWARD:
      set_motors(robot_speed, robot_speed);
      break;
    case RIGHT:
      set_motors(robot_speed*turning_speed, -robot_speed*turning_speed);
      break;
    case BACK:
      set_motors(-robot_speed, -robot_speed);
      break;
    case LEFT:
      set_motors(-robot_speed*turning_speed, robot_speed*turning_speed);
      break;
  }


  
}

void set_motors(int l, int r) {
  // Outputs the speed to the wheels with a trim value so taht we can get the robot to drive straight.
  // motor takes which wheel to write to, and then a value from -127 to 127
  ST.motor(LWHEEL,l+constrain(l_trim,-1,1)*l_trim);
  ST.motor(RWHEEL,r+constrain(r_trim,-1,1)*r_trim);
}

