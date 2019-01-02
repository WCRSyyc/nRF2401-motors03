/*
* Use joystick data received by an from nRF24L01 radio to set the direction and
* speed of individual DC motors, to provide 2-wheel differential steering of a
* carbot chassis.
* https://github.com/WCRSyyc/carbot

* Initially based on the GettingStarted RF24 library and the
* DCMotorTest Adafruit Motor Shield v2 library example sketches.
*/

#include <SPI.h>
#include "RF24.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// motor controller port numbers where motors are physically attached
#define LEFT_MOTOR_PORT 1
#define RIGHT_MOTOR_PORT 3

// Set a range that means 'stopped', to prevent slow speed creeping.
#define ZERO_DEAD_BAND 10
#define MAX_STEP_DELTA 1
// minimum time (milliseconds) to wait beteen motor speed setting changes:
// smaller gives higher acceleration
#define CHANGE_WAIT 10
// MAX_STEP_DELTA and CHANGE_WAIT combine to give maximum acceleration.  Higher
// delta and/or smaller wait give higher (maximum) acceleration

#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED -255
#define TURN_FACTOR 4
// names for motor indexes
#define LEFT 0
#define RIGHT 1
const unsigned int MOTOR_COUNT = 2;
const byte DATA_PIPE = "1Node";  // pipe name (ID)
const int READ_PIPE = 1;  // pipe number
const int STOP_SPEED = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotorL = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *myMotorR = AFMS.getMotor(RIGHT_MOTOR_PORT);

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8
RF24 radio(7, 8);

struct txMessStru {
  int buffType;
  int x, y, z;
};
// x and y can be in the range -255 to 255
// z is zero or one

txMessStru joyStickData;
int curSpeed[MOTOR_COUNT];
int targSpeed[MOTOR_COUNT];
unsigned long nextChangeTime;

void setup() {
  Serial.begin( 115200 );
  Serial.print(F("\nWCRS carbot using Adafruit Motorshield"));

  // Make sure data is cleared.  Mostly done by the compiler, but be extra safe
  for(int motor = 0; motor < MOTOR_COUNT; motor++) {
    curSpeed[motor] = 0;
    targSpeed[motor] = 0;
  }

  AFMS.begin();  // create with the default frequency 1.6KHz

  myMotorL->setSpeed(curSpeed[LEFT]);
  myMotorR->setSpeed(curSpeed[RIGHT]);
  myMotorL->run(RELEASE);
  myMotorR->run(RELEASE);

  radio.begin();

  // Testing has found that even at the lowest power setting, the communications
  // is stable at the greatest range attempted with the remote control.
  radio.setPALevel(RF24_PA_LOW);

  // Currently one way communications, so only need a reading pipe
  radio.openReadingPipe(READ_PIPE, DATA_PIPE);

  // Start the radio listening for data
  radio.startListening();
  nextChangeTime = millis();
}

void loop() {
  unsigned long nowTime;
  getPacket(); // update control settings, whenever new ones are available
  nowTime = millis();
  if(nowTime < nextChangeTime) {  // Not time to change the settings yet
    return;
  }

  // DC motors have maximum torque at zero RPM.  Better might be to dynamically
  // adjust the CHANGE_WAIT based on the current speed (power setting)
  nextChangeTime = nowTime + CHANGE_WAIT;

  // The (non-turning) target speed for both motors is the latest y joystick value
  targSpeed[LEFT] = joyStickData.y;
  targSpeed[RIGHT] = joyStickData.y;

  adjustForTurn();  // Adjust target speeds if turning

  curSpeed[LEFT]  = calcMotorSpeed(curSpeed[LEFT],  targSpeed[LEFT]);
  curSpeed[RIGHT] = calcMotorSpeed(curSpeed[RIGHT], targSpeed[RIGHT]);

  setMotorSpeed(myMotorL, curSpeed[LEFT]);
  setMotorSpeed(myMotorR, curSpeed[RIGHT]);
}// ./void loop()

/**
 * Whenever new joystick values are available, update the latest settings
 */
void getPacket()
{
  if ( radio.available()) { // if another packet has arrived (is arriving)
    radio.read(&joyStickData, sizeof(txMessStru));

    // Adjust (too) low (speed but not steering) readings to actual zero
    if (abs(joyStickData.y) < ZERO_DEAD_BAND) {
      joyStickData.y = 0;
    }
  }
}// ./void getPacket()


/**
 * Adjust the individual target motor speeds to provide delta for turning
 */
void adjustForTurn()
{
  // Negative values of x turn (more) left (slower left, faster right)
  targSpeed[LEFT]  += (joyStickData.x / TURN_FACTOR );
  targSpeed[RIGHT] -= (joyStickData.x / TURN_FACTOR );
}// ./void adjustForTurn()


/**
 * Calculated a new motor speed setting closer to the target, with the change
 * limited by the maximum allowed acceleration (MAX_STEP_DELTA)
 *
 * @param cur the current motor speed setting
 * @param targ the target motor speed setting
 */
int calcMotorSpeed(int cur, int targ)
{
  int newSpeed;

  if(targ < cur) { // more negative speed
    if(cur - MAX_STEP_DELTA < targ) { // delta less than max
      newSpeed = targ;
    } else {
      newSpeed = cur - MAX_STEP_DELTA; // limit change to max step
    }
  } else { // targ >= cur
    if(cur + MAX_STEP_DELTA > targ) { // delta less than max
      newSpeed = targ;
    } else {
      newSpeed = cur + MAX_STEP_DELTA; // limit change to max step
    }
  }

  if(newSpeed < MIN_MOTOR_SPEED) {
    newSpeed = MIN_MOTOR_SPEED;
  }
  if(newSpeed > MAX_MOTOR_SPEED) {
    newSpeed = MAX_MOTOR_SPEED;
  }

  return newSpeed;
}// ./int calcMotorSpeed()


/**
 * Set the new speed for a motor
 *
 * @param aMotor motor to change speed setting on
 * @param speedSetting new motor speed (-255 to 255)
 */
void setMotorSpeed(Adafruit_DCMotor * aMotor, int speedSetting)
{
  if(abs(speedSetting) < ZERO_DEAD_BAND) {
    // In the speed dead band: stop, or stay stopped
    aMotor->setSpeed(STOP_SPEED);
    aMotor->run(RELEASE);
    return;
  }

  if(speedSetting < 0) { // turn wheel backwards
    aMotor->run(BACKWARD);
    aMotor->setSpeed(-speedSetting);
  } else { // turn wheel forward
    aMotor->run(FORWARD);
    aMotor->setSpeed(speedSetting);
  }
}// ./void setMotorSpeed()
