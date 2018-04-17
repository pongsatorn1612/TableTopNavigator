

/******************************************************************************
MostBasicFollower.ino

A very simple method for following a line with a redbot and the line follower array.

Marshall Taylor, SparkFun Engineering

5-27-2015

Library:
https://github.com/sparkfun/SparkFun_Line_Follower_Array_Arduino_Library
Product:
https://github.com/sparkfun/Line_Follower_Array

This example demonstrates the easiest way to interface the redbot sensor bar.
  "SensorBar mySensorBar(SX1509_ADDRESS);" creates the sensor bar object.
  "mySensorBar.init();" gets the bar ready.
  "mySensorBar.getDensity()" gets the number of points sensed.
  "mySensorBar.getPosition()" gets the average center of sensed points.

The loop has three main points of operation.
  1.  check if the density is reasonable
  2.  get the position
  3.  choose a drive mode based on position

Note:
The wheel direction polarity can be switched with RIGHT/LEFT_WHEEL_POL (or by flipping the wires)
  #define RIGHT_WHEEL_POL 1
  #define LEFT_WHEEL_POL 1
To check, hold the bot centered over a line so that the two middle sensors detect
  then observe.  Does the bot try to drive forward?
  
Resources:
sensorbar.h

Development environment specifics:
arduino > v1.6.4
hw v1.0

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/


/*****************************************************************************
 * Modify by : Mark
 * 
 * 4/15/2018
 ****************************************************************************/

 /*
  * CAN:
  *   Detact cliff and turn accordingly
  *   find object
  *   Turn when detech cliffs from left and right
  *   
  * CANNOT:
  *   find object range more than 100 due to ping sensor read only up to 100(don't know why)
  *   
  *   find goal
  */
 

#include "Wire.h"
#include "sensorbar.h"
#include <RedBot.h>  // This line "includes" the RedBot library into your sketch.
// Provides special objects, methods, and functions for the RedBot.
//The Redbot Library can be found here: https://github.com/sparkfun/RedBot

// --- Defining Section ------------------------------------------------------------------------


/////////////////////////////// Sensor array ////////////////////////////
// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);
                                                                        
//////////////////////////////////////////////////////////////////////////

///////////////////////////////// Ping Sensor ///////////////////////////
const int trigPin = 9;   // sending sound out
const int echoPin = 10;  // receive sound in

long duration;
int distance;
int positionCar;
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////// IR Sensor////////////////////////////
boolean haveObject = false;

/////////////////////////////////////////////////////////////////////////

//////////////////////////////// Mortor /////////////////////////////////
//Define motor polarity for left and right side -- 
//USE TO FLIP motor directions if wired backwards
#define RIGHT_WHEEL_POL 1
#define LEFT_WHEEL_POL 1

//Define the states that the decision making machines uses:
#define IDLE_STATE 0
#define FIND_OBJECT 1
#define FIND_GOAL 2
#define GO_FORWARD 3
#define BACK_UP 4
#define GO_LEFT 5
#define GO_RIGHT 6
#define TURN_LEFT 7
#define TURN_RIGHT 8

uint8_t nextState;


int turnWhichWay = TURN_RIGHT ; // make it smarter by keep track of the last cliff
int density;

int forwardSpeed = 100; // test value
int reverseSpeed = 100;
int distanceExpected = 10;


//-----------------------------------------------------------------------
// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.
#define FORWARD 0
#define REVERSE 1

// Motor definitions to make life easier:
#define MOTOR_A 0
#define MOTOR_B 1

// Pin Assignments //
//Default pins:
#define DIRA 2 // Direction control for motor A
#define PWMA 3  // PWM control (speed) for motor A
#define DIRB 4 // Direction control for motor B
#define PWMB 11 // PWM control (speed) for motor B

////Alternate pins:
//#define DIRA 8 // Direction control for motor A
//#define PWMA 9 // PWM control (speed) for motor A
//#define DIRB 7 // Direction control for motor B
//#define PWMB 10 // PWM control (speed) for motor B
//---------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////

// end of defining section ---------------------------------------------------

void setup()
{
  setupPins(); // Set all pins as outputs
  
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  
  //Default: the IR will only be turned on during reads.
  mySensorBar.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default: dark on light
  mySensorBar.clearInvertBits();
  //Other option: light line on dark
  //mySensorBar.setInvertBits();
  
  //Don't forget to call .begin() to get the bar ready.  This configures HW.
  uint8_t returnStatus = mySensorBar.begin();
  if(returnStatus)
  {
	  Serial.println("sx1509 IC communication OK");
  }
  else
  {
	  Serial.println("sx1509 IC communication FAILED!");
  }
  Serial.println();
  
}

void loop() //--------------------------------------------------------------------------
{


  uint8_t state = nextState;
  
  switch (state) {
  case IDLE_STATE: 
    stopArdumoto(MOTOR_A);
    stopArdumoto(MOTOR_B);
    nextState = FIND_OBJECT;
    break;
    
  case FIND_OBJECT:

    if(arrayFoundCliff()) {   // looking for cliff
      nextState = BACK_UP; // found it, back away
      break;
    }

    if(pingFoundObject(distanceExpected)) { // search for object
      nextState = GO_FORWARD;
      Serial.println("FORWARD");
    } else {                  // not found
      nextState = turnWhichWay; // turn accordingly
      Serial.println("Turn");
    }
    break;

  case FIND_GOAL:

    if(arrayFoundCliff()) {   // looking for cliff
      nextState = BACK_UP; // found it, back away
      break;
    }

    
    
  case GO_FORWARD:
    driveBot(forwardSpeed); // test: go slow
    nextState = FIND_OBJECT;
    break;

  case BACK_UP:
    nextState = backUpWhichWay(); // Back up and Determind turnWhichWay and give nextState
    break;
    
  case GO_LEFT:
    turnBot(1);
    nextState = FIND_OBJECT;
    break;
    
  case TURN_LEFT:
    turnBot(2);
    nextState = FIND_OBJECT;
    break;
    
  case GO_RIGHT:
    turnBot(-1);
    nextState = FIND_OBJECT;
    break;
    
  case TURN_RIGHT:
    turnBot(-2);
    nextState = FIND_OBJECT;
    break;
    
  default:
    stopArdumoto(MOTOR_A);
    stopArdumoto(MOTOR_B);       // Stops both motors
    break;
  }
 
  //delay(1);

  
} // end loop() --------------------------------------------------------------

// Check if in front is a cliff----------------------------------------------------------------------
boolean arrayFoundCliff() {

  if( mySensorBar.getDensity() > 0) { // detech a cilff 

    return true;
    
  } else {
    return false;
  }
  
}// arrayFoundCliff end ----------------------------------------------------------------------------------

boolean pingFoundObject(int distanceWanted) {

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance= duration*0.034/2;

  Serial.print("Distance : ");
  Serial.println(distance);

  if(distance <= distanceWanted) {
    return true;
  } else {
    return false;
  }
}

int backUpWhichWay() {

      positionCar = mySensorBar.getPosition();
      density = mySensorBar.getDensity();

      Serial.print("Car position is ");
      Serial.println(positionCar);
      Serial.print("Sensor array Density ");
      Serial.println(density);
      
      if(density > 5) { // to stranght to cliff case
        driveArdumoto(MOTOR_A, REVERSE, reverseSpeed);
        driveArdumoto(MOTOR_B, REVERSE, reverseSpeed);
        return FIND_OBJECT;
        
      }
      
      if(positionCar >50)  // turn to cliff case
      {
        turnWhichWay = TURN_LEFT;
        return TURN_LEFT;
        
      }
      
      if( positionCar <-50)
      {
        turnWhichWay = TURN_RIGHT;
        return TURN_RIGHT;
        
      } else {
        driveArdumoto(MOTOR_A, REVERSE, reverseSpeed);
        driveArdumoto(MOTOR_B, REVERSE, reverseSpeed);
        return FIND_OBJECT;  
      }
      
}
// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed -----------------------------------
void driveArdumoto(byte motor, byte dir, byte spd)
{
  if (motor == MOTOR_A)
  {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  }
  else if (motor == MOTOR_B)
  {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }  
}

// stopArdumoto makes a motor stop
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

// setupArdumoto initialize all pins
void setupPins()
{
  // setup ping sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}
///-----------------------------------------------------------------------------

//When using driveBot( int16_t driveInput ), pass positive number for forward and negative number for backwards
//driveInput can be -255 to 255
void driveBot( byte spd )
{
	driveArdumoto(MOTOR_A, FORWARD, spd);
  driveArdumoto(MOTOR_B, FORWARD, spd);
}

////When using ( int16_t driveInput, float turnInput ), pass + for forward, + for right
////driveInput can be -255 to 255
////turnInput can be -1 to 1, where '1' means turning right, right wheel stopped
//void driveTurnBot( int16_t driveInput, float turnInput )
//{
//	int16_t rightVar;
//	int16_t leftVar;
//	//if driveInput is negative, flip turnInput
//	if( driveInput < 0 )
//	{
//		turnInput *= .5;
//	}
//	
//	//If turn is positive
//	if( turnInput > 0 )
//	{
//		rightVar = driveInput * RIGHT_WHEEL_POL * ( turnInput );
//		leftVar = 1 * driveInput * LEFT_WHEEL_POL;
//	}
//	else
//	{
//		rightVar = driveInput * RIGHT_WHEEL_POL;
//		leftVar = 1 * driveInput * LEFT_WHEEL_POL *  ( turnInput );
//	}
//
//	  driveArdumoto(MOTOR_A, FORWARD, rightVar);
//    driveArdumoto(MOTOR_B, FORWARD, leftVar);
//		delay(1);
//		
//}

//When using turnBot( float turnInput ), pass + for spin right.
//turnInput can be -1 to 1, where '1' means spinning right at max speed
void turnBot( float turnInput )
{
	int16_t rightVar;
	int16_t leftVar;
	// test value: go slow
	if( turnInput == 1) // GO_LEFT
	{
    rightVar = 200;
    leftVar = 100;
    driveArdumoto(MOTOR_A, FORWARD, rightVar);
    driveArdumoto(MOTOR_B, FORWARD, leftVar);
	}
  if(turnInput == 2) // TRUN_LEFT
  {
    rightVar =200;
    leftVar = 200; // -200
    driveArdumoto(MOTOR_A, FORWARD, rightVar);
    driveArdumoto(MOTOR_B, REVERSE, leftVar);
  }
  if(turnInput == -1) // GO_RIGHT
	{
    rightVar = 100;
    leftVar = 200;
    driveArdumoto(MOTOR_A, FORWARD, rightVar);
    driveArdumoto(MOTOR_B, FORWARD, leftVar);
	}
  if(turnInput == -2) // TRUN_RIGHT
  {
    rightVar = 200; // -200
    leftVar = 200;
    driveArdumoto(MOTOR_A, REVERSE, rightVar);
    driveArdumoto(MOTOR_B, FORWARD, leftVar);
  }

 
  
	delay(5);
	
}

