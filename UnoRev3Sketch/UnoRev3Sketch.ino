/**
* This is the running code for Arduino Rev3.
* This arduino receives messages from RaspberrX Pi on UART, receives messages from arduino Rev2 on I2C,
* monitors stepper motor limit switches, and controls the three stepper motors.
*
**/

// can use this to get the last interrupted pin
//#define EI_ARDUINO_INTERRUPTED_PIN

/**
*
* When should I put the stepper motor boards in sleep mode ?
* I need to defined mX bXtes from raspberrX pi to this arduino
* I need to define mX bXtes from teh toher arduino to this one
* Do i need micro stepping ?
*
**/
#include <EnableInterrupt.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <AccelStepperWithDistance.h>

// I2C slave bytes
const uint8_t SERVO_DONE  = 1 << 0;  // 0b00000001

// I2C master bytes
const uint8_t MOVE_SERVO = 1 << 0; 
const int SLAVE_ADDR = 0x08;
bool servo_moving = false;

//limit switches
//green/brown
const int Z_TOP_LIMIT = A0;
//red/blue
const int Z_BTM_LIMIT = A1;
const int X_RIGHT_LIMIT = A2;
const int X_LEFT_LIMIT = A3;

//stepper motors
const int LEFT_Z_MTR_EN = 13;
//SLP will be good for when nothing is actively happening
const int LEFT_Z_MTR_SLP = 12;
const int LEFT_Z_MTR_STEP = 11;
const int LEFT_Z_MTR_DIR = 10;

const int RIGHT_Z_MTR_EN = 9;
const int RIGHT_Z_MTR_SLP = 8;
const int RIGHT_Z_MTR_STEP = 7;
const int RIGHT_Z_MTR_DIR = 6;

const int X_MTR_EN = 5;
const int X_MTR_SLP = 4;
const int X_MTR_STEP = 3;
const int X_MTR_DIR = 2;

// spacing between the shelves
// should be able to do 0.02 mm steps
const double Z_SHELF_DIST = 25.65;
// should be able to do 0.1 mm steps
const double X_SHELF_DIST = 104.5;
// pos of entrance from limit switch
// Z distance is unique, doesn't need to be perfect just above and below as necessary
// 1 is the minimum but it is rubbing and shifting the cassette
const double ENTR_Z_DIST = 0;
// same as middle shelf with offset
//213 is too far right
const double ENTR_X_DIST = 211;
// distance from limit switch to the first shelf
// this could maybe be less
double X_OFFSET = 1.5;
const double Z_OFFSET = -3;
// number of shelves
const int TOP_SHELF_IDX = 11;
const int RIGHT_SHELF_IDX = 4;

// amount to pick up and drop off a cassette
const double PICKUP_DIST = 10;

// from the above, homing sequence seems to put as at the right spot to pick up the casette
// spacing for the shelves seems to be good though, no Z offset necessary

volatile bool topLimLatched = false;
volatile bool btmLimLatched = false;
volatile bool leftLimLatched = false;
volatile bool rightLimLatched = false;

bool topLimHit = false;
bool btmLimHit = false;
bool leftLimHit = false;
bool rightLimHit = false;

AccelStepperWithDistance leftZStepper(AccelStepperWithDistance::DRIVER, LEFT_Z_MTR_STEP, LEFT_Z_MTR_DIR);
AccelStepperWithDistance rightZStepper(AccelStepperWithDistance::DRIVER, RIGHT_Z_MTR_STEP, RIGHT_Z_MTR_DIR);
AccelStepperWithDistance xStepper(AccelStepperWithDistance::DRIVER, X_MTR_STEP, X_MTR_DIR);

struct Position
{
  double xPos;
  double zPos;
};

const Position entrancePos = {ENTR_X_DIST, ENTR_Z_DIST} ;

void killMotors();
Position posTranslation(int xIdx, int zIdx);
void receiveData(int byteCount);
void enableMotorsZ();
void disableMotorsZ();
void enableMotorsX();
void disableMotorsX();
void moveTo(Position posToGoTo);
void pingServo();
void waitForServo();


// // TODO figure out home position
// const Position home_pos = {0,0}

void setup() 
{

Serial.begin(9600);
Serial.println("Serial started");
// setting up serial comm with the raspberrX pi
//Serial1.begin(9600);

//setting up i2c communication with other arduino, this one is the master
Wire.begin();

// limit switches for the stepper motors
pinMode(Z_TOP_LIMIT, INPUT_PULLUP);
pinMode(Z_BTM_LIMIT, INPUT_PULLUP);
pinMode(X_LEFT_LIMIT, INPUT_PULLUP);
pinMode(X_RIGHT_LIMIT, INPUT_PULLUP);

enableInterrupt(Z_TOP_LIMIT, killMotors, FALLING);
enableInterrupt(Z_BTM_LIMIT, killMotors, FALLING);
enableInterrupt(X_LEFT_LIMIT, killMotors, FALLING);
enableInterrupt(X_RIGHT_LIMIT, killMotors, FALLING);

leftZStepper.setMaxSpeed(1500);
// was 500
leftZStepper.setAcceleration(1000);
leftZStepper.setStepsPerRotation(400);  // For a 1.8° stepper motor, it's 200 steps
leftZStepper.setDistancePerRotation(8); // If one rotation moves 8mm

// was 1000, 500
rightZStepper.setMaxSpeed(1500);
rightZStepper.setAcceleration(1000);
rightZStepper.setStepsPerRotation(400);  // For a 1.8° stepper motor, it's 200 steps
rightZStepper.setDistancePerRotation(8); // one rotation moves 8mm

xStepper.setMaxSpeed(1500);
xStepper.setAcceleration(1000);
xStepper.setStepsPerRotation(400);  // For a 1.8° stepper motor, it's 200 steps
// distance per rotation will be different for a timing belt
// pitch of 2mm/tooth with 20 teeth on the circumference
xStepper.setDistancePerRotation(40); 

//invert direction
// I dont think i need this
// leftZStepper.setPinsInverted(false, true);  
// rightZStepper.setPinsInverted(false, true);

  //stepper motor setup
pinMode(LEFT_Z_MTR_EN, OUTPUT);
pinMode(LEFT_Z_MTR_SLP, OUTPUT);
pinMode(LEFT_Z_MTR_DIR, OUTPUT);
pinMode(LEFT_Z_MTR_STEP, OUTPUT);

// low means stepper driver board is enabled
digitalWrite(LEFT_Z_MTR_EN, HIGH);
// high means the stepper driver board is not in sleep mode
digitalWrite(LEFT_Z_MTR_SLP, HIGH);

//Right Z stepper motor setup
pinMode(RIGHT_Z_MTR_EN, OUTPUT);
pinMode(RIGHT_Z_MTR_SLP, OUTPUT);
pinMode(RIGHT_Z_MTR_DIR, OUTPUT);
pinMode(RIGHT_Z_MTR_STEP, OUTPUT);

digitalWrite(RIGHT_Z_MTR_EN, HIGH);
digitalWrite(RIGHT_Z_MTR_SLP, HIGH);

// X motor stepper setup
pinMode(X_MTR_EN, OUTPUT);
pinMode(X_MTR_SLP, OUTPUT);
pinMode(X_MTR_DIR, OUTPUT);
pinMode(X_MTR_STEP, OUTPUT);

digitalWrite(X_MTR_EN, HIGH);
digitalWrite(X_MTR_SLP, HIGH);


}

void loop() 
{

// checking if switch is being held still

  topLimHit = (digitalRead(Z_TOP_LIMIT) == LOW);
  btmLimHit = (digitalRead(Z_BTM_LIMIT) == LOW);
  leftLimHit = (digitalRead(X_LEFT_LIMIT) == LOW);
  rightLimHit = (digitalRead(X_RIGHT_LIMIT) == LOW);

// checking the interrupt condition to kill motors
// TODO :maybe motor getting killed should be in the interrupt so it doesnt have to go thru some conditions before getting here
  if(topLimLatched || btmLimLatched || leftLimLatched || rightLimLatched)
  {
    disableMotorsZ();
    Serial.println("Z Motor's killed");

    disableMotorsX();
    Serial.println("X Motor killed");

    // since motor didnt make it to the position it thought it would, we need to update it with its current position and stop vel/acc commands
    leftZStepper.stop();
    rightZStepper.stop();
    leftZStepper.setCurrentPosition(leftZStepper.currentPosition());
    rightZStepper.setCurrentPosition(rightZStepper.currentPosition());

    xStepper.stop();
    xStepper.setCurrentPosition(xStepper.currentPosition());

    delay(50);

    // can just reset all of them
    topLimLatched = false;
    btmLimLatched = false;
    leftLimLatched = false;
    rightLimLatched = false;
  }

  if(topLimHit)
  {
    // switch is being held
    Serial.println(" top lim is held ");
  }

  else if(btmLimHit)
  {
    Serial.println(" btm lim is held ");
  }

  else if (leftLimHit)
  {
    Serial.println(" left limit is held");
  }

  else if (rightLimHit)
  {
    Serial.println(" right limit is held");
  }

// TODO: equivalent condition is just the hits, not the latches
// got rid of btmLimHit condition for sake of the entrance
  if(!topLimHit && !topLimLatched && !btmLimLatched && !leftLimHit && !leftLimLatched && !rightLimHit && !rightLimLatched)
  {

    if(Serial.available() > 0)
    {
      String incomingMsg = Serial.readStringUntil('\n');
      incomingMsg.trim();
      // TEMPORARY parsing of the serial message in the format xIndex, zIndex
      int firstComma  = incomingMsg.indexOf(',');
      int secondComma = incomingMsg.indexOf(',', firstComma + 1);

      if (incomingMsg == "h" && !btmLimHit)
      {
        Serial.println("Motor enabled");
        homeMotorsZ();
        homeMotorsX();
      }

      else if (incomingMsg == "s")
      {
        Serial.println("Servo pinged");
        pingServo();
      }
      
      else if (incomingMsg == "c")
      {
        Serial.println("Motor enabled");
        moveTo(entrancePos);
      }

      // if there's no commas, it's a 1.4 to change the offset
      else if(firstComma == -1 ) 
      {
        X_OFFSET = incomingMsg.toFloat();
        Serial.print("X_OFFSET change to ");
        Serial.println(X_OFFSET);
        return;
      }

      // this is just if you put in a locaiton 2,3
      else if(secondComma == -1)
    {
      int xIndex = incomingMsg.substring(0, firstComma).toInt();
      int zIndex = incomingMsg.substring(firstComma + 1).toInt();

      // positional indices must be within
      if(xIndex < 0 || xIndex > RIGHT_SHELF_IDX || zIndex <= 0 || zIndex > TOP_SHELF_IDX) 
      {
        Serial.println("Invalid indices");
        return;
      }
      Serial.println(" Move to location");
      Position shelf_pos = posTranslation(xIndex, zIndex);
      moveTo(shelf_pos);
    }
      else 
      {
        Serial.println("pickup from location");
        String pickup = incomingMsg.substring(0,firstComma);
        int xIndex = incomingMsg.substring(firstComma + 1, secondComma).toInt();
        int zIndex = incomingMsg.substring(secondComma + 1).toInt();
        Position shelf_pos = posTranslation(xIndex, zIndex);
        Position above_shelf = {shelf_pos.xPos, shelf_pos.zPos + PICKUP_DIST};
        // p for pickup
        if(pickup == "p")
        {
          moveTo(shelf_pos);
          pingServo();
          waitForServo();
          moveTo(above_shelf);
          pingServo();
          waitForServo();
        }
        // do for drop off
        else if(pickup == "d")
        {
          moveTo(above_shelf);
          pingServo();
          waitForServo();
          moveTo(posTranslation(xIndex, zIndex));
          pingServo();
          waitForServo();
        }

      }
    }

    
  }
}

void moveTo(Position posToGoTo)
{
  enableMotorsZ();
  enableMotorsX();

  leftZStepper.moveToDistance(posToGoTo.zPos);
  rightZStepper.moveToDistance(posToGoTo.zPos);
  xStepper.moveToDistance(posToGoTo.xPos);

  // check if not at position yet, and also check if any limit has latched at every step
  while (( leftZStepper.distanceToGo() != 0 || rightZStepper.distanceToGo() != 0 || xStepper.distanceToGo() != 0) && !topLimLatched && !btmLimLatched && !leftLimLatched && !rightLimLatched) 
  {
    leftZStepper.run();
    rightZStepper.run();
    xStepper.run();
  }

  Serial.println("Motors killed");
  disableMotorsZ();
  disableMotorsX();
}

void pingServo()
{
  Wire.beginTransmission(SLAVE_ADDR);
  // ping it to move the servo
  Serial.println("Servo sent");
  Serial.println(MOVE_SERVO);
  Wire.write(MOVE_SERVO);  // same as MOVE_SERVO
  Wire.endTransmission();
  servo_moving = true;
}

void waitForServo()
{
  while (true)
  {
    Serial.println(" Waiting for servo");
    Wire.requestFrom(SLAVE_ADDR, 1);
    if (Wire.available())
    {
      if (Wire.read() == SERVO_DONE)
      {
        Serial.println("Servo responded done");
        servo_moving = false;
        break;
      }
    }
    delay(100);
  }
}

void homeMotorsZ()
{

  //enable the motors to home Z
  enableMotorsZ();

  // move motors until it hits the limit switch
  Serial.println(" Homing Motors ");
  leftZStepper.moveRelative(-1000);
  rightZStepper.moveRelative(-1000);

  while (!btmLimLatched) 
  {
  leftZStepper.run();
  rightZStepper.run();
  }

  // kills the two motors
  Serial.println("Z motors killed");
  disableMotorsZ();

  // motor doesnt actually move just resets the planner state
  //resets velocity and accel step 
  leftZStepper.stop();
  rightZStepper.stop();
  delay(50);

  Serial.println(" Zero position set");
  leftZStepper.setCurrentPosition(0);
  rightZStepper.setCurrentPosition(0);

  Serial.println(" Backing off from limit switch");
  delay(500);

  Serial.println("Motor enabled");
  enableMotorsZ();

  // sometimes it backs off and sometimes it doesnt
  // TODO: I think this step size could be smaller
  leftZStepper.moveRelative(2);
  rightZStepper.moveRelative(2);

  while (( leftZStepper.distanceToGo() != 0 || rightZStepper.distanceToGo() != 0)) 
  {
    leftZStepper.run();
    rightZStepper.run();
  }
    
  btmLimLatched = false;

  // turn motors back off
  Serial.println("Motor killed");
  disableMotorsZ();

}

void homeMotorsX()
{

  //enable the motors to home Z
  enableMotorsX();

  // move motors until it hits the limit switch
  Serial.println(" Homing X Motor ");
  xStepper.moveRelative(-1000);

  while (!leftLimLatched) 
  {
  xStepper.run();
  }

  // kills the two motors
  Serial.println("x motor killed");
  disableMotorsX();

  // motor doesnt actually move just resets the planner state
  //resets velocity and accel step 
  xStepper.stop();
  delay(50);

  Serial.println(" Zero position set");
  xStepper.setCurrentPosition(0);

  Serial.println(" Backing off from limit switch");
  delay(500);

  Serial.println("X Motor enabled");
  enableMotorsX();

  // sometimes it backs off and sometimes it doesnt
  // TODO: I think this step size could be smaller
  xStepper.moveRelative(4);

  while (xStepper.distanceToGo() != 0) 
  {
    xStepper.run();
  }
    
  // just backed off so can change interrupt flag
  leftLimLatched = false;

  // turn motors back off
  Serial.println("X Motor killed");
  disableMotorsX();
  
}

void enableMotorsZ()
{
  digitalWrite(LEFT_Z_MTR_EN, LOW);
  digitalWrite(RIGHT_Z_MTR_EN, LOW);
}

void disableMotorsZ()
{
  digitalWrite(LEFT_Z_MTR_EN, HIGH);
  digitalWrite(RIGHT_Z_MTR_EN, HIGH);
}

void enableMotorsX()
{
  digitalWrite(X_MTR_EN, LOW);
}

void disableMotorsX()
{
  digitalWrite(X_MTR_EN, HIGH);
}

void killMotors()
{
    if (digitalRead(Z_TOP_LIMIT) == LOW) 
    {
        topLimLatched = true;
    }

    else if (digitalRead(Z_BTM_LIMIT) == LOW) 
    {
        btmLimLatched = true;
    }

    else if (digitalRead(X_LEFT_LIMIT) == LOW)
    {
      leftLimLatched = true;
    }

    else if (digitalRead(X_RIGHT_LIMIT) == LOW)
    {
      rightLimLatched = true;
    }
}

Position posTranslation(int xIdx, int zIdx)
{
  Position shelf;
  shelf.xPos = X_OFFSET + xIdx * X_SHELF_DIST;
  shelf.zPos = Z_OFFSET + zIdx * Z_SHELF_DIST;
  return shelf;
}





