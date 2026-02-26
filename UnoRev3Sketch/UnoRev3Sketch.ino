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

//User interface command library
const uint8_t HOME = 0x01;
const uint8_t PICKUP = 0x02;
const uint8_t DROPOFF = 0x03;
const uint8_t GOTO = 0x04;
const uint8_t SERVO = 0x05;
const uint8_t OFFSET = 0x06;
const uint8_t CANCEL = 0x07;
const uint8_t REMOVE = 0x08;
const uint8_t ENTRANCE = 0x09;
const uint8_t DISPENSE = 0x0A;
const uint8_t RETURN = 0x0B;
const uint8_t SWITCH = 0x0C;

//Internal Comms
const uint8_t CASS_ACTION_DONE = 0x0D;
const uint8_t ARDUINO_DONE = 0x4B;

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
const int RIGHT_SHELF_IDX = 5;

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

// serial that comes in is always
// serial command: HOME, SERVO, CANCEL, ENTRANCE,
// serial command followed by a number (need to divide by 10): offset
// serial commands followed by positions: PICKUP, DROPOFF, GOTO, RETURN, REMOVE, DISPENSE
// serial commands followed by two positions: SWITCH

// arduino to arduino done, CASS_ACTION_DONE
// arduino to rpi done ARDUINO_DONE

    if (Serial.available() >= 1)
    {
        uint8_t cmd = Serial.peek();   // look but don't remove yet

        int requiredBytes = 1;

        // Determine packet size
        if (cmd == OFFSET)
            requiredBytes = 2;
        else if (cmd == SWITCH)
            requiredBytes = 5;
        else if (cmd == PICKUP || cmd == DROPOFF || cmd == GOTO || cmd == RETURN || cmd == REMOVE || cmd == DISPENSE)
            requiredBytes = 3;
        else
            requiredBytes = 1;

        // Wait until full packet is available
        if (Serial.available() >= requiredBytes)
        {
            cmd = Serial.read();   // now actually remove it

            int x = -1;
            int z = -1;
            int x1 = -1;
            int z1 = -1;
            int x2 = -1;
            int z2 = -1;
            int x_offset = -1;

            // ---------------- Read Payload ----------------

            if (cmd == OFFSET)
            {
                x_offset = Serial.read();
            }

            else if (requiredBytes == 3)
            {
                x = Serial.read();
                z = Serial.read();
            }

            else if (cmd == SWITCH)
            {
                x1 = Serial.read();
                z1 = Serial.read();
                x2 = Serial.read();
                z2 = Serial.read();
            }

            // ---------------- Immediate Commands ----------------

            if (cmd == HOME && !btmLimHit)
            {
                homeMotorsZ();
                homeMotorsX();
            }

            else if (cmd == SERVO)
            {
                pingServo();
            }

            // where is the entrance position, is it below or above the entrance
            else if (cmd == ENTRANCE)
            {
                moveTo(entrancePos);
            }

            else if (cmd == CANCEL)
            {
                disableMotorsZ();
                disableMotorsX();
            }

            // ---------------- OFFSET ----------------

            else if (cmd == OFFSET)
            {
                if (x_offset >= 0 && x_offset <= 5)
                {
                    X_OFFSET = x_offset / 10.0;
                }
                return;
            }

            // ---------------- Positional Commands ----------------

            else if (cmd == GOTO || cmd == PICKUP || cmd == DROPOFF || cmd == RETURN || cmd == DISPENSE || cmd == REMOVE)
            {
                Position shelf_pos = posTranslation(x, z);
                // above shelf doesnt work because x and z are indices not a distance
                Position above_shelf = { shelf_pos.xPos, shelf_pos.zPos + PICKUP_DIST };
                // Validate indices BEFORE computing positions
                if (x < 0 || x > RIGHT_SHELF_IDX || z < 0 || z > TOP_SHELF_IDX)
                {
                  return;
                }
                else if (cmd == GOTO)
                {
                  if (x == 0 && z == 0)
                  {
                    moveTo(entrancePos);
                  }
                  else
                  {
                    moveTo(shelf_pos);
                  }
                }
                else if (cmd == PICKUP)
                {
                  moveTo(shelf_pos);
                  pingServo();
                  waitForServo();
                  moveTo(above_shelf);
                  pingServo();
                  waitForServo();
                }
                else if (cmd == DROPOFF)
                {
                  moveTo(above_shelf);
                  pingServo();
                  waitForServo();
                  moveTo(shelf_pos);
                  pingServo();
                  waitForServo();
                }
                else if (cmd == RETURN)
                {
                  // ping other arduino to be ready for a cassette
                  // move to entrance to get cassette
                  // ping RPi that ARDUINO_DONE
                  moveTo(above_shelf);
                  pingServo();
                  waitForServo();
                  moveTo(shelf_pos);
                  pingServo();
                  waitForServo();
                  // ping RPI that arduino DONE 
                }
                else if (cmd == DISPENSE || cmd == REMOVE)
                {
                  // ping other arduino to be ready for a cassette
                  moveTo(shelf_pos);
                  pingServo();
                  waitForServo();
                  moveTo(above_shelf);
                  pingServo();
                  waitForServo();
                  // do the move to drop off cassette at entrance
                  // listen to other servo for IR being low
                  // ping RPi that ARDUINO_DONE
                }

            }

            // ---------------- SWITCH ----------------

            else if (cmd == SWITCH)
            {
                if (x1 < 0 || z1 < 0 || x2 < 0 || z2 < 0)
                    return;

                // Validate ranges if needed

                // perform switching logic here
                return;
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

  if(posToGoTo.xPos == entrancePos.xPos && posToGoTo.zPos == entrancePos.zPos)
  {
    while( xStepper.distanceToGo() != 0 && !topLimLatched && !btmLimLatched && !leftLimLatched && !rightLimLatched)
    {
      xStepper.run();
    }
  // ignore the bottom lim being latched
    while (( leftZStepper.distanceToGo() != 0 || rightZStepper.distanceToGo() != 0) && !topLimLatched && !leftLimLatched && !rightLimLatched) 
    {
      leftZStepper.run();
      rightZStepper.run();
    }
  }
  else {
  // check if not at position yet, and also check if any limit has latched at every step
  while (( leftZStepper.distanceToGo() != 0 || rightZStepper.distanceToGo() != 0 || xStepper.distanceToGo() != 0) && !topLimLatched && !btmLimLatched && !leftLimLatched && !rightLimLatched) 
  {
    leftZStepper.run();
    rightZStepper.run();
    xStepper.run();
  }
  }

  disableMotorsZ();
  disableMotorsX();
}

void pingServo()
{
  Wire.beginTransmission(SLAVE_ADDR);
  // ping it to move the servo
  Serial.println(MOVE_SERVO);
  Wire.write(MOVE_SERVO);  // same as MOVE_SERVO
  Wire.endTransmission();
  servo_moving = true;
}

void waitForServo()
{
  while (true)
  {
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
  leftZStepper.moveRelative(-1000);
  rightZStepper.moveRelative(-1000);

  while (!btmLimLatched) 
  {
  leftZStepper.run();
  rightZStepper.run();
  }

  // kills the two motors
  disableMotorsZ();

  // motor doesnt actually move just resets the planner state
  //resets velocity and accel step 
  leftZStepper.stop();
  rightZStepper.stop();
  delay(50);

  leftZStepper.setCurrentPosition(0);
  rightZStepper.setCurrentPosition(0);

  delay(500);

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
  disableMotorsZ();

}

void homeMotorsX()
{

  //enable the motors to home Z
  enableMotorsX();

  // move motors until it hits the limit switch
  xStepper.moveRelative(-1000);

  while (!leftLimLatched) 
  {
  xStepper.run();
  }

  // kills the two motors
  disableMotorsX();

  // motor doesnt actually move just resets the planner state
  //resets velocity and accel step 
  xStepper.stop();
  delay(50);

  xStepper.setCurrentPosition(0);

  delay(500);

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
  shelf.xPos = X_OFFSET + (xIdx-1) * X_SHELF_DIST;
  shelf.zPos = Z_OFFSET + (zIdx) * Z_SHELF_DIST;
  return shelf;
}





