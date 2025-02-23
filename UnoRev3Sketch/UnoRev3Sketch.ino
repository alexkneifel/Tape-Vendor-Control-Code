/**
* This is the running code for Arduino Rev3.
* This arduino receives messages from Raspberry Pi on UART, receives messages from arduino Rev2 on I2C,
* monitors stepper motor limit switches, and controls the three stepper motors.
*
**/

// can use this to get the last interrupted pin
//#define EI_ARDUINO_INTERRUPTED_PIN

/**
*
* When should I put the stepper motor boards in sleep mode ?
* I need to defined my bytes from raspberry pi to this arduino
* I need to define my bytes from teh toher arduino to this one
* Do i need micro stepping ?
*
**/
#include <EnableInterrupt.h>
#include <Wire.h>

//limit switches
const int Z_TOP_LIMIT = A0;
const int Z_BTM_LIMIT = A1;
const int Y_LEFT_LIMIT = A2;
const int Y_RIGHT_LIMIT = A3;

//stepper motors
const int LEFT_Z_MTR_EN = 13;
const int LEFT_Z_MTR_SLP = 12;
const int LEFT_Z_MTR_STEP = 11;
const int LEFT_Z_MTR_DIR = 10;

const int RIGHT_Z_MTR_EN = 9;
const int RIGHT_Z_MTR_SLP = 8;
const int RIGHT_Z_MTR_STEP = 7;
const int RIGHT_Z_MTR_DIR = 6;

const int Y_MTR_EN = 5;
const int Y_MTR_SLP = 4;
const int Y_MTR_STEP = 3;
const int Y_MTR_DIR = 2;

void killMotors();
void receiveData(int byteCount);

struct Position
{
  float ypos;
  float zpos;
}

// TODO figure out home position
const Position home_pos = {0,0}

void setup() 
{

// setting up serial comm with the raspberry pi
Serial1.begin(9600);

//setting up i2c communication
Wire.begin(0);
Wire.onReceive(receiveData);
// slave will have address 0

// limit switches for the stepper motors
pinMode(Z_TOP_LIMIT, INPUT_PULLUP);
pinMode(Z_BTM_LIMIT, INPUT_PULLUP);
pinMode(Y_LEFT_LIMIT, INPUT_PULLUP);
pinMode(Y_RIGHT_LIMIT, INPUT_PULLUP);

enableInterrupt(Z_TOP_LIMIT, killMotors, FALLING);
enableInterrupt(Z_BTM_LIMIT, killMotors, FALLING);
enableInterrupt(Y_LEFT_LIMIT, killMotors, FALLING);
enableInterrupt(Y_RIGHT_LIMIT, killMotors, FALLING);

  //stepper motor setup
pinMode(LEFT_Z_MTR_EN, OUTPUT);
pinMode(LEFT_Z_MTR_SLP, OUTPUT);
pinMode(LEFT_Z_MTR_DIR, OUTPUT);
pinMode(LEFT_Z_MTR_STEP, OUTPUT);

// low means stepper driver board is enabled
digitalWrite(LEFT_Z_MTR_EN, LOW);
// high means the stepper driver board is not in sleep mode
digitalWrite(LEFT_Z_MTR_SLP, HIGH);

pinMode(RIGHT_Z_MTR_EN, OUTPUT);
pinMode(RIGHT_Z_MTR_SLP, OUTPUT);
pinMode(RIGHT_Z_MTR_DIR, OUTPUT);
pinMode(RIGHT_Z_MTR_STEP, OUTPUT);

digitalWrite(RIGHT_Z_MTR_EN, LOW);
digitalWrite(RIGHT_Z_MTR_SLP, HIGH);

pinMode(Y_MTR_EN, OUTPUT);
pinMode(Y_MTR_SLP, OUTPUT);
pinMode(Y_MTR_DIR, OUTPUT);
pinMode(Y_MTR_STEP, OUTPUT);

digitalWrite(Y_MTR_EN, LOW);
digitalWrite(Y_MTR_SLP, HIGH);


}

void loop() 
{
  // Receving Raspberry Pi Serial
  if (Serial1.available()) 
  {
    // i should come up with a hex encoding system for my data
    byte incomingByte = Serial1.read();  // Read one byte from Serial1

    // Optional: Print the received byte for debugging
    Serial.print("Received byte: ");
    Serial.println(incomingByte, HEX);

    // if command is inputting a casette
    // home the steppers

    // if command is inputting a new cassette
    // home the steppers
    // bring the casette to the position, which is sent by raspberry pi
  }

}
// functions to make
// function which takes a positoin, the stepper motor and it moves the stepper there
// then based on the current position and the positio it has to go to
// will output the required number of steps, and the direction
// home the steppers, should be a set position with this code that is called
// every high to low is one step, the higher the frequency, the faster the 
// motor spins

// i probably need a zeroing function, in which i drive a stepper motor into 
// the bottom limit, i then have all the motors killed, i set the z and y pos
// to zero

/**
* This arduino receives data from the arduino rev3.
*
**/
void receiveData(int byteCount) 
{
  // data here will tell us if a casette has entered
  // if the casette has left as well
  while (Wire.available()) 
  {
    byte receivedByte = Wire.read();  // Read the incoming byte
    Serial.print("Received: ");
    // Print the received byte in hexadecimal
    Serial.println(receivedByte, HEX);  
  }
  
}
void killMotors(){
// kill all the stepper motors, this means set D13, D9, D5 to high
// this disables the stepper motor driver, as an emergency stop
digitalWrite(LEFT_Z_MTR_EN, HIGH);
digitalWrite(RIGHT_Z_MTR_EN, HIGH);
digitalWrite(Y_MTR_EN, HIGH);
// means i should have step down
// maybe send a message to raspberry pi that there was a collision
}
