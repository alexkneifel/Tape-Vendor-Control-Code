/**
*
* This file contains control code for Arduino Rev 2.
* It is the master in I2C to Uno Rev3, it controls all the motors (via 
* a DC motor shield) : the DC motors, the servo motor via low switching.
* It also monitors if a casette has been entered into the machine via two infrared sensors.
*
**/

#include <Wire.h>
#include <AFMotor.h>
#include <Servo.h>

// Create motor shield object
AF_DCMotor motor1(1);
AF_DCMotor motor4(4);

// Servo object
Servo cassetteServo;
const int SERVO_PIN = 9; // Servo 1
const int SERVO_ENABLE = A0; // Enable pin

// Infrared Sensors
const int IR_SENSOR_FRONT = A2;
const int IR_SENSOR_BACK = A3;

// I2C Address of Slave Arduino
// TODO i dont think this is my slave address
const int SLAVE_ADDRESS = 0x08;

enum State 
{
  IDLE,
  CASSETTE_DETECTED_FRONT,
  CASSETTE_READY,
  CASSETTE_DETECTED_REAR
};

State currentState = IDLE;

bool frontside = false;
bool backside = false;

void setup() 
{
    Serial.begin(9600);
    Wire.begin();

    motor1.setSpeed(0);
    motor4.setSpeed(0);
    motor1.run(RELEASE);
    motor4.run(RELEASE);

    // Attach servo
    cassetteServo.attach(SERVO_PIN);
    pinMode(SERVO_ENABLE, OUTPUT);
    digitalWrite(SERVO_ENABLE, LOW); // Disable servo initially

    // Sensor setup
    pinMode(IR_SENSOR_FRONT, INPUT_PULLUP);
    pinMode(IR_SENSOR_BACK, INPUT_PULLUP);

}

void loop() 
{
  frontside = digitalRead(IR_SENSOR_FRONT);
  backside = digitalRead(IR_SENSOR_BACK);

  Serial.println(frontside);
  Serial.println(backside);



    // motor1.setSpeed(255);
    // motor4.setSpeed(255);
    // motor1.run(BACKWARD);
    // motor4.run(FORWARD);

  switch (currentState)
  {
    case IDLE:
      if(frontside == LOW && backside == HIGH)
      {
        startRampUpMotors();
        currentState = CASSETTE_DETECTED_FRONT;
      }
      else if(frontside == HIGH && backside == LOW)
      {
        startRampUpOuttakeMotors();
        currentState = CASSETTE_DETECTED_REAR;
      }
      else
      {
        stopMotors();
      }
      break;

    case CASSETTE_DETECTED_FRONT:
      if(backside == LOW)
      {
        rampDownMotors();
        currentState = CASSETTE_READY;
      }
      break;

    case CASSETTE_READY:
      if(backside == HIGH)
      {
        currentState = IDLE;
      }
      break;
    
    case CASSETTE_DETECTED_REAR:
      if(frontside == HIGH && backside == HIGH)
      {
        rampDownOuttakeMotors();
        currentState = IDLE;
      }
      break;
  }



}


// could do a test on my motors to see when each starts spinning
void startRampUpMotors() 
{
    motor1.setSpeed(255);
    motor4.setSpeed(255);
    motor1.run(FORWARD);
    motor4.run(BACKWARD);
    delay(100); // Short boost (adjust if needed)

    // Now smoothly ramp up from 75 to max (255)
    for (int speed = 75; speed <= 255; speed++) 
    {
        motor1.setSpeed(speed);
        motor4.setSpeed(speed);
        delay(10); // Smooth transition
    }
}

void rampDownMotors() 
{
    for (int speed = 255; speed >= 0; speed--) 
    {
        motor1.setSpeed(speed);
        motor4.setSpeed(speed);
        motor1.run(FORWARD); // Intake direction
        motor4.run(BACKWARD);
        delay(10);  // Ramp down delay
    }
    stopMotors();
}

void startRampUpOuttakeMotors() 
{
    motor1.setSpeed(255);
    motor4.setSpeed(255);
    motor1.run(BACKWARD);
    motor4.run(FORWARD);
    delay(100); // Short boost (adjust if needed)
    
    for (int speed = 75; speed <= 255; speed++) 
    {
        motor1.setSpeed(speed);
        motor4.setSpeed(speed);
        delay(10);  // Ramp up delay
    }
}

void rampDownOuttakeMotors() 
{
    for (int speed = 255; speed >= 0; speed--) 
    {
        motor1.setSpeed(speed);
        motor4.setSpeed(speed);
        motor1.run(BACKWARD); // Intake direction
        motor4.run(FORWARD);
        delay(10);  // Ramp down delay
    }
    stopMotors();
}

void stopMotors() 
{
    motor1.run(RELEASE);
    motor4.run(RELEASE);
    motor1.setSpeed(0);
    motor4.setSpeed(0);
}
