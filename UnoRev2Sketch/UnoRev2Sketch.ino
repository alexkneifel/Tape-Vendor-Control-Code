/**
*
* This file contains control code for Arduino Rev 2.
* It is the slave in I2C to Uno Rev3, it controls all the DC motors (via 
* a DC motor shield) abnd the servo motor via low switching.
* It also monitors if a casette has been entered into the machine via two infrared sensors.
*
**/

#include <Wire.h>
#include <AFMotor.h>
#include <Servo.h>
#include <stdint.h>

// I2C slave bytes
const uint8_t SERVO_DONE  = 1 << 0;  // 0b00000001
const uint8_t SERVO_MOVING = 0;

// I2C master bytes
const uint8_t MOVE_SERVO = 1 << 0; 


// Create motor shield object
AF_DCMotor motor1(1);
AF_DCMotor motor4(4);

// Servo object
Servo servo2;
const int SERVO_ENABLE = A0; // Enable pin
const int SERVO_SIG = 9;

const int I2C_INDICATOR = A1;

// Infrared Sensors
const int IR_SENSOR_FRONT = A2;
const int IR_SENSOR_BACK = A3;

// I2C Address of Slave Arduino
const int SLAVE_ADDR = 0x08;
// bit that flips everytime master commands this arduino to track extension direction
bool servo_extended = false;
uint8_t servo_status = SERVO_DONE; // 0 means the servo is moving
unsigned long servo_start_time = 0;
// how long exactly does it take for the servo to move ?
const unsigned long SERVO_MOVE_TIME = 1000; // ms
volatile bool move_servo_requested = false;
volatile bool servo_ack_pending = false;

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

// it should only need to send back
// servos are done moving (whether it's pickup or dropoff, it doesnt matter, master arduino will keep track off)

// it needs to receive
// at position for servo extension (whether it's pickup or dropoff it's the same thing, master arduino will keep track of this)
void receiveData(int bytes);
void sendData ();
void startRampUpMotors(); 
void rampDownMotors();
void startRampUpOuttakeMotors();
void rampDownOuttakeMotors();
void stopMotors();
void moveServo();

void setup() 
{
    Serial.begin(9600);

    Wire.begin(SLAVE_ADDR);
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    motor1.setSpeed(0);
    motor4.setSpeed(0);
    motor1.run(RELEASE);
    motor4.run(RELEASE);

    // Attach servo
    servo2.attach(SERVO_SIG);
    pinMode(SERVO_ENABLE, OUTPUT);
    digitalWrite(SERVO_ENABLE, LOW); // Disable servo initially

    pinMode(I2C_INDICATOR, OUTPUT);
    digitalWrite(I2C_INDICATOR, LOW);

    // Sensor setup
    pinMode(IR_SENSOR_FRONT, INPUT_PULLUP);
    pinMode(IR_SENSOR_BACK, INPUT_PULLUP);

}

void loop() 
{
  frontside = digitalRead(IR_SENSOR_FRONT);
  backside = digitalRead(IR_SENSOR_BACK);

  //Serial.println(frontside);
  //Serial.println(backside);

    // motor1.setSpeed(255);
    // motor4.setSpeed(255);
    // motor1.run(BACKWARD);
    // motor4.run(FORWARD);

 // since .write() is non-blocking, need to flip the bit after a set time that the servo is moving
 // because servos dont output their actual position
   if(move_servo_requested)
  {
    Serial.println(" Move Servo ");
    moveServo();
    move_servo_requested = false;
  }

// servo_status == 0 means it is in motion
// this is checking that 1 second has elapsed since it first moved
  if ( servo_ack_pending /*&& servo_status == SERVO_MOVING*/ && millis() - servo_start_time >= SERVO_MOVE_TIME) 
  {
    // sometimes this responds done way quicker than it actually is
    //servo_status = SERVO_DONE;
    servo_ack_pending = false;
    digitalWrite(SERVO_ENABLE, LOW);
    digitalWrite(I2C_INDICATOR, LOW);
  }

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

void receiveData(int bytes) 
{
  uint8_t command = Wire.read();  // read one character from the I2C

  if(command == MOVE_SERVO)
  {
    move_servo_requested = true;
    servo_ack_pending = true;
  }
}

void moveServo()
{
  // 0 means it is in motion
  //servo_status = SERVO_MOVING;

  digitalWrite(I2C_INDICATOR, HIGH);

  if(servo_extended)
  {
    // retract servo
    digitalWrite(SERVO_ENABLE, HIGH);
    servo2.write(0);
    servo_extended = false;
  }
  else
  {
    // extend servo
    digitalWrite(SERVO_ENABLE, HIGH);
    // was 155
    servo2.write(158);
    servo_extended = true;
  }
  servo_start_time = millis();
}

void sendData()
{
  //Wire.write(servo_status);
  // 0 should be that it's moving and 1 not
  // and servo_ack_pending is the not of this so
  Wire.write(!servo_ack_pending);
}
