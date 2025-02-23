#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

// Create motor shield object
//TODO i dont think this is how i use motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create motor objects
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); // M1
Adafruit_DCMotor *motor4 = AFMS.getMotor(4); // M4

// Servo object
Servo cassetteServo;
const int SERVO_PIN = 9; // Servo 1
const int SERVO_ENABLE = A0; // Enable pin

// Infrared Sensors
const int IR_SENSOR_1 = A2;
const int IR_SENSOR_2 = A3;

// I2C Address of Slave Arduino
// TODO i dont think this is my slave address
const int SLAVE_ADDRESS = 0x08;

// State tracking
bool cassettePresent = false;

void setup() 
{
    Serial.begin(9600);
    Wire.begin();

    // Initialize motor shield
    AFMS.begin();

    // Set motor speed to 255 (max speed)
    motor1->setSpeed(255);
    motor4->setSpeed(255);

    // Attach servo
    cassetteServo.attach(SERVO_PIN);
    pinMode(SERVO_ENABLE, OUTPUT);
    digitalWrite(SERVO_ENABLE, LOW); // Disable servo initially

    // Sensor setup
    pinMode(IR_SENSOR_1, INPUT_PULLUP);
    pinMode(IR_SENSOR_2, INPUT_PULLUP);
}

void loop() 
{
    // Read infrared sensors
    bool sensor1 = digitalRead(IR_SENSOR_1);
    bool sensor2 = digitalRead(IR_SENSOR_2);

    if (sensor1 == LOW && sensor2 == LOW && !cassettePresent) 
    {
        Serial.println("Cassette detected, driving motors...");
        driveMotors();

        Wire.beginTransmission(SLAVE_ADDRESS);
        Wire.write(0x01); // Cassette detected
        Wire.endTransmission();

        cassettePresent = true;
    }

    if (sensor1 == HIGH && sensor2 == HIGH && cassettePresent) 
    {
        Serial.println("Cassette on lift, stopping motors.");
        stopMotors();

        Wire.beginTransmission(SLAVE_ADDRESS);
        Wire.write(0x02); // Cassette on lift
        Wire.endTransmission();

        cassettePresent = false;
    }
}

void driveMotors() 
{
    motor1->run(FORWARD);
    motor4->run(FORWARD);
}

void stopMotors() 
{
    motor1->run(RELEASE);
    motor4->run(RELEASE);
}
