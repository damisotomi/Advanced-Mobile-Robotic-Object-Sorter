// Code description
// Our Robot works using PID for its line follow. 
// We created three modes
// Mode 1. the robot moves on the line and stops when it gets to the end of its path if no obstacle is seen within a certain distance. 
// Mode 2: Robot moves on the line, tries to pick up an object it senses but then continues its journey if no object was actually picked up
// Mode 3, robot moves on the line, picks up an actual object and returns it back to the base and sorts based on color (Red to the left and others to the right)

// linefollower 
#include <QTRSensors.h>
#include <PID_v1.h>
double Setpoint;
double Input; 
double Output; 
QTRSensors qtr;
uint16_t position;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

float Kp = 0.9;
float Ki = 0.025;
float Kd = 0.425;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float Pvalue;
float Ivalue;
float Dvalue;

int counter;
int speedAdjustment;
int totalError = 0;
int previousError = 0;
int P, D, I, PIDvalue, error;
int lsp, rsp;
int lfspeed = 90;


// dc motors 
#include <L298N.h>
#define AIN1 2  //Assign the motor pins
#define BIN1 4
#define AIN2 3
#define BIN2 5
#define PWMA 6
#define PWMB 11

const int offsetA = 1;
const int offsetB = 1;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

// Gripper 
#include <Servo.h>
Servo servo; 

// ultrasonic sensor 
long duration, distance; 
#define SENSOR_PIN 12 

// IR sensor 
const int irPin = A0;
int obstacleState = 0;
int confirm=0;

// color sensor 
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define TCS34725_ADDR (0x29) // I2C address
#define SCL_PIN A5
#define SDA_PIN A4

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_ADDR);
int colorValue;


void setup() {
  Serial.begin(9600);

// servo setup
  servo.attach(A2);  // Attach gripper servo to pin A2

  // ultrasonic setup
  pinMode(SENSOR_PIN, OUTPUT); // Set the sensor pin as an output to send the pulse

  // ir sensor setup
  pinMode(irPin, INPUT); // Sets the irPin as an Input for IR sensor

   // configure the sensors for line following
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(-90, 90);
  qtr.setTypeRC(); /// Call this function to set up RC-type sensors.
  qtr.setSensorPins((const uint8_t[]){7,8,9,10}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  delay(2000);

for (int i = 0; i < 5; i++) { // Repeat the turning and calibrating process
    // Turn gradually to the right and calibrate
    motor_drive(100, -95); 
    delay(250); 
    motor1.stop();
    motor2.stop();
    qtr.calibrate();

    // Turn back to center and calibrate
    motor_drive(-100, 95); // Reverse direction to return to center
    delay(250); 
    motor1.stop();
    motor2.stop();
    qtr.calibrate();

    // Turn gradually to the left and calibrate
    motor_drive(-100, 95); 
    delay(250); 
    motor1.stop();
    motor2.stop();
    qtr.calibrate();

    // Turn back to center and calibrate
    motor_drive(100, -95); // Reverse direction to return to center
    delay(250); 
    motor1.stop();
    motor2.stop();
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.println();
  Serial.print("Calibration Complete!");
  Serial.println();
  delay(2000);
  openGripper();
}

void loop() {
  ultraSonicObstacleCheck();

  counter = 0;       //variable that keeps a count of how long the robot has been on a full black line
  confirm=0;         //variable to confirm if an object was actually picked

 int checkDistance=0;  //variable that tracks when the robot gets to the end of its path.

//  this section keeps the robot moving on the line as long as an obstacle isnt within 4cm of the sensor and the checkDistance is less than 400 
  while (distance > 4 && checkDistance < 400 ) {
      robot_control();
      ultraSonicObstacleCheck();
      checkDistance +=1;
      Serial.println(checkDistance);
      if(checkDistance >= 400){
        motor1.stop();
        motor2.stop();
      while(1){
        // infinite loop to stop the car indicating we have gotten to the end of the path and no obstacle was seen
      }
      }
   
    }

  pickItem();
  delay(1000);

  confirmItem(); //confirms if an item was actually picked
  delay(1000);

if(confirm){
turn90DegreesRight();
  delay(1000);

  // Keep turning until a black line is detected
    bool lineDetected = false;
    while (!lineDetected) {
      lineDetected = keepturning(); // This will stop turning once a black line is detected
      delay(100);
    }

    //code block to ensure the robot doesnt stop at the first full black line it sees
    while (counter < 5){
      robot_control();
      if (sensorValues[0]>=900 && sensorValues[1]>=900 && sensorValues[2]>=900 && sensorValues[3]>=900){
        counter += 1;
        delay(50);
      }
      Serial.println(counter);
    }
    dropItem();
}
else {
    openGripper();
    delay(1000); // Delay to ensure the gripper has enough time to open
    
    while (checkDistance < 400) {
      robot_control();
      ultraSonicObstacleCheck();
      checkDistance += 1;
      Serial.println(checkDistance);

      if (checkDistance >= 400) {
        motor1.stop();
        motor2.stop();
        while(1) {
          // Infinite loop to halt further execution
        }
      }
    }
  }
}


// function definations 

void pickItem(){
    motor1.stop();
    motor2.stop();
    delay(1000);
    closeGripper();
    delay(1000);
}

void confirmItem(){
    int detectionCount = 0;
    const int totalReadings = 10; // Take 10 readings to improve accuracy
    const int detectionThreshold = 7; // Require at least 7 detections to confirm

    for (int i = 0; i < totalReadings; i++) {
        IRSensorObstacleCheck(); // Update obstacleState
        if (obstacleState == LOW) { // LOW means an object is detected
            detectionCount++;
        }
        delay(200); // Wait a bit before the next reading
    }

    if (detectionCount >= detectionThreshold) {
        confirm = 1;
        Serial.println("Object confirmed.");
    } else {
        confirm = 0;
        Serial.println("No object detected.");
    }
}

void dropItem(){
  motor1.stop();
  motor2.stop();
  delay(1000);
  colorCheck();
  if(colorValue==0){
    //green
    turn90DegreesRight();
    delay(1000);
    moveForwardShortDistance();
    delay(1000);
    openGripper();
    delay(1000);
    moveBackwardShortDistance();
    delay(1000);
    turn90DegreesRight();
    delay(1000);
    moveForwardShorterDistance();
    delay(1000);
  }
  else{
    //red
    turn90DegreesLeft();
    delay(1000);
    moveForwardShortDistance();
    delay(1000);
    openGripper();
    delay(1000);
    moveBackwardShortDistance();
    delay(1000);
    turn90DegreesLeft();
    delay(1000);
    moveForwardShorterDistance();
    delay(1000);
  }
}


void moveForwardShortDistance() {
  motor_drive(120, 120); // Move forward
  delay(350); // 
  motor1.stop(); // Stop motors
  motor2.stop();
}

void moveForwardShorterDistance() {
  motor_drive(110, 110); // Move forward
  delay(350); 
  motor1.stop(); // Stop motors
  motor2.stop();
}

void moveBackwardShortDistance() {
  motor_drive(-120, -120); // Move Backward
  delay(350); 
  motor1.stop(); // Stop motors
  motor2.stop();
}

bool keepturning() {
  motor_drive(100, 0); 
  delay(200); 
  position = qtr.readLineBlack(sensorValues);
  motor1.stop();
  motor2.stop();
  delay(50); // Short delay to stabilize sensor readings
  
  // Check if any sensor reads above a threshold, indicating a black line
  for(int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 600) { 
      return true; // Line detected
    }
  }
  return false; // Line not detected, continue turning
}

void turn90DegreesRight() {
  motor_drive(125, -125);
  delay(350); 
  motor1.stop();
  motor2.stop();
}

void turn90DegreesLeft() {
  motor_drive(-115, 115);
  delay(350); 
  motor1.stop();
  motor2.stop();
}


// gripper close function
void closeGripper(){
  servo.write(0);
}

// gripper Open function
void openGripper(){
  servo.write(180);
}

void ultraSonicObstacleCheck() {
  digitalWrite(SENSOR_PIN, LOW);
  delayMicroseconds(2);
  
  // Send a 10-microsecond pulse to start the measurement
  digitalWrite(SENSOR_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_PIN, LOW);
  
  // Change the pin to input to receive the echo
  pinMode(SENSOR_PIN, INPUT);
  
  // Measure the time for the echo to return
  duration = pulseIn(SENSOR_PIN, HIGH);
  
  // Calculate the distance
  distance = duration / 58.2; // Convert the time to distance in cm
  
  // Reset the pin to output after reading
  pinMode(SENSOR_PIN, OUTPUT);

}

// IR sensor routine
void IRSensorObstacleCheck(){
  obstacleState = digitalRead(irPin);
  //low means object detected
  Serial.println(obstacleState);
}
// 
void robot_control(){
  // read calibrated sensor values and obtain a measure of the line position
  delay(15);
  position = qtr.readLineBlack(sensorValues);

  Input = map(position, 0, 3000, -90, 90);

  myPID.Compute();
  speedAdjustment = Output;

  lsp = lfspeed + speedAdjustment;
  rsp = lfspeed - speedAdjustment;

  if (lsp > 90) {
    lsp = 90;
  }
 if (lsp < -90) {
    lsp = -90;
  }
  if (rsp > 90) {
    rsp = 90;
  }
  if (rsp < -90) {
    rsp = -90;
  }
  motor_drive(lsp,rsp);
  Serial.print(lsp);
  Serial.print("    ");
  Serial.print(rsp);
  Serial.println();

}


void motor_drive(int left, int right){
  motor1.setSpeed(abs(left)); // Set speed for left motor
  motor2.setSpeed(abs(right)); // Set speed for right motor
  
  if(right>0)
  {
    motor2.forward();
  }
  else 
  {
    motor2.backward();
  }
  
 
  if(left>0)
  {
    motor1.forward();
  }
  else 
  {
    motor1.backward();}
}

void colorCheck(){
   uint16_t clear, red, green, blue;

  tcs.getRawData(&red, &green, &blue, &clear);

  int colorTemp = tcs.calculateColorTemperature(red,green,blue);

if(colorTemp <= 3000) {
  colorValue=1; //indicating red

}
else {
  colorValue=0; //indicating other colors

  } 
}