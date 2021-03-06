#include <Servo.h>
#include <math.h>

const int turretHeight = 36.5; //used in physics calculations for aiming turret, cm
const int hoopZ = 71.12; //used in physics calculations for aiming turret, cm
const int hoopX = 62.23; //used in physics calculations for aiming turret, cm
const int distanceMainLineToWall = 13; //hardcoded, passed in to moveNorthAlignedBy 
//const int distanceToFirstIntersection = ; //hardcoded, passed in to moveNorthAlignedBy
//const int distanceToSecondIntersection = ; //hardcoded, passed in to moveNorthAlignedBy
//const int distanceToThirdIntersection = ; //hardcoded, passed in to moveNorthAlignedBy
//Parallax Continuous Rotation Servo Motors 
const int pin_motorBank_NS_1 = 2;  
const int pin_motorBank_NS_2 = 3;
const int pin_motorBank_EW_1 = 4; 
const int pin_motorBank_EW_2 = 5;
//HC-SR04 Arduino Ultrasonic Sensors (2 sensors in x, 1 in y direction, 1 towards the block side)
const int distanceSensorY1_TriggerPin = 52;
const int distanceSensorY1_EchoPin = 53;
const int distanceSensorX1_TriggerPin = 44;
const int distanceSensorX1_EchoPin = 45;
const int distanceSensorX2_TriggerPin = 22;
const int distanceSensorX2_EchoPin = 24;
const int distanceSensorB1_TriggerPin = 32; // white
const int distanceSensorB1_EchoPin = 33;  // blue
//TowerPro SG-5010 
const int pin_turret_Rotation = 6;
const int pin_turret_Tilt = 13;
//Turret trigger and flywheels
const int turret_Trigger = 11;
const int turret_Flywheels = 12;

Servo motorBank_NS_1; //Motor bank that physically moves the robot in the North and South directions (3 and 4 in the diagram)
Servo motorBank_NS_2;
Servo motorBank_EW_1; //Motor bank that physically moves the robot in the East and West directions (1 and 2 in the diagram)
Servo motorBank_EW_2;

Servo turret_Rotation;
Servo turret_Tilt;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {

  //Initialize serial communication
  Serial.begin(9600);

  //Initialize servo motors
  motorBank_NS_1.attach(pin_motorBank_NS_1); //Parallax continuous rotation motor, responds to PWM durations between 1300 and 1700 ms
  motorBank_NS_2.attach(pin_motorBank_NS_2);
  motorBank_EW_1.attach(pin_motorBank_EW_1); //Parallax continuous rotation motor, responds to PWM durations between 1300 and 1700 ms
  motorBank_EW_2.attach(pin_motorBank_EW_2);
  turret_Rotation.attach(pin_turret_Rotation, 600, 2400);
  turret_Tilt.attach(pin_turret_Tilt, 600, 2400);

  //Initialize distance sensors
  pinMode(distanceSensorY1_TriggerPin, OUTPUT);
  pinMode(distanceSensorY1_EchoPin, INPUT);
  pinMode(distanceSensorX1_TriggerPin, OUTPUT);
  pinMode(distanceSensorX1_EchoPin, INPUT);
  pinMode(distanceSensorX2_TriggerPin, OUTPUT);
  pinMode(distanceSensorX2_EchoPin, INPUT);
  pinMode(distanceSensorB1_TriggerPin, OUTPUT);
  pinMode(distanceSensorB1_EchoPin, INPUT);

  //Initialize turret flywheel motors (don't use servo library)
  pinMode(turret_Trigger,OUTPUT);
  pinMode(turret_Flywheels,OUTPUT);

  //Set all motors to rest and servos to 90 degrees
  motorBank_NS_1.writeMicroseconds(1500); //initially set to rest
  motorBank_NS_2.writeMicroseconds(1500); //initially set to rest
  motorBank_EW_1.writeMicroseconds(1500); //initially set to rest
  motorBank_EW_2.writeMicroseconds(1500); //initially set to rest
  turret_Rotation.write(90); //initially set to rest
  turret_Tilt.write(90); //initially set to rest
}

void loop()
{

  debugDistanceSensorsSmoothed();
  aimTurret(calculateTurretAngleRotation(), calculateTurretAngleTilt());
  nonBlockingDelay(10000);


  // moveNorthAlignedBy(100,8);

  //  moveNorthOnMainLineV2(8);

  //moveWestAligned();

  //aimTurret(calculateTurretAngleRotation(), calculateTurretAngleTilt());
  //nonBlockingDelay(10000); //10 sec delay
  //  moveNorthOnMainLine(6);


  //  delay(100000000);
}  
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void moveSouthTimed(long durationMilliseconds) {


  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500); //Lock up perpendicular motor bank
  motorBank_NS_1.writeMicroseconds(1700); 
  motorBank_NS_2.writeMicroseconds(1300); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);

  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);
}

//Move North indefinitely
void moveSouth() {

  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500); //Lock up perpendicular motor bank
  motorBank_NS_1.writeMicroseconds(1700); 
  motorBank_NS_2.writeMicroseconds(1300); //Rotate motors in North and South direction at full speed   

}

void moveNorthTimed(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500); //Lock up perpendicular motor bank
  motorBank_NS_1.writeMicroseconds(1450); 
  motorBank_NS_2.writeMicroseconds(1550); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);

  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);
}

//Move South indefinitely
void moveNorth() {

  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500); //Lock up perpendicular motor bank
  motorBank_NS_1.writeMicroseconds(1300); 
  motorBank_NS_2.writeMicroseconds(1700); //Rotate motors in North and South direction at full speed   
}

void moveEastTimed(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);//Lock up perpendicular motor bank
  motorBank_EW_1.writeMicroseconds(1700); 
  motorBank_EW_2.writeMicroseconds(1300); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);

  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500);
}

void moveEast() {

  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);//Lock up perpendicular motor bank
  motorBank_EW_1.writeMicroseconds(1700); 
  motorBank_EW_2.writeMicroseconds(1300); //Rotate motors in North and South direction at full speed   
}


void moveWestTimed(long durationMilliseconds) {


  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);//Lock up perpendicular motor bank
  motorBank_EW_1.writeMicroseconds(1300); 
  motorBank_EW_2.writeMicroseconds(1700); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);

  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500);
}

void moveWest() {
  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);//Lock up perpendicular motor bank
  motorBank_EW_1.writeMicroseconds(1300); 
  motorBank_EW_2.writeMicroseconds(1700); //Rotate motors in North and South direction at full speed 
}

void haltTimed(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);//Lock up perpendicular motor bank
  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);
}

void debugUnit() {

  char inByte;
  // Input serial information:
  if (Serial.available() > 0){
    inByte = Serial.read();

    if (inByte == 'N') {
      moveNorthTimed(1000); 
      Serial.println("N");
    }
    else if (inByte == 'E') {
      moveEastTimed(1000); 
      Serial.println("E");
    }
    else if (inByte == 'S') {
      moveSouthTimed(1000);
      Serial.println("S"); 
    }
    else if (inByte == 'W') {
      moveWestTimed(1000);
      Serial.println("W");
    }
  }
}

void debugDistanceSensors() {

  Serial.print("X1: "); 
  Serial.print(getDistanceX1());
  delay(1);
  Serial.print(" X2: "); 
  Serial.print(getDistanceX2());
  Serial.print(" Y: "); 
  Serial.print(getDistanceY());
  Serial.println();
  delay(100);
}

void debugDistanceSensorsSmoothed() {

  Serial.print("X1: "); 
  Serial.print(getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin));
  delay(9);
  Serial.print(" X2: "); 
  Serial.print(getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin));
  Serial.print(" Y: "); 
  Serial.print(getDistanceSensorSmoothed(distanceSensorY1_TriggerPin, distanceSensorY1_EchoPin));
  Serial.print(" B1: ");
  Serial.print(getDistanceSensorSmoothed(distanceSensorB1_TriggerPin, distanceSensorB1_EchoPin));
  Serial.println();
  delay(100);
}

//Returns an average of both of the Y-direction ultrasonic sensors, giving a distance in cm from the starting wall (with the hoop)
long getDistanceY() {

  unsigned long previousMicroseconds;  // will store time before pulse is sent
  unsigned long currentMicroseconds;
  long durationFirstPulse = 2;
  long durationSecondPulse = 10;  
  long securityPause = 50; //50us delay to ensure that the sensor isn't overloaded 
  long duration1;
  long distance1;

  digitalWrite(distanceSensorY1_TriggerPin, LOW);  

  //delayMicroseconds(2); // Added this line
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < durationFirstPulse);

  digitalWrite(distanceSensorY1_TriggerPin, HIGH);   

  //delayMicroseconds(10); // Added this line
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < durationSecondPulse);

  digitalWrite(distanceSensorY1_TriggerPin, LOW);   

  duration1 = pulseIn(distanceSensorY1_EchoPin, HIGH);

  distance1 = (duration1/2) / 29.1;

  //delayMicroseconds(50);
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < securityPause);

  return distance1;
}

//Returns an average of both of the X-direction ultrasonic sensors, giving a distance in cm from the side wall (closest to the starting gate)
long getDistanceX1() { 

  unsigned long previousMicroseconds;  // will store time before pulse is sent
  unsigned long currentMicroseconds;
  long durationFirstPulse = 2;
  long durationSecondPulse = 10;
  long securityPause = 50; //50us delay to ensure that the sensor isn't overloaded 
  long duration1; 
  long distance1;

  digitalWrite(distanceSensorX1_TriggerPin, LOW);

  // delayMicroseconds(2); // Added this line
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < durationFirstPulse);

  digitalWrite(distanceSensorX1_TriggerPin, HIGH);   

  //delayMicroseconds(10); // Added this line
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < durationSecondPulse);

  digitalWrite(distanceSensorX1_TriggerPin, LOW);   
  duration1 = pulseIn(distanceSensorX1_EchoPin, HIGH);

  distance1 = (duration1/2) / 29.1;

  //delayMicroseconds(50);
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < securityPause);

  return distance1;
}

long getDistanceX2() {

  unsigned long previousMicroseconds;  // will store time before pulse is sent
  unsigned long currentMicroseconds;
  long durationFirstPulse = 2;
  long durationSecondPulse = 10;
  long securityPause = 50; //50us delay to ensure that the sensor isn't overloaded 
  long duration2;
  long distance2;

  digitalWrite(distanceSensorX2_TriggerPin, LOW);

  // delayMicroseconds(2); // Added this line
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < durationFirstPulse);

  digitalWrite(distanceSensorX2_TriggerPin, HIGH);

  //delayMicroseconds(10); // Added this line
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < durationSecondPulse);

  digitalWrite(distanceSensorX2_TriggerPin, LOW);

  duration2 = pulseIn(distanceSensorX2_EchoPin, HIGH);
  distance2 = (duration2/2) / 29.1;

  //delayMicroseconds(50);
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < securityPause);

  return distance2;
}

//Moves north with correction from the distance sensors
void moveNorthCorrected() {

  long distanceX1;
  long distanceX2;

  moveNorthTimed(200);

  distanceX1 = getDistanceX1();
  // Serial.print("X1: "); Serial.print(distanceX1);
  delay(1);
  distanceX2 = getDistanceX2();
  // Serial.print(" X2: "); Serial.println(distanceX2);

  if(((distanceX1 - distanceX2) > 3) || (distanceX1 - distanceX2) <  -3)  {    //if the robot goes off course by 3cm in either direction
    haltTimed(150); //stop
    Serial.print("Correcting...");
    //Correct for imbalance
    if(distanceX1 > distanceX2) {
      motorBank_EW_1.detach();
      motorBank_EW_2.detach();
      motorBank_NS_1.writeMicroseconds(1450); //slowly move correct side to straighten out robot
      motorBank_NS_2.writeMicroseconds(1450);
      do
      {
        distanceX2 = getDistanceX2();

      } 
      while(distanceX1 > distanceX2);

      motorBank_EW_1.attach(pin_motorBank_EW_1);
      motorBank_EW_2.attach(pin_motorBank_EW_2);
      moveNorthTimed(200);
    }

    if (distanceX1 < distanceX2) {
      motorBank_EW_1.detach();
      motorBank_EW_2.detach();
      motorBank_NS_2.writeMicroseconds(1550); //slowly move correct side to straighten out robot
      motorBank_NS_1.writeMicroseconds(1550);
      do
      {
        distanceX1 = getDistanceX1();
      } 
      while(distanceX1 < distanceX2); 

      motorBank_EW_1.attach(pin_motorBank_EW_1);
      motorBank_EW_2.attach(pin_motorBank_EW_2);
      moveNorthTimed(200);
    }
  }
}

void straighten() {

  int success = 3;

  long distanceX1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);
  //nonBlockingDelay(1);
  delay(1);
  long distanceX2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);

  while (abs(distanceX1 - distanceX2) >= 1 && success > 0) {

    if (abs(distanceX1 - distanceX2) == 1) {
      success--;
    }

    if(distanceX1 < distanceX2) {
      //detach every other motor
      motorBank_NS_1.detach();
      motorBank_NS_2.detach();
      motorBank_EW_2.writeMicroseconds(1500);
      motorBank_EW_1.writeMicroseconds(1525);
    } 
    else {
      motorBank_NS_1.detach();
      motorBank_NS_2.detach();
      motorBank_EW_1.writeMicroseconds(1500);
      motorBank_EW_1.writeMicroseconds(1525);    
    }
    nonBlockingDelay(30);
    motorBank_NS_1.attach(pin_motorBank_NS_1);
    motorBank_NS_2.attach(pin_motorBank_NS_2);
    motorBank_NS_1.writeMicroseconds(1500);
    motorBank_NS_2.writeMicroseconds(1500);
    motorBank_EW_1.writeMicroseconds(1500);
    motorBank_EW_2.writeMicroseconds(1500);

    distanceX1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);
    // nonBlockingDelay(1);
    delay(10);
    distanceX2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);
  } 

  motorBank_EW_1.attach(pin_motorBank_EW_1);
  motorBank_EW_1.attach(pin_motorBank_EW_2);
  haltTimed(10);
}

void nonBlockingDelay(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); 
  unsigned long currentMilliseconds;

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);
}


long getDistanceSensor(int trigPin, int echoPin) {

  unsigned long previousMicroseconds;  // will store time before pulse is sent
  unsigned long currentMicroseconds;
  long durationFirstPulse = 2;
  long durationSecondPulse = 10;
  long securityPause = 50; //50us delay to ensure that the sensor isn't overloaded 
  long duration;
  long distance;

  digitalWrite(trigPin, LOW);

  // delayMicroseconds(2); // Added this line
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < durationFirstPulse);

  digitalWrite(trigPin, HIGH);

  //delayMicroseconds(10); // Added this line
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < durationSecondPulse);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

  //delayMicroseconds(50);
  previousMicroseconds =  micros();
  do {

    currentMicroseconds = micros();
  } 
  while(currentMicroseconds - previousMicroseconds < securityPause);

  return distance;     
}

int getDistanceSensorSmoothed(int trigPin, int echoPin) {

  double total = 0;
  const double numberOfRuns = 7;

  for(int i = 0; i < numberOfRuns; i++) {

    total += getDistanceSensor(trigPin, echoPin); 
    nonBlockingDelay(10);
  }

  double average = total/numberOfRuns;
  if (average-(int)average > 0.5) {
    return (int)average + 1;
  } 
  else {
    return (int)average; 
  } 
}

void moveNorthOnMainLine(int distanceMLToWall)
{
  moveNorthTimed(400);
  long distanceX1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);

  if (distanceX1 != distanceMLToWall)
  {
    if (distanceX1 < distanceMLToWall)
    {
      // lock EW1
      motorBank_EW_1.writeMicroseconds(1500); 

      // dettach NS1 and NW2
      motorBank_NS_1.detach();
      motorBank_NS_2.detach();

      // move EW2 west for say 50 ms at slow speed

      do
      {
        Serial.println(distanceX1);
        motorBank_EW_2.writeMicroseconds(1525); 
        nonBlockingDelay(50);
        distanceX1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);
      }
      while(distanceX1 != distanceMLToWall && distanceX1 < (distanceMLToWall + 1));
      Serial.print("final distance: ");
      Serial.println(distanceX1);
      motorBank_NS_1.attach(pin_motorBank_NS_1);
      motorBank_NS_2.attach(pin_motorBank_NS_2);
    }
    if (distanceX1 > distanceMLToWall)
    {
      // lock EW1
      motorBank_EW_1.writeMicroseconds(1500); 

      // dettach NS1 and NW2
      motorBank_NS_1.detach();
      motorBank_NS_2.detach();

      // move EW2 east for say 50 ms at slow speed

      do
      {
        Serial.println(distanceX1);
        motorBank_EW_2.writeMicroseconds(1475); 
        nonBlockingDelay(50);
        distanceX1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);
      }
      while(distanceX1 != distanceMLToWall && distanceX1 > (distanceMLToWall + 1));
      Serial.print("final distance: ");
      Serial.println(distanceX1);
      motorBank_NS_1.attach(pin_motorBank_NS_1);
      motorBank_NS_2.attach(pin_motorBank_NS_2);
    }
  }

  //ALIGN EW1
  long distanceX2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);

  if (distanceX2 != distanceMLToWall)
  {
    if (distanceX2 < distanceMLToWall)
    {
      // lock EW2
      motorBank_EW_2.writeMicroseconds(1500); 

      // dettach NS1 and NW2
      motorBank_NS_1.detach();
      motorBank_NS_2.detach();

      // move EW1 west for say 50 ms at slow speed
      do
      {
        Serial.println(distanceX2);
        motorBank_EW_1.writeMicroseconds(1475); 
        nonBlockingDelay(50);
        distanceX2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);
      }
      while(distanceX2 != distanceMLToWall && distanceX2 < (distanceMLToWall + 1));
      Serial.print("final distance: ");
      Serial.println(distanceX2);
      motorBank_NS_1.attach(pin_motorBank_NS_1);
      motorBank_NS_2.attach(pin_motorBank_NS_2);
    }
    if (distanceX2 > distanceMLToWall)
    {
      // lock EW2
      motorBank_EW_2.writeMicroseconds(1500); 

      // dettach NS1 and NW2
      motorBank_NS_1.detach();
      motorBank_NS_2.detach();

      // move EW1 east for say 50 ms at slow speed
      do
      {
        Serial.println(distanceX2);
        motorBank_EW_1.writeMicroseconds(1525); 
        nonBlockingDelay(50);
        distanceX2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);
      }
      while(distanceX2 != distanceMLToWall && distanceX2 > (distanceMLToWall + 1));
      Serial.print("final distance: ");
      Serial.println(distanceX2);
      motorBank_NS_1.attach(pin_motorBank_NS_1);
      motorBank_NS_2.attach(pin_motorBank_NS_2);
    }
  }   
}

void moveWestAligned()
{
  // move east by time, say 100 ms
  moveWestTimed(100);

  //get x1 and x2
  long X1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);
  nonBlockingDelay(10);
  long X2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);

  // either x1 < x2, slanted south
  if (X1 < X2-1)
  {
    // lock ew1, detach nw1 nw2
    motorBank_EW_1.writeMicroseconds(1500); 
    motorBank_NS_1.detach();
    motorBank_NS_2.detach();

    // move ew2 west while x1 = x2

    while(X1 < X2)
    {
      //motorBank_EW_2.writeMicroseconds(1510); 
      motorBank_EW_2.writeMicroseconds(1525); 
      nonBlockingDelay(20);
      //  motorBank_EW_2.writeMicroseconds(1500); 
      X2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);
      // delay(3);
      X1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);
    }

    motorBank_NS_1.attach(pin_motorBank_NS_1);
    motorBank_NS_2.attach(pin_motorBank_NS_2);
    motorBank_EW_2.writeMicroseconds(1500);
  }
  // or x1 > x2
  if (X1 > X2+1)
  {
    // lock ew2, detach nw1 nw2
    motorBank_EW_2.writeMicroseconds(1500); 
    motorBank_NS_1.detach();
    motorBank_NS_2.detach();

    // move ew1 west while x1 = x2

    while(X1 > X2)
    {
      //motorBank_EW_1.writeMicroseconds(1490); 
      motorBank_EW_1.writeMicroseconds(1475); 
      nonBlockingDelay(20);
      //  motorBank_EW_2.writeMicroseconds(1500); 
      X2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);
      //delay(3);
      X1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);
    }  

    motorBank_NS_1.attach(pin_motorBank_NS_1);
    motorBank_NS_2.attach(pin_motorBank_NS_2);
    motorBank_EW_1.writeMicroseconds(1500);  
  }
}

void moveNorthAlignedBy(int yCoordinate, int distanceMLToWall)
{
  long distanceY1 = getDistanceSensorSmoothed(distanceSensorY1_TriggerPin, distanceSensorY1_EchoPin);
  while(distanceY1 < yCoordinate)
  {
    moveNorthOnMainLine(distanceMLToWall);
    distanceY1 = getDistanceSensorSmoothed(distanceSensorY1_TriggerPin, distanceSensorY1_EchoPin);
  }
  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500);
  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500); 
}

//Angles less than 90 correspond to rotating to the right and tilting up
void aimTurret(int rotationAngle, int tiltAngle) {

  if(turret_Rotation.read() > rotationAngle) {

    for(int i = turret_Rotation.read(); i >= rotationAngle; i--) {

      turret_Rotation.write(i);
      nonBlockingDelay(40); 

    }
    Serial.println(turret_Rotation.read());
  } 
  else {

    for(int i = turret_Rotation.read(); i <= rotationAngle; i++) {

      turret_Rotation.write(i);
      nonBlockingDelay(40); 

    }
    Serial.println(turret_Rotation.read());
  }

  nonBlockingDelay(200);

  if(turret_Tilt.read() > tiltAngle) {

    for(int i = turret_Tilt.read(); i >= tiltAngle; i--) {

      turret_Tilt.write(i);
      nonBlockingDelay(40); 
    }
    Serial.println(turret_Tilt.read());
  } 
  else {

    for(int i = turret_Tilt.read(); i <= tiltAngle; i++) {

      turret_Tilt.write(i);
      nonBlockingDelay(40); 
    }
    Serial.println(turret_Tilt.read());
  }
}

void fire() // The higher the PWM values, the more current the motors get. Have to calibrate these.
{
  analogWrite(turret_Flywheels, 255); //PWM to control the speed of the motors
  nonBlockingDelay(2200); //Allow sufficient time for the motors to get to full speed

  analogWrite(turret_Trigger, 255); //Send a short signal to fire the gun
  nonBlockingDelay(350);
  analogWrite(turret_Trigger, 0); //Set all motors back to 0
  analogWrite(turret_Flywheels, 0);
}

int calculateTurretAngleRotation() {

  double distanceX1 = getDistanceSensorSmoothed(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin);
  nonBlockingDelay(10);
  double distanceX2 = getDistanceSensorSmoothed(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin);
  double trueXDistance = (distanceX1 + distanceX2) / 2.0;

  double distanceY = getDistanceSensorSmoothed(distanceSensorY1_TriggerPin, distanceSensorY1_EchoPin);

  return (int)atan2(distanceY, (trueXDistance - hoopX));
}

int calculateTurretAngleTilt() {

  double distanceY = getDistanceSensorSmoothed(distanceSensorY1_TriggerPin, distanceSensorY1_EchoPin);

  return (int)atan2((hoopZ - turretHeight), distanceY);
}


//---------------------------------------------------------------------------
//          Test Codes
//---------------------------------------------------------------------------
/*
  // Turret Test Code
 aimTurret(90, 90);
 delay(2000);
 aimTurret(45, 45);
 delay(2000);
 aimTurret(135, 135);
 delay(2000);
 aimTurret(180, 180);
 */
//---------------------------------------------------------------------------





