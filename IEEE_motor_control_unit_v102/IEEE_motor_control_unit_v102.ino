#include <Servo.h>

//Parallax Continuous Rotation Servo Motors 
const int pin_motorBank_NS_1 = 2;  
const int pin_motorBank_NS_2 = 3;
const int pin_motorBank_EW_1 = 7; 
const int pin_motorBank_EW_2 = 8;
//HC-SR04 Arduino Ultrasonic Sensors (2 sensors in x, 1 in y direction)
const int distanceSensorY1_TriggerPin = 52;
const int distanceSensorY1_EchoPin = 53;
const int distanceSensorX1_TriggerPin = 44;
const int distanceSensorX1_EchoPin = 45;
const int distanceSensorX2_TriggerPin = 22;
const int distanceSensorX2_EchoPin = 24;


Servo motorBank_NS_1; //Motor bank that physically moves the robot in the North and South directions (3 and 4 in the diagram)
Servo motorBank_NS_2;
Servo motorBank_EW_1; //Motor bank that physically moves the robot in the East and West directions (1 and 2 in the diagram)
Servo motorBank_EW_2;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(9600);

  motorBank_NS_1.attach(pin_motorBank_NS_1); //Parallax continuous rotation motor, responds to PWM durations between 1300 and 1700 ms
  motorBank_NS_2.attach(pin_motorBank_NS_2);
  motorBank_EW_1.attach(pin_motorBank_EW_1); //Parallax continuous rotation motor, responds to PWM durations between 1300 and 1700 ms
  motorBank_EW_2.attach(pin_motorBank_EW_2);

  pinMode(distanceSensorY1_TriggerPin, OUTPUT);
  pinMode(distanceSensorY1_EchoPin, INPUT);
  pinMode(distanceSensorX1_TriggerPin, OUTPUT);
  pinMode(distanceSensorX1_EchoPin, INPUT);
  pinMode(distanceSensorX2_TriggerPin, OUTPUT);
  pinMode(distanceSensorX2_EchoPin, INPUT);

  motorBank_NS_1.writeMicroseconds(1500); //initially set to rest
  motorBank_NS_2.writeMicroseconds(1500); //initially set to rest
  motorBank_EW_1.writeMicroseconds(1500); //initially set to rest
  motorBank_EW_2.writeMicroseconds(1500); //initially set to rest

}

void loop() {

  
  while(getDistanceY() < 30) {
    moveNorthCorrected();
  }
  
  //moveSouth();
  
  
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
  motorBank_NS_1.writeMicroseconds(1300); 
  motorBank_NS_2.writeMicroseconds(1700); //Rotate motors in North and South direction at full speed   

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

  getDistanceX1();
  delay(1);
  getDistanceX2();
  getDistanceY();
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

  Serial.print("Distance Y : ");
  Serial.print(distance1);
  Serial.print(" ");
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

  Serial.print("Distance X1: ");
  Serial.print(distance1);
  Serial.print(" ");
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

  Serial.print("Distance X2: ");
  Serial.print(distance2);
  Serial.print(" ");
  return distance2;
}





//Moves north with correction from the distance sensors
void moveNorthCorrected() {

  long distanceX1;
  long distanceX2;

  moveNorthTimed(200);

  distanceX1 = getDistanceX1();
  distanceX2 = getDistanceX2();

  if(((distanceX1 - distanceX2) > 3) || (distanceX1 - distanceX2) > -3)  {    //if the robot goes off course by 3cm in either direction
    haltTimed(150); //stop

    //Correct for imbalance
    if(distanceX1 > distanceX2) {
      motorBank_NS_1.writeMicroseconds(1350); //slowly move correct side to straighten out robot
      motorBank_NS_2.writeMicroseconds(1350);
      do
      {
        distanceX2 = getDistanceX2();

      } 
      while(distanceX1 > distanceX2);

      moveNorthTimed(200);
    }

    if (distanceX1 < distanceX2) {
      motorBank_NS_2.writeMicroseconds(1650); //slowly move correct side to straighten out robot
      motorBank_NS_1.writeMicroseconds(1650);
      do
      {
        distanceX1 = getDistanceX1();
      } 
      while(distanceX1 < distanceX2); 

      moveNorthTimed(200);
    }
  }


}


















