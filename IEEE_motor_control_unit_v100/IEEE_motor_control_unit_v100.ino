#include <Servo.h>

//Parallax Continuous Rotation Servo Motors 
const int pin_motorBank_NS_1 = 2;  
const int pin_motorBank_NS_2 = 5;
const int pin_motorBank_EW_1 = 9; 
const int pin_motorBank_EW_2 = 13;
//HC-SR04 Arduino Ultrasonic Sensors (2 sensors in x and y direction)
const int distanceSensorY1_TriggerPin = 2;
const int distanceSensorY1_EchoPin = 3;
const int distanceSensorY2_TriggerPin = 4;
const int distanceSensorY2_EchoPin = 5;
const int distanceSensorX1_TriggerPin = 7;
const int distanceSensorX1_EchoPin = 8;
const int distanceSensorX2_TriggerPin = 12;
const int distanceSensorX2_EchoPin = 13;


Servo motorBank_NS_1; //Motor bank that physically moves the robot in the North and South directions (3 and 4 in the diagram)
Servo motorBank_NS_2;
Servo motorBank_EW_1; //Motor bank that physically moves the robot in the East and West directions (1 and 2 in the diagram)
Servo motorBank_EW_2;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(9600);

  motorBank_NS_1.attach(pin_motorBank_NS_1, 1300, 1700); //Parallax continuous rotation motor, responds to PWM durations between 1300 and 1700 ms
  motorBank_NS_2.attach(pin_motorBank_NS_2, 1300, 1700);
  motorBank_EW_1.attach(pin_motorBank_EW_1, 1300, 1700); //Parallax continuous rotation motor, responds to PWM durations between 1300 and 1700 ms
  motorBank_EW_1.attach(pin_motorBank_EW_2, 1300, 1700);

  pinMode(distanceSensorY1_TriggerPin, OUTPUT);
  pinMode(distanceSensorY1_EchoPin, INPUT);
  pinMode(distanceSensorY2_TriggerPin, OUTPUT);
  pinMode(distanceSensorY2_EchoPin, INPUT);
  pinMode(distanceSensorX1_TriggerPin, OUTPUT);
  pinMode(distanceSensorX1_EchoPin, INPUT);
  pinMode(distanceSensorX2_TriggerPin, OUTPUT);
  pinMode(distanceSensorX2_EchoPin, INPUT);

}

void loop() {
   
  motorBank_NS_1.writeMicroseconds(1700);
  motorBank_NS_2.writeMicroseconds(1700);
  motorBank_EW_1.writeMicroseconds(1700);
  motorBank_EW_2.writeMicroseconds(1700);

}  
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void moveSouthTimed(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_EW_1.write(90); 
  motorBank_EW_2.write(90); //Lock up perpendicular motor bank
  motorBank_NS_1.write(180); 
  motorBank_NS_2.write(0); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);

  motorBank_NS_1.write(90); 
  motorBank_NS_2.write(90);

}

//Move North indefinitely
void moveSouth() {

  motorBank_EW_1.write(90); 
  motorBank_EW_2.write(90); //Lock up perpendicular motor bank
  motorBank_NS_1.write(180); 
  motorBank_NS_2.write(0); //Rotate motors in North and South direction at full speed   

}

void moveNorthTimed(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_EW_1.write(90); 
  motorBank_EW_2.write(90); //Lock up perpendicular motor bank
  motorBank_NS_1.write(0); 
  motorBank_NS_2.write(180); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);

  motorBank_NS_1.write(90); 
  motorBank_NS_2.write(90);

}


//Move South indefinitely
void moveNorth() {

  motorBank_EW_1.write(90); 
  motorBank_EW_2.write(90); //Lock up perpendicular motor bank
  motorBank_NS_1.write(0); 
  motorBank_NS_2.write(180); //Rotate motors in North and South direction at full speed   
}




void moveEastTimed(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_NS_1.write(90); 
  motorBank_NS_2.write(90);//Lock up perpendicular motor bank
  motorBank_EW_1.write(180); 
  motorBank_EW_2.write(180); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);

  motorBank_EW_1.write(90); 
  motorBank_EW_2.write(90);
}

void moveEast() {

  motorBank_NS_1.write(90); 
  motorBank_NS_2.write(90);//Lock up perpendicular motor bank
  motorBank_EW_1.write(180); 
  motorBank_EW_2.write(180); //Rotate motors in North and South direction at full speed   
}


void moveWestTimed(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_NS_1.write(90); 
  motorBank_NS_2.write(90);//Lock up perpendicular motor bank
  motorBank_EW_1.write(0); 
  motorBank_EW_2.write(0); //Rotate motors in North and South direction at full speed   

  do {

    currentMilliseconds = millis();
  } 
  while(currentMilliseconds - previousMilliseconds < durationMilliseconds);

  motorBank_EW_1.write(90); 
  motorBank_EW_2.write(90);
}

void moveWest() {
  motorBank_NS_1.write(90); 
  motorBank_NS_2.write(90);//Lock up perpendicular motor bank
  motorBank_EW_1.write(0); 
  motorBank_EW_2.write(0); //Rotate motors in North and South direction at full speed 
}

void haltTimed(long durationMilliseconds) {

  unsigned long previousMilliseconds = millis(); // will store time before motor bank was turned on
  unsigned long currentMilliseconds;

  motorBank_NS_1.write(90); 
  motorBank_NS_2.write(90);//Lock up perpendicular motor bank
  motorBank_EW_1.write(90); 
  motorBank_EW_2.write(90); //Rotate motors in North and South direction at full speed   

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

//Returns an average of both of the Y-direction ultrasonic sensors, giving a distance in cm from the starting wall (with the hoop)
long getDistanceCentimetersY() {

  long duration1;
  long distance1;
  long duration2;
  long distance2;
  digitalWrite(distanceSensorY1_TriggerPin, LOW);  
  digitalWrite(distanceSensorY2_TriggerPin, LOW);
  delayMicroseconds(2); // Added this line
  digitalWrite(distanceSensorY1_TriggerPin, HIGH);   
  digitalWrite(distanceSensorY2_TriggerPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(distanceSensorY1_TriggerPin, LOW);   
  digitalWrite(distanceSensorY2_TriggerPin, LOW);
  duration1 = pulseIn(distanceSensorY1_EchoPin, HIGH);
  duration2 = pulseIn(distanceSensorY2_EchoPin, HIGH);

  distance1 = (duration1/2) / 29.1;
  distance2 = (duration2/2) / 29.1;

  return (distance1 + distance2)/2;
}

//Returns an average of both of the X-direction ultrasonic sensors, giving a distance in cm from the side wall (closest to the starting gate)
long getDistanceCentimetersX() {

  long duration1; 
  long distance1;
  long duration2;
  long distance2;
  digitalWrite(distanceSensorX1_TriggerPin, LOW);  
  digitalWrite(distanceSensorX2_TriggerPin, LOW);
  delayMicroseconds(2); // Added this line
  digitalWrite(distanceSensorX1_TriggerPin, HIGH);   
  digitalWrite(distanceSensorX2_TriggerPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(distanceSensorX1_TriggerPin, LOW);   
  digitalWrite(distanceSensorX2_TriggerPin, LOW);
  duration1 = pulseIn(distanceSensorX1_EchoPin, HIGH);
  duration2 = pulseIn(distanceSensorX2_EchoPin, HIGH);

  distance1 = (duration1/2) / 29.1;
  distance2 = (duration2/2) / 29.1;

  return (distance1 + distance2)/2;
}













