#include <NewPing.h>
#include <Servo.h>
#include <math.h>

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// LEGEND
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// * GV  = Global variables, pins, servos, and sensors
// * SU  = Setup
// * LO  = Loop
// * PC  = Play code
// * UF  = Universal functions, such as nonBlockingDelay
// * DS  = Distance sensing
// * MB  = Basic move
// * MA  = Advanced move
// * TC  = Turret controls
// * DB  = Debug/Calibration
//
// Use Ctrl + F and *XX (without the extra space)to quickly jump to section

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBAL VARIABLES, PINS, SERVOS, AND SENSORS *GV
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// COURSE DIMENSIONS
const double TURRET_HEIGHT = 36.25; //36.5 //used in physics calculations for aiming turret, cm
const double TURRET_CENTER_Y = 14.0; //used in physics calculations for aiming turret, cm
const double TURRET_CENTER_X = 14.0; //used in physics calculations for aiming turret, cm
const double HOOP_Z = 71.1; //used in physics calculations for aiming turret, cm
const double HOOP_X = 60.5; //used in physics calculations for aiming turret, cm
const int X_TO_WALL = 8; //hardcoded, passed in to moveNorthByTo 
const int FIRST_INTERSECTION = 84; //hardcoded, passed in to moveNorthByTo
//const int SECOND_INTERSECTION = ; //hardcoded, passed in to moveNorthByTo
const int THIRD_INTERSECTION = 149; //hardcoded, passed in to moveNorthByTo
const int B_TO_WALL = 80; // Threshold value for B1. B1 < this value means blocked detected

//Parallax Continuous Rotation Servo Motors 
const int pin_motorBank_NS_1 = 2;
const int pin_motorBank_NS_2 = 3;
const int pin_motorBank_EW_1 = 4;
const int pin_motorBank_EW_2 = 5;

//HC-SR04 Arduino Ultrasonic Sensors (2 sensors in x (X1 is closest to hoop), 1 in y direction, 1 towards the block side)
const int distanceSensorY1_TriggerPin = 52;
const int distanceSensorY1_EchoPin = 53;
const int distanceSensorX1_TriggerPin = 44;
const int distanceSensorX1_EchoPin = 45;
const int distanceSensorX2_TriggerPin = 22;
const int distanceSensorX2_EchoPin = 24;
const int distanceSensorB1_TriggerPin = 32; // white
const int distanceSensorB1_EchoPin = 33;  // blue
const int maximumDistance = 300; //used in the NewPing library

//TowerPro SG-5010 
const int pin_turret_Rotation = 6;
const int pin_turret_Tilt = 13;

//Turret trigger and flywheels
const int turret_Trigger = 11;
const int turret_Flywheels = 12;

Servo motorBank_NS_1; // Corresponds to NS motor furthest from hoop (at starting position)
Servo motorBank_NS_2; // Corresponds to NS motor closest to hoop (at starting position)
Servo motorBank_EW_1; // Corresponds to EW motor furthest from hoop
Servo motorBank_EW_2; // Corresponds to EW motor closest from hoop

Servo turret_Rotation;
Servo turret_Tilt;

NewPing sonarY1(distanceSensorY1_TriggerPin, distanceSensorY1_EchoPin, maximumDistance); // NewPing setup of pins and maximum distance.
NewPing sonarX1(distanceSensorX1_TriggerPin, distanceSensorX1_EchoPin, maximumDistance); // NewPing setup of pins and maximum distance.
NewPing sonarX2(distanceSensorX2_TriggerPin, distanceSensorX2_EchoPin, maximumDistance); // NewPing setup of pins and maximum distance.
NewPing sonarB1(distanceSensorB1_TriggerPin, distanceSensorB1_EchoPin, maximumDistance); // NewPing setup of pins and maximum distance.

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// SETUP *SU
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  //Initialize serial communication
  Serial.begin(9600);

  //Initialize servo motors
  motorBank_NS_1.attach(pin_motorBank_NS_1); //Parallax continuous rotation motor, responds to PWM durations between 1300 and 1700 ms
  motorBank_NS_2.attach(pin_motorBank_NS_2);
  motorBank_EW_1.attach(pin_motorBank_EW_1); //Parallax continuous rotation motor, responds to PWM durations between 1300 and 1700 ms
  motorBank_EW_2.attach(pin_motorBank_EW_2);
  turret_Rotation.attach(pin_turret_Rotation, 600, 2400);
  turret_Tilt.attach(pin_turret_Tilt, 600, 2400);

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

  Serial.println("Constants: ");
  Serial.print("Turret Paramters: Height = ");
  Serial.print(TURRET_HEIGHT);
  Serial.print(" Center X = ");
  Serial.print(TURRET_CENTER_X);
  Serial.print(" Center Y = ");
  Serial.println(TURRET_CENTER_Y);
  
  Serial.print("Hoop parameters: Center Z = ");
  Serial.print(HOOP_Z);
  Serial.print(" Center X = ");
  Serial.println(HOOP_X);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// LOOP *LO
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{
  halt(2000);
  playRound();
  halt(100000);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// PLAY CODE *PC
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

boolean startSignal()
{
  //returns true when start signl is detected, otherwise false
  return true;
}

void playRound()
{
  // wait for the start signal
  while(true)
  {
    if (startSignal)
     {
       Serial.println("Start signal detected!");
       break;
     }
    else
      Serial.println("Signal not yet detected...");
  }
  
  // move to first line
  moveNorthAlignedBy(FIRST_INTERSECTION, X_TO_WALL);
  
  // detect block
  long distanceB1 = getB1();
  long distanceB1ToWestWall = B_TO_WALL - 16.5 + 80;  // 16.5 cm is the distance from B1 sensor to the middle of the robot (specifically the center of the EW wheels  
  
  Serial.print("B1: : ");
  Serial.println(distanceB1);
  debugDistance();
  // general case where the block is not on the most east location
  if (distanceB1 < distanceB1ToWestWall - 15) // -15 cm as a safe tolerance for furthest block
  {
    Serial.println("General case");
    // move to cover block
    moveWestByTo(FIRST_INTERSECTION, X_TO_WALL + distanceB1  + 12); // 12cm is the distance from the B1 sensor to the left most position of EW2's wheel
  }
  else // means the block is on the first location
  {
    Serial.println("Special case");
    
    // move to cover block
    
    // 8 cm is a complete guess
    moveWestByTo(FIRST_INTERSECTION, X_TO_WALL + 12 + 8 -20 + 9 ); // 12cm is the distance from the B1 sensor to the left most position of EW2's wheel; 8 cm is the distance from B1 to the left most position on the block
  }
  
  Serial.println("finished");
  
  // get locations x and y
  // rotate and tilt the turret
  // prime and fire dart 
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// UNIVERSAL FUNCTIONS *UF
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Non-blocking delay in milliseconds
void mDelay(long durationMilliseconds)
{
  unsigned long previousMilliseconds = millis(); 
  unsigned long currentMilliseconds = millis();

  while(currentMilliseconds - previousMilliseconds < durationMilliseconds)
  {
    currentMilliseconds = millis();
  }
}

// Non-blocking delay in microseconds
void uDelay(long durationMicros)
{
  unsigned long previousMicros = micros(); 
  unsigned long currentMicros = micros();

  while(currentMicros - previousMicros < durationMicros)
  {
    currentMicros = micros();
  }
}

// Re-attaches all motor servos and halt
void reattach()
{
  if (!motorBank_NS_1.attached())
    motorBank_NS_1.attach(pin_motorBank_NS_1);
  if (!motorBank_NS_2.attached())
    motorBank_NS_2.attach(pin_motorBank_NS_2);
  if (!motorBank_EW_1.attached())
    motorBank_EW_1.attach(pin_motorBank_EW_1);
  if (!motorBank_EW_2.attached())
    motorBank_EW_2.attach(pin_motorBank_EW_2);
  halt();
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// DISTANCE SENSING *DS
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

long getB1() 
{
  unsigned int uS = sonarB1.ping();// Send ping, get ping time in microseconds (uS)
  mDelay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  return(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range) 
}

// Smooths out the B1 value by averaging several distances
long getB1(int runs)
{
  long total = 0;
  for (int i = 0; i <runs;i++)
  {
    total += getB1();
  }
  return round(total/runs);
}

long getY1()
{
  unsigned int uS = sonarY1.ping();// Send ping, get ping time in microseconds (uS)
  mDelay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  return (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
}

// Smooths out the Y1 value by averaging several distances
long getY1(int runs)
{
  long total = 0;
  for (int i = 0; i <runs;i++)
  {
    total += getY1();
  }
  return round(total/runs);
}

long getX1()
{
  unsigned int uS = sonarX1.ping();// Send ping, get ping time in microseconds (uS)
  mDelay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  return (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
}

// Smooths out the X1 value by averaging several distances
long getX1(int runs)
{
  long total = 0;
  for (int i = 0; i <runs;i++)
  {
    total += getX1();
  }
  return round(total/runs);
}

long getX2()
{
  unsigned int uS = sonarX2.ping();// Send ping, get ping time in microseconds (uS)
  mDelay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  return (uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
}

// Smooths out the X2 value by averaging several distances
long getX2(int runs)
{
  long total = 0;
  for (int i = 0; i <runs;i++)
  {
    total += getX2();
  }
  return round(total/runs);
}

long getX()
{
  long X1 = getX1();
  long X2 = getX2();
  return (X1+X2)/2;
}

// Smooths out the X value by averaging several distances
long getX(int runs)
{
  long total = 0;
  for (int i = 0; i <runs;i++)
  {
    total += getX();
  }
  return round(total/runs);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// BASIC MOVE FUNCTIONS *MB
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void moveSouth()
{
  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500); //Lock up perpendicular motor bank
  motorBank_NS_1.writeMicroseconds(1550); 
  motorBank_NS_2.writeMicroseconds(1440); //Rotate motors in North and South direction at desired speed   
}

void moveSouth(long durationMilliseconds)
{
  moveSouth();
  mDelay(durationMilliseconds);
  halt();
}

void moveNorth() 
{
  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500); //Lock up perpendicular motor bank
  motorBank_NS_1.writeMicroseconds(1446); // Calibrate these so the robot doesn't deviate
  motorBank_NS_2.writeMicroseconds(1550); //Rotate motors in North and South direction at desired speed   
}

void moveNorth(long durationMilliseconds)
{
  moveNorth();
  mDelay(durationMilliseconds);
  halt();
}

void moveEast() 
{
  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);//Lock up perpendicular motor bank
  motorBank_EW_1.writeMicroseconds(1550); 
  motorBank_EW_2.writeMicroseconds(1450); //Rotate motors in East and West direction at desired speed   
}

void moveEast(long durationMilliseconds)
{
  moveEast();
  mDelay(durationMilliseconds);
  halt();
}

void moveWest()
{
  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);//Lock up perpendicular motor bank
  motorBank_EW_1.writeMicroseconds(1450); 
  motorBank_EW_2.writeMicroseconds(1542); //Rotate motors in East and West direction at desired speed 
}

void moveWest(long durationMilliseconds)
{
  moveWest();
  mDelay(durationMilliseconds);
  halt();
}

void halt()
{
  motorBank_NS_1.writeMicroseconds(1500); 
  motorBank_NS_2.writeMicroseconds(1500);
  motorBank_EW_1.writeMicroseconds(1500); 
  motorBank_EW_2.writeMicroseconds(1500);
}

void halt(long durationMilliseconds)
{
  halt();
  mDelay(durationMilliseconds);
}

void rotateCW()
{
  motorBank_EW_1.detach();
  motorBank_EW_2.detach();
  motorBank_NS_1.writeMicroseconds(1525); //slowly move correct side to straighten out robot
  motorBank_NS_2.writeMicroseconds(1525);
}

void rotateCounterCW()
{
  motorBank_EW_1.detach();
  motorBank_EW_2.detach();
  motorBank_NS_1.writeMicroseconds(1475); //slowly move correct side to straighten out robot
  motorBank_NS_2.writeMicroseconds(1475);
}

void pivotX1AwayFromWall()
{
  motorBank_EW_1.writeMicroseconds(1500); // lock EW1
  motorBank_NS_1.detach(); // dettach NS1 and NS2
  motorBank_NS_2.detach();
  motorBank_EW_2.writeMicroseconds(1525); // move EW2 west at slow speed
}

void pivotX1ToWall()
{
  motorBank_EW_1.writeMicroseconds(1500); // lock EW1
  motorBank_NS_1.detach(); // dettach NS1 and NS2
  motorBank_NS_2.detach();
  motorBank_EW_2.writeMicroseconds(1475); // move EW2 east at slow speed
}

void pivotX2AwayFromWall()
{
  motorBank_EW_2.writeMicroseconds(1500); // lock EW2
  motorBank_NS_1.detach(); // dettach NS1 and NS2
  motorBank_NS_2.detach();
  motorBank_EW_1.writeMicroseconds(1475); // move EW1 west at slow speed
}

void pivotX2ToWall()
{
  motorBank_EW_2.writeMicroseconds(1500); // lock EW2
  motorBank_NS_1.detach(); // dettach NS1 and NS2
  motorBank_NS_2.detach();
  motorBank_EW_1.writeMicroseconds(1525); // move EW1 east at slow speed
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ADVANCED MOVE FUNCTIONS *MA
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void moveNorthByTo(int xCoordinateAlignedBy, int yCoordinateSentTo)
{
  while(getY1() < yCoordinateSentTo)
  {
    straighten();
    if (abs(getX() - xCoordinateAlignedBy) >= 1)
      adjustX(xCoordinateAlignedBy);
    moveNorth(400);
  }
  straighten();
  halt();
}

void moveWestByTo(int yCoordinateAlignedBy, int xCoordinateSentTo) 
{
  long distanceY1;
  while(getX() < xCoordinateSentTo)
  {
    straighten();
    distanceY1 = getY1();
    if ((distanceY1 < yCoordinateAlignedBy - 3) || (distanceY1 > yCoordinateAlignedBy + 3))
    {
      adjustY(yCoordinateAlignedBy);
    }
    moveWest(400);
  }
  straighten();
  halt();
}

void moveEastByTo(int yCoordinateAlignedBy, int xCoordinateSentTo) 
{
  long distanceY1;
  while(getX() > xCoordinateSentTo)
  {
    straighten();
    distanceY1 = getY1();
    if ((distanceY1 < yCoordinateAlignedBy - 3) || (distanceY1 > yCoordinateAlignedBy + 1))
    {
      adjustY(yCoordinateAlignedBy);
    }
    moveEast(400);
  }
  straighten();
  halt();
}

void straighten()
{
  long X1 = getX1();
  long X2 = getX2();

  if (X1 < X2-1)
  {
    pivotX1AwayFromWall();
    while(X1 < X2)
    {
      mDelay(30);
      X1 = getX1();
      X2 = getX2();
    }
    reattach();
  }

  if (X1 > X2+1)
  {
    pivotX2AwayFromWall();
    while(X1 > X2)
    {
      mDelay(30);
      X2 = getX2();
      X1 = getX1();
    }
    reattach();
  }
}

void adjustY(int yDistance) 
{
  while(getY1() < yDistance) 
  {
    moveNorth(75);
    mDelay(300);
  }
  while (getY1() > yDistance)
  {
    moveSouth(75);
    mDelay(300);
  }
}

void adjustX(int xDistance)
{
  while(getX() < xDistance) 
  {
    moveWest(75);
    mDelay(300);
  }
  while (getX() > xDistance)
  {
    moveEast(75);
    mDelay(300);
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TURRET CONTROLS *TC
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void transition(Servo s, int angle)
{
  if(s.read() > angle)
    for(int i = s.read(); i >= angle; i--)
    {
      s.write(i);
      mDelay(40); 
    }
  else
    for(int i = s.read(); i <= angle; i++)
    {
      s.write(i);
      mDelay(40); 
    }
}

// Angles passed into this fucntion correspond to: 
//  =0 to aiming back and level
//  >0 to aiming right and up
//  <0 to aiming left and down
void aimTurret(int rotationAngle, int tiltAngle)
{
  //Angles from servo.read less than 90 correspond to rotating to the right and tilting up
  int trueRotationAngle = map(rotationAngle, -90, 90, 180, 0);
  int trueTiltAngle =  map(tiltAngle, -90, 90, 180, 0);

  transition(turret_Rotation , trueRotationAngle);
  mDelay(200);
  transition(turret_Tilt , trueTiltAngle);
}

void fire() 
{
  analogWrite(turret_Flywheels, 255); // The higher the PWM values, the more current the motors get. Have to calibrate these.
  mDelay(2500); //Allow sufficient time for the motors to get to full speed

  analogWrite(turret_Trigger, 255); //Send a short signal to fire the gun
  mDelay(350);
  analogWrite(turret_Trigger, 0); //Set all motors back to 0
  analogWrite(turret_Flywheels, 0);
}

// Returns an angle: 0 for dead ahead, >0 for aim to right, <0 for aim to left
int calculateTurretRotationAngle() 
{
  double xDistance = getX(3) + TURRET_CENTER_X;
  double yDistance = getY1(3)+ TURRET_CENTER_Y;
  Serial.print("(X,Y) Distance for Rotation: (");
  Serial.print(xDistance);
  Serial.print(",");
  Serial.print(yDistance);
  Serial.println(")");
  double angle = atan2((HOOP_X - xDistance) , yDistance) *180/PI;
  return (int)angle;
}

// Returns an angle: 0 for level, >0 for aim up, <0 for aim down
int calculateTurretTiltAngle()
{
  double yDistance = getY1(3)+TURRET_CENTER_Y;
  Serial.print("Y Distance for Tilt: ");
  Serial.println(yDistance);
  double angle = atan2((HOOP_Z - TURRET_HEIGHT) , yDistance) *180/PI;
  return (int)angle;
}

void fireFromLocation()
{
  int turretRotationAngle = calculateTurretRotationAngle();
  int turretTiltAngle = calculateTurretTiltAngle();

  debugDistance();
  Serial.print("Rotation Angle: ");
  Serial.println(turretRotationAngle);
  Serial.print("Tilt Angle: ");
  Serial.println(turretTiltAngle);
  Serial.println();

  aimTurret(turretRotationAngle,turretTiltAngle);
  mDelay(1000); // Allow turret to stabilize
  fire();
  mDelay(300); // Allow dart to fully leave before moving turret
  aimTurret(0,0);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//  DEBUG CODE *DB
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void debugUnit() 
{
  char inByte;
  // Input serial information:
  if (Serial.available() > 0)
  {
    inByte = Serial.read();

    if (inByte == 'N')
    {
      moveNorth(1000); 
      Serial.println("N");
    }
    else if (inByte == 'E')
    {
      moveEast(1000); 
      Serial.println("E");
    }
    else if (inByte == 'S')
    {
      moveSouth(1000);
      Serial.println("S"); 
    }
    else if (inByte == 'W')
    {
      moveWest(1000);
      Serial.println("W");
    }
  }
}

void debugDistance() 
{
  Serial.print("X1: "); 
  Serial.print(getX1());
  Serial.print(" X2: "); 
  Serial.print(getX2());
  Serial.print(" Y: "); 
  Serial.print(getY1());
  Serial.print(" B: ");
  Serial.print(getB1());
  Serial.println();
  delay(100);
}

void calibrateMovement()
{
  halt(5000);
  moveNorth(6000);
  halt(5000);
  moveWest(5000);
  halt(5000);
  moveSouth(5000);
  halt(5000);
  moveEast(5000);
}

void calibrateTurret()
{
  halt(10000);
  straighten();
  fireFromLocation();
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Old Move North Functions
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void moveNorthOnMainLine(int distanceMLToWall)
{
  moveNorth(400);
  long distanceX1 = getX1();

  if (distanceX1 != distanceMLToWall)
  {
    if (distanceX1 < distanceMLToWall)
    {
      pivotX1AwayFromWall();
      do
      {
        Serial.println(distanceX1);
        mDelay(50);
        distanceX1 = getX1();
      }
      while(distanceX1 != distanceMLToWall && distanceX1 < (distanceMLToWall + 1));
      Serial.print("final distance: ");
      Serial.println(distanceX1);
      reattach();
    }
    if (distanceX1 > distanceMLToWall)
    {
      pivotX1ToWall();
      do
      {
        Serial.println(distanceX1);
        mDelay(50);
        distanceX1 = getX1();
      }
      while(distanceX1 != distanceMLToWall && distanceX1 > (distanceMLToWall + 1));
      Serial.print("final distance: ");
      Serial.println(distanceX1);
      reattach();
    }
  }

  //ALIGN EW1
  long distanceX2 = getX2();

  if (distanceX2 != distanceMLToWall)
  {
    if (distanceX2 < distanceMLToWall)
    {
      pivotX2AwayFromWall();
      do
      {
        Serial.println(distanceX2);
        mDelay(50);
        distanceX2 = getX2();
      }
      while(distanceX2 != distanceMLToWall && distanceX2 < (distanceMLToWall + 1));
      Serial.print("final distance: ");
      Serial.println(distanceX2);
      reattach();
    }
    if (distanceX2 > distanceMLToWall)
    {
      pivotX2ToWall();
      do
      {
        Serial.println(distanceX2);
        mDelay(50);
        distanceX2 = getX2();
      }
      while(distanceX2 != distanceMLToWall && distanceX2 > (distanceMLToWall + 1));
      Serial.print("final distance: ");
      Serial.println(distanceX2);
      reattach();
    }
  }   
}

void moveNorthAlignedBy(int yCoordinate, int distanceMLToWall)
{
  while(getY1() < yCoordinate)
  {
    moveNorthOnMainLine(distanceMLToWall);
  }
  halt();
}
