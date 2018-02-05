// Smart RobotCar V2 demo (C)2018 Lee
// This demo needs to be used in conjunction with the mobile APP/IR controller.
// You can change the value of the SZ_SPEEDTHR to change the default motor speed
// You can control the servo to turn 0 - 180 degree.
// This demo support obstacle avoidance and trace

#include <UCMotor.h>
#include <Servo.h>
#include <Wire.h>
#include "UCNEC.h"

#define TRIG_PIN A2
#define ECHO_PIN A3

//Define the turn distance
#define TURN_DIST 30

//Define the tracking pin
#define leftSensor    A0
#define middleSensor  A1
#define rightSensor   13

//Define the turn time of the car
#define  turnTime    500
//You can change the TrackFactorUp and TrackFactorDown to adjust the value of tracking
//If the car deviate too far from the trail line,you can increase the value of TrackFactorUp and reduce the value of TrackFactorDown
//If the car is shaking very badly, even turn around,you'd better reduce the  value of the TrackFactorUp and increase the value of rackFactorDown
#define  TrackFactorUp     0.5
#define  TrackFactorDown   0.7

int MAX_SPEED_LEFT ;
int MAX_SPEED_RIGHT;
int SZ_SPEEDPRO = 0;
int SZ_SPEEDTHR = 150;

byte serialIn = 0;
byte commandAvailable = false;
String strReceived = "";


byte leftspeed = 0;
byte rightspeed = 0;

String motorSet = "";
int speedSet = 0;
int detecteVal = 0;
bool detected_flag = false;
bool timeFlag = true;

bool trackFlag = false;
bool avoidFlag = false;
bool trackStopFlag = false;
bool avoidStopFlag = false;
long currentTime = 0;

//The center Angle of two steering engines.
byte servoXCenterPoint = 88;

//The accuracy of servo
byte servoStep = 4;

//The current Angle of the two steering engines is used for retransmission
byte servoXPoint = 0;

UCNEC myIR(2);
UC_DCMotor leftMotor1(3, MOTOR34_64KHZ);
UC_DCMotor rightMotor1(4, MOTOR34_64KHZ);
UC_DCMotor leftMotor2(1, MOTOR34_64KHZ);
UC_DCMotor rightMotor2(2, MOTOR34_64KHZ);

Servo neckControllerServoMotor;

void setup()
{
  pinMode(leftSensor, INPUT_PULLUP);
  pinMode(middleSensor, INPUT_PULLUP);
  pinMode(rightSensor, INPUT_PULLUP);
  pinMode(ECHO_PIN, INPUT); //Set the connection pin output mode Echo pin
  pinMode(TRIG_PIN, OUTPUT);//Set the connection pin output mode trog pin
  Serial.begin(9600);
  neckControllerServoMotor.attach(10);
  neckControllerServoMotor.write(88);
  delay(2000);
  MAX_SPEED_LEFT = SZ_SPEEDTHR;
  MAX_SPEED_RIGHT = SZ_SPEEDTHR;
  neckControllerServoMotor.detach();
  delay(100);
  myIR.begin();
}

void loop()
{
  getSerialLine();
  if (commandAvailable) {
    processCommand(strReceived);
    strReceived = "";
    commandAvailable = false;
  }
}
void getSerialLine()
{
  static int32_t temp = 0;
  while (serialIn != '\r')
  {
    if (!(Serial.available() > 0))
    {
      if (timeFlag) {
        currentTime = micros();
        timeFlag = false;
      }
      if (  micros() - currentTime >= 100000) {
        timeFlag = true;
        currentTime = micros();
        if (detected_flag) {
          temp = readPing();
          if ( temp <= TURN_DIST && temp > 0 ) {
            moveStop();
          } else {
            ;
          }
        }
      }

      if (!avoidFlag) {
        while (myIR.available())
        {
          temp =  myIR.read();
        }
        if (temp == 0xFF46B9)  //up
        {
          temp = 0;
          Serial.println(F(" Move forward"));
          detected_flag = true;
          int temp = readPing();
          if ( temp <= TURN_DIST && temp > 0 ) {
            moveStop();
          }
          moveForward();
        } else if (temp == 0xFF15EA) { //down
          detected_flag = false;
          Serial.println(F(" Move backward"));
          temp = 0;
          moveBackward();
        } else if (temp == 0xFF44BB) { // left
          Serial.println(F(" Turn left"));
          temp = 0;
          detected_flag = true;
          turnLeft();
        } else if (temp == 0xFF43BC) { //right
          temp = 0;
          detected_flag = false;
          Serial.println(F(" Turn right"));
          turnRight();
        } else if (temp == 0xFF40BF) {
          Serial.println(F(" Stop"));
          temp = 0;
          detected_flag = false;
          neckControllerServoMotor.write(90);
          moveStop();
        }
      }
      return;
    }
    serialIn = Serial.read();
    if (serialIn != '\r') {
      if (serialIn != '\n') {
        char a = char(serialIn);
        strReceived += a;
      }
    }

  }
  if (serialIn == '\r') {
    commandAvailable = true;
    serialIn = 0;
  }
}

void processCommand(String input)
{
  String command = getValue(input, ' ', 0);
  static String SZ_speed, SZ_T, SZ_K;
  byte iscommand = true;
  int val;
  if (command == "MD_up")
  {
    detected_flag = true;
    int temp = readPing();
    if ( temp <= TURN_DIST && temp > 0 ) {
      moveStop();
    } else {
      moveForward();
    }
    if (trackFlag) {
      trackStopFlag = true;
    }
    if (avoidFlag) {
      avoidStopFlag = true;
    }

  } else if (command == "MD_up_left" || command == "MD_up_right" || command == "MD_down_left" || command == "MD_down_right")
  {
    movePianZhuan (command.substring(command.indexOf("_") + 1));
    detected_flag = false;
    if (trackFlag) {
      trackFlag = false;
      trackStopFlag = true;
    }
    if (avoidFlag) {
      avoidFlag = false;
      avoidStopFlag = true;
      neckControllerServoMotor.detach();
      delay(100);
      myIR.begin();
    }


  } else if (command == "MD_down")
  {
    moveBackward(); detected_flag = false;
    if (trackFlag) {
      trackFlag = false;
      trackStopFlag = true;
    }
    if (avoidFlag) {
      avoidFlag = false;
      avoidStopFlag = true;
      neckControllerServoMotor.detach();
      delay(100);
      myIR.begin();
    }


  } else if (command == "MD_left")
  {
    turnLeft(); detected_flag = false;
    if (trackFlag) {
      trackFlag = false;
      trackStopFlag = true;
    }
    if (avoidFlag) {
      avoidFlag = false;
      avoidStopFlag = true;
      neckControllerServoMotor.detach();
      delay(100);
      myIR.begin();
    }


  } else if (command == "MD_right")
  {
    turnRight(); detected_flag = false;
    if (trackFlag) {
      trackFlag = false;
      trackStopFlag = true;
    }
    if (avoidFlag) {
      avoidFlag = false;
      avoidStopFlag = true;
      neckControllerServoMotor.detach();
      delay(100);
      myIR.begin();
    }


  } else if (command == "MD_stop")
  {
    moveStop(); detected_flag = false;
    if (trackFlag) {
      trackFlag = false;
      trackStopFlag = true;
    }
    if (avoidFlag) {
      avoidFlag = false;
      avoidStopFlag = true;
      neckControllerServoMotor.detach();
      delay(100);
      myIR.begin();
    }

  } else if (command == "track")
  {
    trackFlag = true;
    trackStopFlag = false;
    moveTrack();
  }
  else if (command == "avoidance")
  {
    avoidFlag = true;
    avoidStopFlag = false;
    neckControllerServoMotor.attach(10);
    neckControllerServoMotor.write(90);
    avoidance();
  } else if (command == "stop")
  {
    moveStop(); detected_flag = false;
    trackStopFlag = true; avoidStopFlag = true; trackFlag = false; avoidFlag = false;

  } else if (command == "MD_SD")
  {
    val = getValue(input, ' ', 1).toInt();
    leftspeed = val;
    val = getValue(input, ' ', 2).toInt();
    rightspeed = val;
  } else if (command == "DJ_left")
  {
    servo_left();
  } else if (command == "DJ_right")
  {
    servo_right();
  } else if (command == "DJ_middle")
  {
    servo_center();
  }

  else if (!(command.indexOf("SZ") < 0))
  {
    if (!(command.indexOf("_K") < 0))
    {
      SZ_K = command.substring(command.indexOf("K") + 1, command.indexOf("T"));
      SZ_SPEEDPRO = SZ_K.toInt ();
      SZ_T = command.substring(command.indexOf("T") + 1);
      SZ_SPEEDTHR = SZ_T.toInt ();
      if (SZ_SPEEDPRO < 0)
      {
        if (SZ_SPEEDTHR + abs (SZ_SPEEDPRO) > 255)
        {
          MAX_SPEED_RIGHT = 255;
        }
        else
        {
          MAX_SPEED_RIGHT = SZ_SPEEDTHR + abs (SZ_SPEEDPRO);
        }
        MAX_SPEED_LEFT = SZ_SPEEDTHR;

      } else if (SZ_SPEEDPRO > 0 || SZ_SPEEDPRO == 0)
      {
        if (SZ_SPEEDTHR + SZ_SPEEDPRO > 255)
        {
          MAX_SPEED_LEFT = 255;
        }
        else
        {
          MAX_SPEED_LEFT = SZ_SPEEDTHR + SZ_SPEEDPRO;
        }
        MAX_SPEED_RIGHT = SZ_SPEEDTHR;

      }
      leftMotor1.setSpeed(MAX_SPEED_LEFT);
      rightMotor1.setSpeed(MAX_SPEED_RIGHT);
      leftMotor2.setSpeed(MAX_SPEED_LEFT);
      rightMotor2.setSpeed(MAX_SPEED_RIGHT);
    }
  }
  else
  {
    iscommand = false;
  }
  if (iscommand) {
    SendMessage("cmd:" + input);
  }
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void moveForward(void)
{
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);
  rightMotor1.run(FORWARD);
  motorSet = "FORWARD2";
  leftMotor2.run(FORWARD2);
  rightMotor2.run(FORWARD2);
  leftMotor1.setSpeed(MAX_SPEED_LEFT);
  rightMotor1.setSpeed(MAX_SPEED_RIGHT);
  leftMotor2.setSpeed(MAX_SPEED_LEFT);
  rightMotor2.setSpeed(MAX_SPEED_RIGHT);
}
void movePianZhuan (String mode)
{
  static int MAX_SPEED_LEFT_A, MAX_SPEED_RIGHT_A, MODE;
  if (!(mode.indexOf("up") < 0))
  {
    if (mode.indexOf("left") > 0)
    {
      MAX_SPEED_RIGHT_A = MAX_SPEED_RIGHT > (255 - (int)(MAX_SPEED_RIGHT * 0.5)) ? 255 : MAX_SPEED_RIGHT + (int)(MAX_SPEED_RIGHT * 0.5);
      MAX_SPEED_LEFT_A  = (MAX_SPEED_LEFT - (int)(MAX_SPEED_LEFT * 0.5)) < 0 ? 0 : MAX_SPEED_LEFT - (int)(MAX_SPEED_LEFT * 0.5);
    }
    else if (mode.indexOf("right") > 0)
    {
      MAX_SPEED_LEFT_A = MAX_SPEED_LEFT > (255 - (int)(MAX_SPEED_LEFT * 0.5)) ? 255 : MAX_SPEED_LEFT + (int)(MAX_SPEED_LEFT * 0.5);
      MAX_SPEED_RIGHT_A  = (MAX_SPEED_RIGHT - (int)(MAX_SPEED_RIGHT * 0.5)) < 0 ? 0 : MAX_SPEED_RIGHT - (int)(MAX_SPEED_RIGHT * 0.5);
    }
    leftMotor1.run(FORWARD);
    rightMotor1.run(FORWARD);
    leftMotor2.run(FORWARD2);
    rightMotor2.run(FORWARD2);
  }
  if (!(mode.indexOf("down") < 0))
  {
    if (mode.indexOf("left") > 0)
    {
      MAX_SPEED_RIGHT_A = MAX_SPEED_RIGHT > (255 - (int)(MAX_SPEED_RIGHT * 0.5)) ? 255 : MAX_SPEED_RIGHT + (int)(MAX_SPEED_RIGHT * 0.5);
      MAX_SPEED_LEFT_A  = (MAX_SPEED_LEFT - (int)(MAX_SPEED_LEFT * 0.5)) < 0 ? 0 : MAX_SPEED_LEFT - (int)(MAX_SPEED_LEFT * 0.5);
    }
    else if (mode.indexOf("right") > 0)
    {
      MAX_SPEED_LEFT_A = MAX_SPEED_LEFT > (255 - (int)(MAX_SPEED_LEFT * 0.5)) ? 255 : MAX_SPEED_LEFT + (int)(MAX_SPEED_LEFT * 0.5);
      MAX_SPEED_RIGHT_A  = (MAX_SPEED_RIGHT - (int)(MAX_SPEED_RIGHT * 0.5)) < 0 ? 0 : MAX_SPEED_RIGHT - (int)(MAX_SPEED_RIGHT * 0.5);
    }
    leftMotor1.run(BACKWARD);
    rightMotor1.run(BACKWARD);
    leftMotor2.run(BACKWARD2);
    rightMotor2.run(BACKWARD2);
  }
  leftMotor1.setSpeed(MAX_SPEED_LEFT_A);
  rightMotor1.setSpeed(MAX_SPEED_RIGHT_A);
  leftMotor2.setSpeed(MAX_SPEED_LEFT_A);
  rightMotor2.setSpeed(MAX_SPEED_RIGHT_A);
}
void moveBackward(void)
{
  motorSet = "BACKWARD";
  leftMotor1.run(BACKWARD);
  rightMotor1.run(BACKWARD);
  leftMotor2.run(BACKWARD2);
  rightMotor2.run(BACKWARD2);
  leftMotor1.setSpeed(MAX_SPEED_LEFT);
  rightMotor1.setSpeed(MAX_SPEED_RIGHT);
  leftMotor2.setSpeed(MAX_SPEED_LEFT);
  rightMotor2.setSpeed(MAX_SPEED_RIGHT);
}
void turnRight(void)
{
  static int MAX_SPEED_LEFT_AR, MAX_SPEED_RIGHT_AR;
  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);
  rightMotor1.run(BACKWARD);
  leftMotor2.run(FORWARD2);
  rightMotor2.run(BACKWARD2);
  MAX_SPEED_LEFT_AR = MAX_SPEED_LEFT > 200 ? 200 : MAX_SPEED_LEFT;
  MAX_SPEED_RIGHT_AR = MAX_SPEED_RIGHT > 200 ? 200 : MAX_SPEED_RIGHT;
  leftMotor1.setSpeed(MAX_SPEED_LEFT_AR);
  rightMotor1.setSpeed(MAX_SPEED_RIGHT_AR);
  leftMotor2.setSpeed(MAX_SPEED_LEFT_AR);
  rightMotor2.setSpeed(MAX_SPEED_RIGHT_AR);
}
void turnLeft(void)
{
  static int MAX_SPEED_LEFT_AL, MAX_SPEED_RIGHT_AL;
  motorSet = "LEFT";
  leftMotor1.run(BACKWARD);
  rightMotor1.run(FORWARD);
  leftMotor2.run(BACKWARD2);
  rightMotor2.run(FORWARD2);
  MAX_SPEED_LEFT_AL = MAX_SPEED_LEFT > 200 ? 200 : MAX_SPEED_LEFT;
  MAX_SPEED_RIGHT_AL = MAX_SPEED_RIGHT > 200 ? 200 : MAX_SPEED_RIGHT;
  leftMotor1.setSpeed(MAX_SPEED_LEFT_AL);
  rightMotor1.setSpeed(MAX_SPEED_RIGHT_AL);
  leftMotor2.setSpeed(MAX_SPEED_LEFT_AL);
  rightMotor2.setSpeed(MAX_SPEED_RIGHT_AL);
}


void TrackturnLeft(void)
{
  static int MAX_SPEED_LEFT_A, MAX_SPEED_RIGHT_A;
  MAX_SPEED_RIGHT_A = MAX_SPEED_RIGHT > (255 - (int)(MAX_SPEED_RIGHT * 0.5)) ? 255 : MAX_SPEED_RIGHT + (int)(MAX_SPEED_RIGHT * 0.5);
  MAX_SPEED_LEFT_A  = (MAX_SPEED_LEFT - (int)(MAX_SPEED_LEFT * 0.7)) < 0 ? 0 : MAX_SPEED_LEFT - (int)(MAX_SPEED_LEFT * 0.7);

  motorSet = "LEFT";
  leftMotor1.run(BACKWARD);
  rightMotor1.run(FORWARD);
  leftMotor2.run(BACKWARD2);
  rightMotor2.run(FORWARD2);
  leftMotor1.setSpeed(MAX_SPEED_LEFT_A);
  rightMotor1.setSpeed(MAX_SPEED_RIGHT_A);
  leftMotor2.setSpeed(MAX_SPEED_LEFT_A);
  rightMotor2.setSpeed(MAX_SPEED_RIGHT_A);
}

void TrackturnRight(void)
{
  static int MAX_SPEED_LEFT_A, MAX_SPEED_RIGHT_A;
  MAX_SPEED_LEFT_A = MAX_SPEED_LEFT > (255 - (int)(MAX_SPEED_LEFT * TrackFactorUp)) ? 255 : MAX_SPEED_LEFT + (int)(MAX_SPEED_LEFT * TrackFactorUp);
  MAX_SPEED_RIGHT_A  = (MAX_SPEED_RIGHT - (int)(MAX_SPEED_RIGHT * TrackFactorDown)) < 0 ? 0 : MAX_SPEED_RIGHT - (int)(MAX_SPEED_RIGHT * TrackFactorDown);

  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);
  rightMotor1.run(BACKWARD);
  leftMotor2.run(FORWARD2);
  rightMotor2.run(BACKWARD2);
  leftMotor1.setSpeed(MAX_SPEED_LEFT_A);
  rightMotor1.setSpeed(MAX_SPEED_RIGHT_A);
  leftMotor2.setSpeed(MAX_SPEED_LEFT_A);
  rightMotor2.setSpeed(MAX_SPEED_RIGHT_A);
}
void moveStop(void)
{
  leftMotor1.run(RELEASE); rightMotor1.run(RELEASE);
  leftMotor2.run(RELEASE); rightMotor2.run(RELEASE);
}
void moveTrack(void)
{
  int temp = 0, num1 = 0, num2 = 0, num3 = 0;
  strReceived = "";
  commandAvailable = false;
  while (1) {
    if ((Serial.available() > 0))
    {
      serialIn = Serial.read();
      if (serialIn != '\r') {
        if (serialIn != '\n') {
          char a = char(serialIn);
          strReceived += a;
        }
      }
    }
    if (serialIn == '\r') {
      commandAvailable = true;
      serialIn = 0;
      processCommand(strReceived);
      strReceived = "";
      commandAvailable = false;
    }

    if (trackStopFlag) {
      trackStopFlag = false;
      moveStop();
      strReceived = "";
      commandAvailable = false;
      break;
    }
    num1 = digitalRead(leftSensor);
    num2 = digitalRead(middleSensor);
    num3 = digitalRead(rightSensor);
    if ((num2 == 0) && (num1 == 0) && (num3 == 0)) {
      moveStop(); continue;
    } else if ( (num1 == 0) && num3 == 1) { //go to right
      TrackturnLeft();
      while (1) {
        if ((Serial.available() > 0))
        {
          serialIn = Serial.read();
          if (serialIn != '\r') {
            if (serialIn != '\n') {
              char a = char(serialIn);
              strReceived += a;
            }
          }
        }
        if (serialIn == '\r') {
          commandAvailable = true;
          serialIn = 0;
          processCommand(strReceived);
          strReceived = "";
          commandAvailable = false;
        }

        if (trackStopFlag) {
          //trackStopFlag = false;
          moveStop();
          strReceived = "";
          commandAvailable = false;
          break;
        }
        num2 = digitalRead(middleSensor);
        if (num2) {
          TrackturnLeft(); continue;
        } else
          break;
      }
    } else if ((num3 == 0) && (num1 == 1)) { // go to left
      TrackturnRight();
      while (1) {

        if ((Serial.available() > 0))
        {
          serialIn = Serial.read();
          if (serialIn != '\r') {
            if (serialIn != '\n') {
              char a = char(serialIn);
              strReceived += a;
            }
          }
        }
        if (serialIn == '\r') {
          commandAvailable = true;
          serialIn = 0;
          processCommand(strReceived);
          strReceived = "";
          commandAvailable = false;
        }

        if (trackStopFlag) {
          // trackStopFlag = false;
          moveStop();
          strReceived = "";
          commandAvailable = false;
          break;
        }
        num2 = digitalRead(middleSensor);
        if (num2) {
          TrackturnRight(); continue;
        } else
          break;
      }
    } else
    {
      moveForward();
    }

  }
}

void avoidance(void)
{
  unsigned int S;
  strReceived = "";
  commandAvailable = false;
  while (1) {
    if ((Serial.available() > 0))
    {
      serialIn = Serial.read();
      if (serialIn != '\r') {
        if (serialIn != '\n') {
          char a = char(serialIn);
          strReceived += a;
        }
      }
    }
    if (serialIn == '\r') {
      commandAvailable = true;
      serialIn = 0;
      processCommand(strReceived);
      strReceived = "";
      commandAvailable = false;
    }
    if (avoidStopFlag) {
      avoidStopFlag = false;
      moveStop();
      strReceived = "";
      commandAvailable = false;

      delay(100);
      neckControllerServoMotor.detach();
      delay(100);
      myIR.begin();

      break;
    }
    neckControllerServoMotor.write(90);
    S = readPing();
    if (S <= TURN_DIST ) {
      turn();delay(500);
    } else if (S > TURN_DIST) {
      moveForward();
    }
  }
}

void turn() {
  int S,Sleft,Sright;
  moveStop();
  neckControllerServoMotor.write(150);
  delay(300);
  S = readPing();
  Sleft = S;
  neckControllerServoMotor.write(90);
  delay(300);
  neckControllerServoMotor.write(30);
  delay(300);
  S = readPing();
  Sright = S;
  neckControllerServoMotor.write(90);
  delay(300);
  if (Sleft <= TURN_DIST && Sright <= TURN_DIST) {
    moveBackward();
    delay(500);
    int x = random(1);
    if (x = 0) {
      turnRight();
    }
    else {
      turnLeft();
    }
  } else {
    if (Sleft >= Sright) {
      turnLeft();
    } else {
      turnRight();
    }
  }
}

void SendMessage(String data) {
  Serial.println(data);
}
int readPing()
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  return cm ;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void servo_right(void)
{
  if (!avoidFlag) {
    neckControllerServoMotor.attach(10); delay(10);
    int servotemp = neckControllerServoMotor.read();
    servotemp -= servoStep;
    servo_Horizontal(servotemp);
    delay(50);
    neckControllerServoMotor.detach();
    delay(50);
    myIR.begin();
  }

}
void servo_left(void)
{
  if (!avoidFlag) {
    neckControllerServoMotor.attach(10); delay(10);
    int servotemp = neckControllerServoMotor.read();
    servotemp += servoStep;
    servo_Horizontal(servotemp);
    delay(50);
    neckControllerServoMotor.detach();
    delay(50);
    myIR.begin();
  }
}
void servo_center(void)
{
  if (!avoidFlag) {
    neckControllerServoMotor.attach(10); delay(10);
    servo_Horizontal(servoXCenterPoint);
    delay(50);
    neckControllerServoMotor.detach();
    delay(50);
    myIR.begin();
  }
}

void servo_Horizontal(int corner)
{
  int i = 0;
  byte cornerX = neckControllerServoMotor.read();
  if (cornerX > corner) {
    for (i = cornerX; i > corner; i = i - servoStep) {
      \
      neckControllerServoMotor.write(i);
      servoXPoint = i;
      delay(50);
    }
  }
  else {
    for (i = cornerX; i < corner; i = i + servoStep) {
      neckControllerServoMotor.write(i);
      servoXPoint = i;
      delay(50);
    }
  }
  neckControllerServoMotor.write(corner);
  servoXPoint = corner;
}

