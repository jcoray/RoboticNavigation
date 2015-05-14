/*
 Code for running a robot with an arm and a claw.
 Created by Jakob Coray, 8/30/2014. Last edited 02/02/2014.
 
 Version 2.0.4
 
 Description: (last updated 01/05/14)
 
 Design: 
 The robot is about 13 inches long by 12 3/4 inches wide by 11 inches high 
 with out the arm (TODO has changed). The arm sits at the back of the robot and is 18 2 inches 
 long with the claw closed. The robot is about 20 1/2 inches long with the 
 arm are parallel to the ground, and 27 inches high when it is all the way up. 
 It weighs 5 pounds, 7 1/8 ounces (2.468 kg) (12/29/14).
 The chassis, wheels, drive motors, and other components came from a VEX Robotics
 kit. The arm and claw can lift ? pounds without stressing the system. The wheels
 have a 4 inches diameter and have a 1:1 gearing with the two drive motors. There are two 
 sets of gear reductions on the arm with a ?:? ratio, and 1 set on the claw with a 
 ?:? ratio. 
 The brain of the robot is a Arduino Mega 2560 R3.
 
 Power: 
 The robot is powered by a 7.4 V, 1400 mAh Lithium-ion battery
 adapted from a Lego Mindstorms. It has built in protection and charging 
 circuitry. It charges by a ? V, ? A wall adapter with a 2.1 mm adapter.
 If the battery is shorted, it will shut off, and can only be turned back on by
 plugging it in again. As of 9/16/14 I am not sure if it automatically shuts
 off when the voltage output drops beyond a certain point, so I would
 advise frequently charging it and not fully discharging it. The drive, arm, 
 and claw motors run directly off of the battery, and the Arduino is connected
 in parallel. The robot pulls no more than 4 amps (around 3.5 when I last tested 
 it with all of the motors stalled). All other electronic components run off of 
 5V power from the an external LM7805 linear voltage regulator (can output up to 1.5 amps,
 but there is not much heat sinking on it, so I would not go above 1 amp), 
 except the wireless transceiver, which runs off of 3.3V 
 from the Arduino. A yellow LED indicates that the
 regulated 5V power is on, and a red LED indicates that the battery is on. There
 is a master power switch on the power board.
 
 Wiring: (last updated 12/29/14)
 Hardware SPI: 
 MISO -> 50 (green)
 MOSI -> 51 (red)
 SCK  -> 52 (orange)
 Configurable:
 CE   -> 48 (orange)
 CSN  -> 47 (white)
 
 For pin out see global variables
 file directory or web address for CAD file/pictures (TODO have to make those)
 
 Change log:
 manual (05/14/15) switched to git for version control
 v2.0.4 (01/30/15) rewrote arm limits from the ground up; removed calibrateArm, and arm read, all the other limit funcctions as they are not used or useful, and other general cleanups
 v2.0.3 (01/25/15) added red/blueshift to the front LED, worked on arm limits
 v2.0.2 (01/05/15) put program through spell check, fixed comments, removed function void pan(), it compiles, but does is work (untested!)
 v2.0.1 (01/01/15) worked on OLED setup screen
 v2.0   (12/29/14) Began work after rewiring entire system. Set up hardware for autonomous control.
 v1.0.1 (12/01/14) converted the serial print out for the control data into a function (untested!) (EDIT: works!)
 v1.0:  (09/21/14) got the wireless to work for driving and moving the claw.
 v0.1:  (09/17/14) began work on calibrateArm, added pinMode for sensors
 v0.0:  (09/16/14) converted old robotBaseCode.ino to baseCode.ino. Got rid of clawBot class.
 began writing description. Formated code. Made minor edits to code.
 */

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#include <Servo.h> 

#include <Adafruit_ssd1306syp.h> //  OLED library (notice the syp at the end, this is *not* the standard Adafruit_ssd1306.h library, it is not compatible and will not work)

const int armMotorPin = 23; 
const int leftMotorPin = 25;  
const int rightMotorPin = 27;
const int clawMotorPin = 29; 

//  OLED software i2c pins
const int OLEDSDAPin = 9; 
const int OLEDSCLPin = 8; 
//  OLED init
Adafruit_ssd1306syp display(OLEDSDAPin, OLEDSCLPin);

//  Digital Pins
const int upLimitPin = 22; //connected to upper bumper sensor, pulls low when arm is max hight, used with internal pull up
const int downLimitPin = 24; //connected to lower bumper sensor, pulls low when arm is minimum hight, used with internal pull up
const int greenPin = 4; //PWM pin; used with RGB LED on arm
const int redPin = 5; //PWM pin; used with RGB LED on arm
const int bluePin = 6; //PWM pin; used with RGB LED on arm
const int handLimitPin = 7;//TODO write code; used to mechanically sense if there is an object in the hand; NOTE has a tendency to rotate
const int upPin = 8;  //when there is a limiting event at the top of the arm's reach this LED turns on
const int downPin = 9;  //when there is a limiting event at the bottom of the arm's reach this LED turns on
const int tiltSensorPin = 10; //senses when the robot flips over; is sensitive to shaking
const int echoPin = 11; //tentative, may change com pin for ultrasonic
const int trigPin = 12; //tentative, may change, com pin for ultrasonic
const int ultraServoPin = 13;

//  Analog Pins
const int IRPin = 15; //TODO senses objects' distance from hand, offset by a few inches, may need repositioning, also is broken..


//TODO declare, OLEDTopButtonPin, OLEDBottomButtonPin, all of the GPS pins, speakerPin, tiltSensorPin, lineSensorPin,
//wiringLEDPin, and accelerometer pins (First get one)

Servo leftMotor;  //2000 forwards, 1000 reverse
Servo rightMotor; //1000 forwards, 2000 reverse
Servo armMotor;  //2000 up, 1000 down
Servo clawMotor; //2000 close, 1000 open
Servo ultraServo;  //0 point left, 180 point right

//------------------------------------------------------------------------------

void setup()  {
  /*
  The setup for this program works by first setting up and stabilizing
  the motors, as this is the most central and important part of the 
  system. Everything should be initialized in order of importance. If 
  something is preventing normal operation the user should be notified
  and given the option to override. The main user interface is the 128
  by 64 pixel OLED screen the first 16 rows of pixels (indexed 0-15) are
  yellow, and are used to display system information and as a menu
  header, while the remaining lines are blue are are used to describe
  menu specifics and button detail. There are two buttons used with the
  OLED, they are the primary way of getting input from the user. After 
  the motors are stabilized, the remainder of the peripherals are initialed,
  along with serial communication. Then control over initiation is 
  given to the user, and if the user does not respond with input within
  15 seconds, the default configuration is activated: wireless with lights on 
  auto.
  */
  
  leftMotor.attach(leftMotorPin);
  rightMotor.attach(rightMotorPin);
  armMotor.attach(armMotorPin);
  clawMotor.attach(clawMotorPin);
  ultraServo.attach(ultraServoPin);

  //stabilize motors in stop position (prevents sporadic motor movement)
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  armMotor.writeMicroseconds(1500);  
  clawMotor.writeMicroseconds(1500);
  ultraServo.writeMicroseconds(1500);

  Serial.begin(19200); //high baud rate was picked so lots of data could be moved quickly

  pinMode(upLimitPin, INPUT_PULLUP); // large SPST normally open bumper
  pinMode(downLimitPin, INPUT_PULLUP); //large SPST normally open bumper
  pinMode(upPin, OUTPUT);
  pinMode(downPin, OUTPUT);
  pinMode(echoPin, INPUT); //used to tell ultrasonic sensor when to collect and send data
  pinMode(trigPin, OUTPUT);  //ultrasonic sensor uses this pin to send back data
  pinMode(handLimitPin, INPUT_PULLUP);
  pinMode(tiltSensorPin, INPUT);
  // Wireless Setup
  //set up the configurable pins for the transceiver, uses Mirf library
  Mirf.cePin = 48;
  Mirf.csnPin = 49;
  
  Mirf.spi = &MirfHardwareSpi; // SPI config for the transceiver 
  Mirf.init(); // Initialize the transceiver hardware
  
  Mirf.setRADDR((byte *)"clie1"); //sets up the transceiver to receiver a byte array and names the device clie1 (client 1)
  
  Mirf.payload = 7; //there are 7 bytes of data in the payload array
  Mirf.config(); // Configure the transceiver software 
  
  // Display Setup
  display.initialize();
  writeScreen("", 0,"Joystick Control");
}

void loop()  {
  static unsigned long timeSinceReceived = 0;
  byte data[Mirf.payload];  // Buffer array to hold the received data
  if(Mirf.dataReady())  {  
    timeSinceReceived = millis();
    // If a packet has been received from the nun-chuck
//   Serial.println("Got packet");
    Mirf.getData(data);  // Get load the packet into the buffer.
    
    writeWheels(data[0],data[1]); // use the joystick data to drive the wheels
    writeClaw(data[5],data[6]);  // use the button data to move the claw (z close, c open)
    writeArm(data[3]); // use accelerometer on y axis 
    
    dopplerShift(data[1]); 
    autoRecover();
//    printDebug(data);
  }
  else if(millis()-timeSinceReceived > 300) {
    //detect if there is not a signal, then stop everything.
    drive(0);
    writeClaw(1,1); 
    writeArm(125);
    dopplerShift(127);
    writeScreen("ERROR: No signal", 0, "Out of range or joystick off");
    while(!Mirf.dataReady()) {
      delay(10); //  why not delay
    }
    writeScreen("", 0,"Joystick Control");
  }
}

void autoRecover()  {
  //TODO add explanatory comments 
  static long flipped;
  flipped = startTilt();
  if(!flipped) return;
  else if(flipped)  {
    drive(0); // stop
    writeScreen("ERROR: flipped over", 1, "Auto-recovering...");
    unsigned long timeStart = millis();
    while(digitalRead(handLimitPin)==0) { 
      if(debounceTilt()==1) break;
      // if they is anything in the claw, drop it; 
      // if it refuses to come out after 2 seconds, quit
      clawMotor.writeMicroseconds(1000); // open
      if(millis()-timeStart>1000) break;
    }
    clawMotor.writeMicroseconds(1500);
    debounceTilt();
    while(digitalRead(upLimitPin)==1)  {
      if(debounceTilt()==1) break;
      // move the arm all the way down to shift the center of gravity
      armMotor.writeMicroseconds(1000); // down  
      }
    armMotor.writeMicroseconds(1500);
    timeStart = millis(); // set start time
    debounceTilt();
    while(digitalRead(tiltSensorPin)==0)  {
      if(debounceTilt()==1) break;
      drive(2); // backwards
      // if the robot does not flip back over after 4 seconds, give up
      if(millis()-timeStart>4000) break;  
    }
    //  if auto-recovery fails, wait for the user to flip the robot over
    drive(0); //s top
    writeScreen("ERROR: flipped over", 1, "Flip the robot over.");
    while(1)
    {
      if(debounceTilt()==1) break;
      writeRGB(255,0,0);
      delay(750);
      if(debounceTilt()==1) break;
      writeRGB(0,0,0);
      delay(750);
    }
    //  move the arm back up a little bit
    armMotor.writeMicroseconds(2000);
    delay(500);
    armMotor.writeMicroseconds(1500);
    writeScreen("", 0,"Joystick Control");
  }
}

int debounceTilt()  {
  //0 = 
  const int debounceTime = 250;
  static int calles;
  static unsigned long stateTime;
  if(digitalRead(tiltSensorPin)==0)  {
    stateTime = millis();
    calles = 0;
    return 0;
  }
  else if(digitalRead(tiltSensorPin)==1 && millis()-stateTime>debounceTime)  {
   calles++;
   if(calles > 5)  return 1;
   else return 0;
  }
}

int startTilt()  {
  const int debounceTime = 250;
  static int calles;
  static unsigned long stateTime;
  if(digitalRead(tiltSensorPin)==1)  {
    stateTime = millis();
    calles = 0;
    return 0;
  }
  else if(digitalRead(tiltSensorPin)==0 && millis()-stateTime>debounceTime)  {
    calles++;
    if(calles > 20)  return 1;
    else return 0;
  }
}

void writeScreen(char message[19], int robotPosition, char text[47])  {
  // do not make message or  more than 18 characters long
  enum RobotPosition  {
    UPRIGHT,
    FLIPED,
  };
  display.clear();
  display.setCursor(0,0);
  display.setTextColor(WHITE); 
  display.setTextSize(1);
  display.println("Robot 2.0.4 by Jakob"); 
  display.println(message);
  switch(robotPosition)  {
    case(UPRIGHT):
      display.println("        >===o");
      display.println("        ____]");
      display.println("        *   *");
      break;
    case(FLIPED):
      display.println("        *)   {}"); 
      display.println("         |   / ");
      display.println("        *)==/  ");
  }
  display.drawBitmap();
  display.println();
  display.println(text);
  display.update();
}
    
void writeRGB(byte red, byte green, byte blue)  {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}
  
void dopplerShift(int joyY)  { 
  //  When the robot moves forward the light blue shifts, backwards the light redshifts
  int color = map(constrain(joyY, 34, 217),34, 217 , 0, 255);
  if(color > 127) writeRGB(255 - color ,255 - color, color);
  else writeRGB(255-color, color, color);
}

void printDebug(byte controlData[7])  {
  // print the received data, order is: xJoy, yJoy, xAccel, yAccel, zAccel, Z, C
  Serial.println();
  for(int i= 0; i < 7; i++)  {
    Serial.print(controlData[i]);
    Serial.print(",");
  }
}

void writeWheels(byte xRaw, byte yRaw)  {
  // Takes the x,y position of the joystick and converts it into microsecond values to send to the drive servos
  int yScaled, xScaled;
  
  if (yRaw > 133 || yRaw < 123) yScaled = map(constrain(yRaw,29,222), 222, 29, 1000, 2000); //If the joystick value is out of the center range ~127 +-5
  else yScaled = 1500; //joystick on center (y axis)

  if (xRaw > 133 || xRaw < 123) xScaled = map(constrain(xRaw,30,222), 222, 30, 500, -500); //If the joystick value is out of the center range ~127 +-5, 
  else xScaled = 0;  //joystick on center (x axis)
  
  int rightMicroseconds = yScaled + xScaled;  // converts scaled values into to  drive values
  int leftMicroseconds = 3000 - yScaled + xScaled; // the left servo is flipped 

  rightMotor.writeMicroseconds(rightMicroseconds);
  leftMotor.writeMicroseconds(leftMicroseconds);
//  Serial.println(yScaled);
//  Serial.println(xScaled);
//  Serial.println(rightMicroseconds);
//  Serial.println(leftMicroseconds);
}

void drive(int movement)  {
  // 0 -> stop; 1 -> forward; 2 -> backward; 3 -> left; 4 -> right
  switch(movement)  {
  case 0:    // stop
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
    break;

  case 1:    // forwards
    leftMotor.writeMicroseconds(2000);
    rightMotor.writeMicroseconds(1000);
    break;

  case 2:    // backwards
    leftMotor.writeMicroseconds(1000);
    rightMotor.writeMicroseconds(2000);
    break;

  case 3:    // left
    leftMotor.writeMicroseconds(1000);
    rightMotor.writeMicroseconds(1000);
    break;

  case 4:    // right
    leftMotor.writeMicroseconds(2000);
    rightMotor.writeMicroseconds(2000);
    break;
  }
}

void writeArm(byte y)  {
  // IT WORKS!!
  // The goal is for prevent the user from extending the arm past its limits
  // and hurting the robot. 
  // If the user is, for example, raising the joystick and thereby the arm,
  // and he/she extends the arm up too far, it should hit the bumper, stop, turn
  // on the up LED, and *not bounce.* It should
  // then hold the arm steady until the user points the nun-chuck down, and the up
  // LED should turn off. The same should apply to the lower limit.
  enum Directions  {
    DOWN = -1,
    UP = 1,
  };

  int nunchuckPointing;
  if (y < 100) nunchuckPointing = UP; //user points nun-chuck up
  else if (y > 160) nunchuckPointing = DOWN; //user points nun-chuck down
  
  int limitPressed;  //which, if any limit pins are currently being activated
  if(digitalRead(upLimitPin)==0) limitPressed = UP; //if the arm hits the upper limit
  else if(digitalRead(downLimitPin)==0) limitPressed = DOWN; //if the arm hits the lower limit
  else limitPressed = false;
  
  static int limitingEvent;
  if(limitPressed==UP) limitingEvent=UP; //start a limiting event, more or less emulate a loop
  else if(limitPressed==DOWN) limitingEvent=DOWN; 
    
  if(limitingEvent)  {
    static int limitingAction; 
    const int moveArm=1; 
    const int waitForUser=2;
    if(limitPressed) limitingAction=moveArm;
    
    if(limitingEvent==UP)  {
      if(limitingAction==moveArm)  {
        armMotor.writeMicroseconds(2000); //move the arm down
        digitalWrite(upPin,HIGH);
        limitingAction=waitForUser;
      }
      else if(limitingAction==waitForUser && nunchuckPointing==UP)  {
          armMotor.writeMicroseconds(2000); //move the arm down
          limitingAction=false; 
          limitingEvent=false;
          digitalWrite(upPin,LOW);
      }
        else armMotor.writeMicroseconds(1500); //center the arm
    }
      
    else if(limitingEvent==DOWN)  {
      if(limitingAction==moveArm)  {
        armMotor.writeMicroseconds(1000); //move the arm up
        digitalWrite(downPin,HIGH);
        limitingAction=waitForUser;
      }
      else if(limitingAction==waitForUser && nunchuckPointing==DOWN)  {
          armMotor.writeMicroseconds(1000); //move the arm up
          limitingAction=false; 
          limitingEvent=false;
          digitalWrite(downPin,LOW);
      }
        else armMotor.writeMicroseconds(1500); //center the arm
    }
  }
  else if(!limitingEvent)  {
    if(nunchuckPointing==UP) armMotor.writeMicroseconds(2000);  //move the arm up
    else if(nunchuckPointing==DOWN) armMotor.writeMicroseconds(1000); //move the arm down
    else armMotor.writeMicroseconds(1500); //center the arm
  }
}

void writeClaw(byte z,byte c)  {
  //  controls the claw from the remote
  if(z == 0) clawMotor.writeMicroseconds(2000); //opens the claw
  else if (c == 0) clawMotor.writeMicroseconds(1000); //closes the claw
  else clawMotor.writeMicroseconds(1500);
}

int readIR()  {
  //  This sensor broke...
  unsigned int value = analogRead (IRPin);
  if (value < 10) value = 10;
  unsigned int cm = ((67870.0 / (value - 3.0)) - 40.0)/10; //signed or unsigned?
  return cm;
}

int readUltra(int angle)  {
  //  TODO see if delayMicroseconds() will interfere with operation
  //  TODO figure out how long it takes the servo motor to move
  static int a = angle;//angle
  ultraServo.write(angle);

  long duration, cm;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH);
  //Serial.println("Pulse in duration from ultra, see how long this is so I can figure out what type of number it should be."); // test; remove
  //Serial.print(duration);
  cm = duration / 29 / 2;
  cm = int(cm);
  return cm;
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance traveled.
}
