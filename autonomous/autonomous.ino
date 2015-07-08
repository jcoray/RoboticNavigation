//  Sketch to have a Robot move to a location
//  Created by Jakob Coray 02/03/2015 (last edited 04/12/2015)

//  Keep the GPS backup battery voltage between  2.0V~4.3V

//  set the robot's desination from the serial port
//  Have the robot navigate itself using GPS
/*
  Change Log
  * 04/12/2015 - Incorporated MPU9250 Basic Example Code by: Kris Winer


  MPU9250 code by Kris Winer

  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  SDA ---------------------- A4
  SCL ---------------------- A5
  GND ---------------------- GND

 */

#include "Navigation.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Adafruit_ssd1306syp.h>
#include <math.h> //  for some trig functions
#include <Bounce2.h> // https://github.com/thomasfredericks/Bounce-Arduino-Wiring
#include <MPU-9250.h>
#include <PID_v1.h>
#include <Wire.h> //  As if there were not enough libraries already...

double destLat;
double destLon;

const int armMotorPin = 23;
const int leftMotorPin = 25;
const int rightMotorPin = 27;
const int clawMotorPin = 29;

Servo leftMotor;  //2000 forwards, 1000 reverse
Servo rightMotor; //1000 forwards, 2000 reverse
Servo armMotor;  //2000 up, 1000 down
Servo clawMotor; //2000 close, 1000 open

Adafruit_GPS GPS(&Serial1);
#define GPSECHO false // I don't think it is necessary, but if it works...
boolean usingInterrupt = false;

//  Interrupt Code borrowed from CPARKTX's instuctable (uses CC BY-NC-SA license)
//  http://www.instructables.com/id/Arduino-Powered-Autonomous-Vehicle
//  Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect)
{
    GPS.read();
}

// turn interrupt on and off
void useInterrupt(boolean v)
{
    if (v) {
        // Timer0 is already used for millis() - we'll just interrupt somewhere
        // in the middle and call the "Compare A" function above
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
        usingInterrupt = true;
    } else {
        // do not call the interrupt function COMPA anymore
        TIMSK0 &= ~_BV(OCIE0A);
        usingInterrupt = false;
    }
}

//  OLED software i2c pins
const int OLEDSDAPin = 9;
const int OLEDSCLPin = 8;
//  OLED init
Adafruit_ssd1306syp display(OLEDSDAPin, OLEDSCLPin);
/*
const int upPin = 30; //  NC
const int downPin = 31; //  NC
const int leftPin = 32; //  NC
const int rightPin = 33; //  NC
*/
const int enterPin = 13;
/*
Bounce upButton = Bounce();
Bounce downButton = Bounce();
Bounce leftButton = Bounce();
Bounce rightButton = Bounce();
*/
Bounce enterButton = Bounce();

// Used by centerDrive()
const int centerPin = 14;
int rightOffset;
int leftOffset;

enum RobotStatus  {
    UPRIGHT,
    FLIPED,
    GPS_FIXED,
    GPS_NOT_FIXED,
    AT_LOCATION,
    NAVIGATING,
};

double Setpoint, Input, Output;
const double kkp = 2;
const double kki = 5;
const double kkd = 1;
PID robot(&Input, &Output, &Setpoint, kkp, kki, kkd, DIRECT);
float deltaGZ; //
unsigned long serialTime; //this will help us know when to talk with processing
const bool tunePID = true; // use this to set up the PID, more info: http://playground.arduino.cc/Code/PIDLibrary

double endLat, endLong;
bool firstRun = true;

double distanceToDest; // uhhhhhh, global

void setup()
{
    Wire.begin();

    Serial.begin(115200);
    Serial.println(F("Robot GPS"));

    leftMotor.attach(leftMotorPin);
    rightMotor.attach(rightMotorPin);
    armMotor.attach(armMotorPin);
    clawMotor.attach(clawMotorPin);

    //  stabilize motors in stop position (prevents sporadic motor movement)
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
    armMotor.writeMicroseconds(1500);
    clawMotor.writeMicroseconds(1500);
    /*
    //  Set up buttons
    pinMode(upPin,INPUT_PULLUP);
    pinMode(downPin,INPUT_PULLUP);
    pinMode(leftPin,INPUT_PULLUP);
    pinMode(rightPin,INPUT_PULLUP);
    */
    pinMode(enterPin,INPUT_PULLUP);
    /*
    upButton.attach(upPin);
    downButton.attach(downPin);
    leftButton.attach(leftPin);
    rightButton.attach(rightPin);
    */
    enterButton.attach(enterPin);

    /*
    upButton.interval(1);
    downButton.interval(1);
    leftButton.interval(1);
    rightButton.interval(1);
    */
    enterButton.interval(1);

    pinMode(centerPin, INPUT);

    //  Set up display
    display.initialize();
    writeScreen("Coray, Created 2/3/15", 2,"Automous Navigation");

    // the following was taken from example code
    GPS.begin(9600);
    // turnon RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    //GPS.sendCommand(PGCMD_ANTENNA);
    GPS.sendCommand(PGCMD_NOANTENNA);                // turn off antenna status info
    useInterrupt(true);                            // use interrupt to constantly pull data from GPS

    delay(1000);

    //                      MPU-9250


    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW); //disable the internal pullup
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    // Read the WHO_AM_I register, this is a good test of communication
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    Serial.print(F("MPU9250 "));
    Serial.print(F("I AM "));
    Serial.print(c, HEX);
    Serial.print(F(" I should be "));
    Serial.println(0x71, HEX);

    delay(1000);

    if (c == 0x71) { // WHO_AM_I should always be 0x68
        Serial.println(F("MPU9250 is online..."));

        MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
        Serial.print(F("x-axis self test: acceleration trim within : "));
        Serial.print(SelfTest[0], 1);
        Serial.println(F("% of factory value"));
        Serial.print(F("y-axis self test: acceleration trim within : "));
        Serial.print(SelfTest[1], 1);
        Serial.println(F("% of factory value"));
        Serial.print(F("z-axis self test: acceleration trim within : "));
        Serial.print(SelfTest[2], 1);
        Serial.println(F("% of factory value"));
        Serial.print(F("x-axis self test: gyration trim within : "));
        Serial.print(SelfTest[3], 1);
        Serial.println(F("% of factory value"));
        Serial.print(F("y-axis self test: gyration trim within : "));
        Serial.print(SelfTest[4], 1);
        Serial.println(F("% of factory value"));
        Serial.print(F("z-axis self test: gyration trim within : "));
        Serial.print(SelfTest[5], 1);
        Serial.println(F("% of factory value"));

        calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

        delay(1000);

        initMPU9250();
        Serial.println(F("MPU9250 initialized for active data mode....")); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
        Serial.print(F("AK8963 "));
        Serial.print(F("I AM "));
        Serial.print(d, HEX);
        Serial.print(F(" I should be "));
        Serial.println(0x48, HEX);

        delay(1000);

        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magCalibration);
        Serial.println(F("AK8963 initialized for active data mode....")); // Initialize device for active mode read of magnetometer

        if (SerialDebug) {
            //  Serial.println("Calibration values: ");
            Serial.print(F("X-Axis sensitivity adjustment value "));
            Serial.println(magCalibration[0], 2);
            Serial.print(F("Y-Axis sensitivity adjustment value "));
            Serial.println(magCalibration[1], 2);
            Serial.print(F("Z-Axis sensitivity adjustment value "));
            Serial.println(magCalibration[2], 2);
        }

        delay(1000);
    } else {
        Serial.print(F("Could not connect to MPU9250: 0x"));
        Serial.println(c, HEX);
        while (1) ; // Loop forever if communication doesn't happen
    }

    //Setup PID

    Setpoint = 100; //TODO test value
    robot.SetMode(AUTOMATIC);

    //  Wait for GPS to fix and the user to set the destination.
    Serial.println(F("Waiting for GPS to fix..."));
    writeScreen("", 3,"Looking for satellites.");
    while(!GPS.fix) {
        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
        }
    }
    static byte userStatus = 0;
    while (userStatus < 2) { //  Check if the user has finished setup.
        /*
        upButton.update();
        downButton.update();
        leftButton.update();
        rightButton.update();
        */
        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
        }
        enterButton.update();
        userStatus = userSetup();
        Serial.print(F("user setup: "));
        Serial.println(userStatus);
    }
    Serial.println(F("Finished Setup"));
    Serial.print(F("Initial distance to destination:"));
    Serial.println(distanceToDest);
}

void loop()
{
    static unsigned long displayTimer = millis(), MPUTimer = millis();
    if(tunePID) {
        if(millis() > serialTime) {
            SerialReceive();
            SerialSend();
            serialTime+=500;
        }
    }
    readMPU();
	static double bearing = calculateBearing(GPS.latitudeDegrees, destLat, GPS.longitudeDegrees, destLon);
    if (GPS.newNMEAreceived()) {
        Serial.println(F("\nNew NMEA received"));
        if (GPS.parse(GPS.lastNMEA())) {// this also sets the newNMEAreceived() flag to false
            Serial.println(F("Successfully parsed GPS"));
            writeSerialGPS();
            distanceToDest = calculateDistance( GPS.latitudeDegrees, destLat, GPS.longitudeDegrees, destLon);
            bearing = calculateBearing(GPS.latitudeDegrees, destLat, GPS.longitudeDegrees, destLon);
        } else {
            Serial.println(F("ERROR: Failed to parsed GPS"));
        }
    }
    Setpoint = 0; // we want this to be relative 
    serialDebugMPU();
    if (millis() - displayTimer > 2000) {
        writeScreenGPS(distanceToDest, yaw);
        displayTimer = millis();
    }

    double error = GetHeadingError(yaw, bearing);

    //  Drive
    Input = error;
    Serial.print(F("Error: "));
    Serial.println(error);
    robot.Compute();
    centerDrive(); //  Manual calibration to account for motor strength variation
    if(distanceToDest <= 3) { //  within 3 meters of the destination
        drive(0);
        writeScreenGPS(distanceToDest, yaw); // convert from -180,180 to 0,360
        //writeScreen("", AT_LOCATION, "");
        Serial.println(F("Arrived at destination. Press enter to reset"));
        while(!enterButton.fell()){};
    } else if(error > Setpoint) {
        leftMotor.writeMicroseconds(2000 - Output - leftOffset);
        rightMotor.writeMicroseconds(1000 + rightOffset);
    } else {
        leftMotor.writeMicroseconds(2000 - leftOffset);
        rightMotor.writeMicroseconds(1000 + Output + rightOffset);
    }

    if(!GPS.fix) {
        Serial.println(F("Error: GPS lost fix. "));
        writeScreen("", 3,"Looking for satellites.");
    }
}

// Taken from (address(stackoverflow)) by (name)
double GetHeadingError(double initial, double final_head) // Alas, final is a reserved keyword.
{
    if (initial > 360 || initial < 0 || final_head > 360 || final_head < 0) {
        Serial.print(F("Error: heading or bearing outside bounds (0,360). Initial, Final:"));
        Serial.print(initial);
        Serial.print(",");
        Serial.println(final_head);
    }

    double diff = final_head - initial;
    double absDiff = abs(diff);

    if (absDiff <= 180) {
        return absDiff == 180 ? absDiff : diff; // TODO what???
    } else if (final_head > initial) {
        return absDiff - 360;
    } else {
        return 360 - absDiff;
    }
}

void centerDrive()
{
    int rawValue = analogRead(centerPin);
    if(rawValue > 512) { //  change right motor
        rightOffset = map(rawValue, 512, 1024, 0, 100);
        leftOffset = 0;
    } else {
        rightOffset = 0;
        leftOffset = map(rawValue, 512, 0, 0, 100);
    }
}

void serialDebugMPU()
{
    Serial.print(F("\nax = "));
    Serial.print((int)1000 * ax);
    Serial.print(F(" ay = "));
    Serial.print((int)1000 * ay);
    Serial.print(F(" az = "));
    Serial.print((int)1000 * az);
    Serial.println(F(" mg"));
    Serial.print(F("gx = "));
    Serial.print( gx, 2);
    Serial.print(F(" gy = "));
    Serial.print( gy, 2);
    Serial.print(F(" gz = "));
    Serial.print( gz, 2);
    Serial.println(F(" deg/s"));
    Serial.print(F("mx = "));
    Serial.print( (int)mx );
    Serial.print(F(" my = "));
    Serial.print( (int)my );
    Serial.print(F(" mz = "));
    Serial.print( (int)mz );
    Serial.println(F(" mG"));

    Serial.print(F("q0 = "));
    Serial.print(q[0]);
    Serial.print(F(" qx = "));
    Serial.print(q[1]);
    Serial.print(F(" qy = "));
    Serial.print(q[2]);
    Serial.print(F(" qz = "));
    Serial.println(q[3]);
    Serial.print(F("Yaw, Pitch, Roll: "));
    Serial.print(yaw, 2);
    Serial.print(F(", "));
    Serial.print(pitch, 2);
    Serial.print(F(", "));
    Serial.println(roll, 2);

    Serial.print(F("rate = "));
    Serial.print((float)sumCount / sum, 2);
    Serial.println(F(" Hz"));
    Serial.print(F("temp = "));
    Serial.print(temperature, 1);
    Serial.println(F(" C")); // Print T values to tenths of s degree C
}

void readMPU()
{
    //  Read and filter the data from the MPU.
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1] * aRes; // - accelBias[1];
        az = (float)accelCount[2] * aRes; // - accelBias[2];

        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1] * gRes;
        gz = (float)gyroCount[2] * gRes;

        readMagData(magCount);  // Read the x/y/z adc values
        getMres();
        //  I collected bias data on May 7, 2015. I took 12 samples.
        //  The uncertainty is at the two sigma confidence interval.
        //  The units are milliGauss.
        //  Note: Every MPU is different. You will need to take your
        //  own measurements by setting magbias = 0, spinning the sensor
        //  around, recording the min and max values in every direction,
        //  and taking the mean.
        magbias[0] = 120.78; //  +- 32.36  x-axis
        magbias[1] = -64.39; //  +- 37.55  y-axis
        magbias[2] = -69.50; //  +- 20.03  z-axis
        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1] * mRes * magCalibration[1] - magbias[1];
        mz = (float)magCount[2] * mRes * magCalibration[2] - magbias[2];
    }
    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!
    // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz); // trying this instead, hopefully more stable
    //  MahonyQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, mz);

    if(gz < -.02 || gz > .02) { // should hopefully prevent too much drift
        deltaGZ += deltat  * gz; //  rynman sum to find rotation;
    }

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   -= 10.0; // Declination at Arlington, VA is about 10 degrees on 04/12/2015. Added not subtracted because it measures counter-clockwise
    yaw = map(fmod(yaw + 360, 360),0,360,360,0); //  Convert from (-180, 180) to (0, 360)
    roll  *= 180.0f / PI;
}

double readDouble()
{
    //  This serial read process was adapted from code by Scott C.
    //  http://arduinobasics.blogspot.com/2012/07/arduino-basics-simple-arduino-serial.html
    Serial.println(F("Enter a double or 'h' for help."));
    double integers = 0, decimals = 0, value = 0, decimalPlace = 1;
    byte byteRead = 0;
    int numOfDec = 0;
    boolean integer = true, negative = false;
    while(1) {
        if(Serial.available()) {
            byteRead = Serial.read();

            if(byteRead > 47 && byteRead < 58) { //  Listen for numbers between 0-9.
                if(integer) {
                    integers = (integers * 10) + (byteRead - 48);
                }

                else {
                    decimals = (decimals * 10) + (byteRead - 48);
                    decimalPlace = decimalPlace * 10;
                    numOfDec++;
                }
            }

            else if(byteRead == 45) {
                negative = true;
            }

            else if(byteRead == 46) { //  Listen for a decimal point.
                integer = false;
            }

            else if(byteRead == 100) { //  Listen for a 'd'.
                value = integers + (decimals / decimalPlace);
                if(negative) value *= -1;
                Serial.println(value, numOfDec);
                return value;
            }

            else if(byteRead == 104) { //  Listen for a 'h'.
                Serial.println(F("Enter number followed by the lowercase letter"
                                 " 'd'. For example, 12.342d or -4d."));
            }
        }
    }
}


void drive(int movement)
{
    // 0 -> stop; 1 -> forwad; 2 -> backward; 3 -> left; 4 -> right
    switch(movement)  {
    case 0:    //stop
        leftMotor.writeMicroseconds(1500);
        rightMotor.writeMicroseconds(1500);
        break;

    case 1:    //forwards
        leftMotor.writeMicroseconds(2000);
        rightMotor.writeMicroseconds(1000);
        break;

    case 2:    //backwards
        leftMotor.writeMicroseconds(1000);
        rightMotor.writeMicroseconds(2000);
        break;

    case 3:    //left
        leftMotor.writeMicroseconds(1000);
        rightMotor.writeMicroseconds(1000);
        break;

    case 4:    //right
        leftMotor.writeMicroseconds(2000);
        rightMotor.writeMicroseconds(2000);
        break;
    }
}

void writeSerialGPS()
{
    Serial.print(F("\nTime: "));
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print(F("Date: "));
    Serial.print(GPS.day, DEC);
    Serial.print('/');
    Serial.print(GPS.month, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print(F("Fix: "));
    Serial.print((int)GPS.fix);
    Serial.print(F(" quality: "));
    Serial.println((int)GPS.fixquality);
    Serial.print(F("Location: "));
    Serial.print(GPS.latitude, 4);
    Serial.print(GPS.lat);
    Serial.print(F(", "));
    Serial.print(GPS.longitude, 4);
    Serial.println(GPS.lon);
    Serial.print(F("Location (degrees): "));
    Serial.print(GPS.latitudeDegrees,7);
    Serial.print(F(", "));
    Serial.println(GPS.longitudeDegrees,7);
    Serial.print(F("Speed (knots): "));
    Serial.println(GPS.speed);
    Serial.print(F("Angle: "));
    Serial.println(GPS.angle);
    Serial.print(F("Altitude: "));
    Serial.println(GPS.altitude);
    Serial.print(F("Satellites: "));
    Serial.println((int)GPS.satellites);
}

void writeScreenGPS(double distance, double bearing)
{
    display.clear();
    display.setCursor(0,0);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.println(F("Robot GPS by Jakob"));
    int errorMargin = 5;  // TODO may need to play around with this some more. It may need to be a function of the number of satilights and the fix quality.
    if(distance > errorMargin) {
        display.print(F("Dst: "));
        display.print(distance, 0);
        display.print(F("m @ "));
        display.print(bearing, 0);
        printHeading(bearing);
    } else if(distance <= errorMargin) {
        writeScreen("", AT_LOCATION,"");
        return;
    } else {
        display.println("");
    }

    display.print(GPS.hour, DEC);
    display.print(F(":"));
    display.print(GPS.minute, DEC);
    display.print(F(" GMT "));
    display.print(GPS.month, DEC);
    display.print('/');
    display.print(GPS.day, DEC);
    display.print('/');
    display.println(GPS.year, DEC);

    display.print(F("Lat: "));
    display.print(GPS.latitudeDegrees,6);
    display.println(GPS.lat);
    display.print(F("Lon: "));
    display.print(GPS.longitudeDegrees,6);
    display.println(GPS.lon);
    display.print(F("Vel: "));
    display.print(GPS.speed,1);
    display.print(F("mi/h "));
    display.print(GPS.angle,0); //TODO use miles or meters, not both
    printHeading(GPS.angle);
    display.print(F("Alt: "));
    display.println(GPS.altitude,1);
    display.print(F("Sat: "));
    display.print((int)GPS.satellites);
    display.update();

}

void writeScreenMPU()
{
    display.clear();
    display.setCursor(0,0);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.println(F("MPU-9250 Output"));
    display.println(F("   x/Y   y/P   z/R"));

    display.print(F("A: "));
    display.print(ax);
    display.print(F(" "));
    display.print(ay);
    display.print(F(" "));
    display.println(az);
    display.print(F("G: "));
    display.print(gx);
    display.print(F(" "));
    display.print(gy);
    display.print(F(" "));
    display.println(gz);
    display.print(F("M: "));
    display.print(mx,0);
    display.print(F(" "));
    display.print(my,0);
    display.print(F(" "));
    display.println(mz,0);
    display.print(F("Q: \n"));
    display.print(F("O: "));
    display.print(yaw,1);
    display.print(F(" "));
    display.print(pitch,1);
    display.print(F(" "));
    display.println(roll,1);
    display.update();
}

byte userSetup()
{
    static byte userStatus = 0;

    display.clear();
    display.setCursor(0,0);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.println(F("Robot GPS by Jakob"));
    display.println("");

    if(userStatus == 0) {
        display.println(F("Press enter to set current location as robot destination."));
        if(enterButton.fell()) {
            destLat = GPS.latitudeDegrees;
            destLon = GPS.longitudeDegrees;
            userStatus = 1;
            return 1;
        }
    } else if(userStatus == 1) {
        display.println(F("Press enter to navigate to destination."));
        if(enterButton.fell()) {
            userStatus = 2;
            drive(1); //  drive straight to get bearing from GPS by dead reckoning
            delay(3000);
            return 2;
        }
    }

    display.print(F("Lat: "));
    display.print(GPS.latitudeDegrees,6);
    display.println(GPS.lat);
    display.print(F("Lon: "));
    display.print(GPS.longitudeDegrees,6);
    display.println(GPS.lon);
    display.update();
    return userStatus;
}

void writeScreen(char message[19], int robotStatus, char text[47])
{
    //do not make message or  more than 18 characters long
    static int previousCall = 100; //abratrary
    if(previousCall == robotStatus) return;

    display.clear();
    display.setCursor(0,0);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.println(F("Robot GPS by Jakob"));
    display.println(message);
    switch(robotStatus)  {
    case(UPRIGHT):
        display.println(F("        >===o"));
        display.println(F("        ____]"));
        display.println(F("        *   *"));
        break;
    case(FLIPED):
        display.println(F("        *)   {}"));
        display.println(F("         |   / "));
        display.println(F("        *)==/  "));
        break;
    case(GPS_FIXED):
        display.println(F("  |       >===o"));
        display.println(F(" [}(      ____]"));
        display.println(F("  |       *   *"));
        break;
    case(GPS_NOT_FIXED):
        display.println(F("  |    /^\\    >===o"));
        display.println(F(" [}(     /    ____]"));
        display.println(F("  |     !     *   *"));
        break;
    case(AT_LOCATION):
        display.setTextSize(3);
        display.println(F("ARRIVED"));
        display.println("");
        display.setTextSize(1);
        break;
    }
    display.println();
    display.println(text);
    display.update();
    previousCall = robotStatus;
}

void printHeading(float heading)
{
    //  Adapted from https://github.com/adafruit/Flora-GPS-Jacket
    //Remember: this is not the direction you are heading, it is the direction to the destination (north = forward).
    if ((heading > 348.75)||(heading < 11.25))
        display.println(F("  N"));
    if ((heading >= 11.25)&&(heading < 33.75))
        display.println(F("NNE"));
    if ((heading >= 33.75)&&(heading < 56.25))
        display.println(F(" NE"));
    if ((heading >= 56.25)&&(heading < 78.75))
        display.println(F("ENE"));
    if ((heading >= 78.75)&&(heading < 101.25))
        display.println(F("  E"));
    if ((heading >= 101.25)&&(heading < 123.75))
        display.println(F("ESE"));
    if ((heading >= 123.75)&&(heading < 146.25))
        display.println(F(" SE"));
    if ((heading >= 146.25)&&(heading < 168.75))
        display.println(F("SSE"));
    if ((heading >= 168.75)&&(heading < 191.25))
        display.println(F("  S"));
    if ((heading >= 191.25)&&(heading < 213.75))
        display.println(F("SSW"));
    if ((heading >= 213.75)&&(heading < 236.25))
        display.println(F(" SW"));
    if ((heading >= 236.25)&&(heading < 258.75))
        display.println(F("WSW"));
    if ((heading >= 258.75)&&(heading < 281.25))
        display.println(F("  W"));
    if ((heading >= 281.25)&&(heading < 303.75))
        display.println(F("WNW"));
    if ((heading >= 303.75)&&(heading < 326.25))
        display.println(F(" NW"));
    if ((heading >= 326.25)&&(heading < 348.75))
        display.println(F("NWN"));
}


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
    byte asBytes[24];    // us take the byte array
    float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

    // read the bytes sent from Processing
    int index=0;
    byte Auto_Man = -1;
    byte Direct_Reverse = -1;
    while(Serial.available()&&index<26) {
        if(index==0) Auto_Man = Serial.read();
        else if(index==1) Direct_Reverse = Serial.read();
        else foo.asBytes[index-2] = Serial.read();
        index++;
    }

    // if the information we got was in the correct format,
    // read it into the system
    if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1)) {
        Setpoint=double(foo.asFloat[0]);
        //Input=double(foo.asFloat[1]);       // * the user has the ability to send the
        //   value of "Input"  in most cases (as
        //   in this one) this is not needed.
        if(Auto_Man==0) {                     // * only change the output if we are in
            //   manual mode.  otherwise we'll get an
            Output=double(foo.asFloat[2]);      //   output blip, then the controller will
        }                                     //   overwrite.

        double p, i, d;                       // * read in and set the controller tunings
        p = double(foo.asFloat[3]);           //
        i = double(foo.asFloat[4]);           //
        d = double(foo.asFloat[5]);           //
        robot.SetTunings(p, i, d);            //

        if(Auto_Man==0) robot.SetMode(MANUAL);// * set the controller mode
        else robot.SetMode(AUTOMATIC);             //

        if(Direct_Reverse==0) robot.SetControllerDirection(DIRECT);// * set the controller Direction
        else robot.SetControllerDirection(REVERSE);          //
    }
    Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
    Serial.print("PID ");
    Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(Input);
    Serial.print(" ");
    Serial.print(Output);
    Serial.print(" ");
    Serial.print(robot.GetKp());
    Serial.print(" ");
    Serial.print(robot.GetKi());
    Serial.print(" ");
    Serial.print(robot.GetKd());
    Serial.print(" ");
    if(robot.GetMode()==AUTOMATIC) Serial.print("Automatic");
    else Serial.print("Manual");
    Serial.print(" ");
    if(robot.GetDirection()==DIRECT) Serial.println("Direct");
    else Serial.println("Reverse");
}
