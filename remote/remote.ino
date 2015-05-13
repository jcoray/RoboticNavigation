/*
 Code for tranmisting to a robot with an arm and a claw.
 Created by Jakob Coray, 9/20/2014. Last edited 9/20/14.
 
 Version 0.0
 
 Description: (last updated 9/20/14)
 
 Design: 
 
 
 Wiring: (last updated 9/2-/14)
 Hardware SPI: 
 MISO -> 50
 MOSI -> 51
 SCK -> 52
 
 Configurable:
 CE -> 8
 CSN -> 7
 
 For pinout see global variables
 file directory or web adress for CAD file/pictures
 
 
 Changelog:
 v0.0: (9/20/14) Created code, combined elements from Wiitest.ino, 
 sendWireless.ino, and ping_client.ino
 
 
 Todo:
 
 
 
 //http://i2.wp.com/make-images.s3.amazonaws.com/en5A4pRIfXPGQEHl.jpg?resize=620%2C465
 //for wire set up
 */


#include <Wire.h>

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

byte jx ,jy, ax, ay, az, z, c;
byte driveData[7] = {
  jx, jy, ax, ay, az, z, c};

//----------------------------------setup-------------------------------------

void setup()
{
  Serial.begin(19200);
  Serial.println("hello");


  //  Mirf.cePin = 7;
  //  Mirf.csnPin = 8;

  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setTADDR((byte *)"clie1");
  Mirf.payload = 7;
  Mirf.config();

  Serial.println("Beginning ... "); 

  //nunchuck_setpowerpins(); // use analog pins 2&3 as fake gnd & pwr
  nunchuck_init(); // send the initilization handshake
  Serial.print ("Finished setup\n");
}

//------------------------------------loop----------------------------------------

void loop()
{
  byte info[7];
  for(int i= 0; i < 7; i++){
    info[i] = driveData[i];
  }

  Mirf.send((byte *)&info);

  while(Mirf.isSending()){
  }
  Serial.println("Finished sending");

  nunchuck_get_data();
  nunchuck_print_data();
  delay(10);
}

//----------------------------------Nunchuck functions------------------------------

static uint8_t nunchuck_buf[6];   // array to store nunchuck data,

// Uses port C (analog in) pins as power & ground for Nunchuck
static void nunchuck_setpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
  DDRC |= _BV(pwrpin) | _BV(gndpin);
  PORTC &=~ _BV(gndpin);
  PORTC |=  _BV(pwrpin);
  delay(100);  // wait for things to stabilize        
}

// initialize the I2C system, join the I2C bus,
// and tell the nunchuck we're talking to it
void nunchuck_init()
{ 
  Wire.begin();	                // join i2c bus as master
  Serial.println("begin 1");
  Wire.begin();
  Serial.println("begin 2");
  Wire.begin();
  Serial.println("begin 3");
  Wire.beginTransmission(0x52);	// transmit to device 0x52
  Serial.println("transmit to device 0x52");
  Wire.write(0x40);		// sends memory address
  Serial.println("sends memory address");
  Wire.write(0x00);		// sends sent a zero. 
  Serial.println("sends sent a zero.");
  Wire.endTransmission();	// stop transmitting
  Serial.println("stop transmitting");
}

// Send a request for data to the nunchuck
// was "send_zero()"
void nunchuck_send_request()
{
  Wire.beginTransmission(0x52);	// transmit to device 0x52
  Wire.write(0x00);		// sends one byte
  Wire.endTransmission();	// stop transmitting
}

// Receive data back from the nunchuck, 
int nunchuck_get_data()
{
  int cnt=0;
  Wire.requestFrom (0x52, 6);	// request data from nunchuck
  while (Wire.available ()) {
    // receive byte as an integer
    nunchuck_buf[cnt] = nunchuk_decode_byte(Wire.read());
    cnt++;
  }
  nunchuck_send_request();  // send request for next data payload
  // If we recieved the 6 bytes, then go print them
  if (cnt >= 5) {
    return 1;   // success
  }
  return 0; //failure
}

// Print the input data we have recieved
// accel data is 10 bits long
// so we read 8 bits, then we have to add
// on the last 2 bits.  That is why I
// multiply them by 2 * 2
void nunchuck_print_data()
{ 
  static int i=0;
  int joy_x_axis = nunchuck_buf[0];
  int joy_y_axis = nunchuck_buf[1];
  int accel_x_axis = nunchuck_buf[2]; // * 2 * 2; 
  int accel_y_axis = nunchuck_buf[3]; // * 2 * 2;
  int accel_z_axis = nunchuck_buf[4]; // * 2 * 2;

  int z_button = 0;
  int c_button = 0;

  // byte nunchuck_buf[5] contains bits for z and c buttons
  // it also contains the least significant bits for the accelerometer data
  // so we have to check each bit of byte outbuf[5]
  if ((nunchuck_buf[5] >> 0) & 1) 
    z_button = 1;
  if ((nunchuck_buf[5] >> 1) & 1)
    c_button = 1;

  if ((nunchuck_buf[5] >> 2) & 1) 
    accel_x_axis += 2;
  if ((nunchuck_buf[5] >> 3) & 1)
    accel_x_axis += 1;

  if ((nunchuck_buf[5] >> 4) & 1)
    accel_y_axis += 2;
  if ((nunchuck_buf[5] >> 5) & 1)
    accel_y_axis += 1;

  if ((nunchuck_buf[5] >> 6) & 1)
    accel_z_axis += 2;
  if ((nunchuck_buf[5] >> 7) & 1)
    accel_z_axis += 1;


  driveData[0] = byte(joy_x_axis);
  driveData[1] = byte(joy_y_axis);
  driveData[2] = byte(accel_x_axis);
  driveData[3] = byte(accel_y_axis);
  driveData[4] = byte(accel_z_axis);
  driveData[5] = byte(z_button);
  driveData[6] = byte(c_button);

  Serial.print(i,DEC);
  Serial.print("\t");

  Serial.print("joy:");
  Serial.print(joy_x_axis,DEC);
  Serial.print(",");
  Serial.print(joy_y_axis, DEC);
  Serial.print("  \t");

  Serial.print("acc:");
  Serial.print(accel_x_axis, DEC);
  Serial.print(",");
  Serial.print(accel_y_axis, DEC);
  Serial.print(",");
  Serial.print(accel_z_axis, DEC);
  Serial.print("\t");

  Serial.print("but:");
  Serial.print(z_button, DEC);
  Serial.print(",");
  Serial.print(c_button, DEC);

  Serial.print("\r\n");  // newline
  i++;
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char nunchuk_decode_byte (char x)
{
  x = (x ^ 0x17) + 0x17;
  return x;
}



//----------------------------------wireless functions------------------------------ 




