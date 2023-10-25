//WE HAVE ADAPTED SPARK-FUN'S CIRCULAR BUFFER EXAMPLE TO MEET OUR NEEDS ORIGINAL LICENSE BELOW
//We take the averaged positon and transmit over the serial connection
//We tested this on an Arduino Nano IoT
//The nano should be plugged into the USB connection of the Create3 adapter board.
//Follow iRobot's instructions for configuring serial communications. 
//Make sure that the Bluetooth/USB switch on the adapter board is on the "USB" setting



/******************************************************************************
AveragingReadBarOnly.ino

A sketch for reading sensor data into a circular buffer

Marshall Taylor, SparkFun Engineering

5-27-2015

Library:
https://github.com/sparkfun/SparkFun_Line_Follower_Array_Arduino_Library
Product:
https://github.com/sparkfun/Line_Follower_Array

This sketch shows how to use the circular buffer class to create a history of
sensor bar scans.
The buffer configured with CBUFFER_SIZE to have a length of 100 16bit integers.

Resources:
sensorbar.h

Development environment specifics:
arduino > v1.6.4
hw v1.0

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
#define CBUFFER_SIZE 100
#define DEBUG false

#include "Wire.h"
#include "sensorbar.h"

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);

CircularBuffer positionHistory(CBUFFER_SIZE);


void setup()
{
  Serial.begin(115200);  // start serial for output

  #if DEBUG
  Serial.println("Program started.");
  Serial.println();
  #endif
  
  //For this demo, the IR will only be turned on during reads.
  mySensorBar.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default dark on light
  //mySensorBar.clearInvertBits();
  //Other option: light line on dark
  mySensorBar.setInvertBits();//USE LIGHT ON DARK; IT'S GENERALLY MORE RELIABLE UNLESS YOU HAVE A BLACK LINE ON A VERY BRIGHT BACKGROUND.
  
  uint8_t returnStatus = mySensorBar.begin();
  if(returnStatus)
  {
    #if DEBUG
	  Serial.println("sx1509 IC communication OK");
    #endif
  }
  else
  {
    #if DEBUG
	  Serial.println("sx1509 IC communication FAILED!");
    #endif
	  while(1);
  }
  #if DEBUG
  Serial.println();
  #endif
}

void loop()
{
  delay(25);//don't overload the Create3 with messages.

  //Get the data from the bar and save it to the circular buffer positionHistory.
  int temp = mySensorBar.getDensity();
  if( (temp < 4)&&(temp > 0) )
  {
    positionHistory.pushElement( mySensorBar.getPosition());
  }

  //send values over serial to Create3
  
  int16_t i;
	//Get an average of the last 'n' readings
  int16_t avePos = positionHistory.averageLast( 10 );
  //values may be beyond normal signed 8bit range, but valid ones are -127 to 127
  //we can clamp it.
  int16_t clamped = constrain(avePos, -127, 127);
  int16_t shifted = clamped + 128;
  uint8_t bytePos = (uint8_t)shifted;//unsigned value for simplicity on the receiving end in ros2
  Serial.write(bytePos);

  //YOU CAN UNCOMMENT THIS CODE OUT (BELOW) IF YOU WANT TO SEE SPARKFUN'S METER
  //THIS ONLY WORKS IF YOU CONNECT THE ARDUINO DIRECTLY TO YOUR COMPUTER RATHER THAN CREATE3, BUT IS GOOD FOR DEBUGGING
  //   Serial.print("Scale = 5/char :");
  //   for( i = -130; i <= 130; i = i + 5 )
  //   {
  //     if( i < 0 )
  //     {
  //       //if avePos within 5 of i
  //       if((avePos > (i - 3)) && (avePos <= (i + 2)))
  //       {
  //         Serial.print("*");
  //       }
  //       else
  //       {
  //         Serial.print("-");
  //       }
  //     }
  //     else if( i == 0 )
  //     {
  //       //if avePos within 5 of i
  //       if((avePos > (i - 3)) && (avePos <= (i + 2)))
  //       {
  //         Serial.print("*");
  //       }
  //       else
  //       {
  //         Serial.print("+");
  //       }
  //     }
  //     else if( i > 0 )
  //     {
  //       //if avePos within 5 of i
  //       if((avePos > (i - 3)) && (avePos <= (i + 2)))
  //       {
  //         Serial.print("*");
  //       }
  //       else
  //       {
  //         Serial.print("-");
  //       }
  //     }
  //   }
  //   Serial.print(" avePos = ");
  //   Serial.println(avePos);
  
}






