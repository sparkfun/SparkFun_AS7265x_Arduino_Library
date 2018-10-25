/*
  Read the 18 channels of spectral light over I2C using the Spectral Triad
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 25th, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to output the raw sensor values. This is probably never needed since the 
  calibrated values are tuned to each sensor. But it does run faster (2 bytes per channel instead of 4)
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15050

  Hardware Connections:
  Plug a Qwiic cable into the Spectral Triad and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 9600 baud to see the output
*/

#include "SparkFun_AS7265X.h" //Click here to get the library: http://librarymanager/All#SparkFun_AS7265X
AS7265X sensor;

#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Serial.println("AS7265x Spectral Triad Example");

  if(sensor.begin() == false)
  {
    Serial.println("Sensor does not appear to be connected. Please check wiring. Freezing...");
    while(1);
  }

  //Once the sensor is started we can increase the I2C speed
  Wire.setClock(400000);

  sensor.disableIndicator();

  Serial.println("A,B,C,D,E,F,G,H,I,J,K,L,R,S,T,U,V,W");
}

void loop() {
  sensor.takeMeasurementsWithBulb(); //This is a hard wait while all 18 channels are measured

  Serial.print(sensor.getA());
  Serial.print(",");
  Serial.print(sensor.getB());
  Serial.print(",");
  Serial.print(sensor.getC());
  Serial.print(",");
  Serial.print(sensor.getD());
  Serial.print(",");
  Serial.print(sensor.getE());
  Serial.print(",");
  Serial.print(sensor.getF());
  Serial.print(",");

  Serial.print(sensor.getG());
  Serial.print(",");
  Serial.print(sensor.getH());
  Serial.print(",");
  Serial.print(sensor.getI());
  Serial.print(",");
  Serial.print(sensor.getJ());
  Serial.print(",");
  Serial.print(sensor.getK());
  Serial.print(",");
  Serial.print(sensor.getL());
  Serial.print(",");

  Serial.print(sensor.getR());
  Serial.print(",");
  Serial.print(sensor.getS());
  Serial.print(",");
  Serial.print(sensor.getT());
  Serial.print(",");
  Serial.print(sensor.getU());
  Serial.print(",");
  Serial.print(sensor.getV());
  Serial.print(",");
  Serial.print(sensor.getW());
  Serial.print(",");

  Serial.println();
}

