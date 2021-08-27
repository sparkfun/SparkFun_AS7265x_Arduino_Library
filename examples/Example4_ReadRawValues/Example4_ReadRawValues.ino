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
  Open the serial monitor at 115200 baud to see the output
*/

#include "SparkFun_AS7265X.h" //Click here to get the library: http://librarymanager/All#SparkFun_AS7265X
AS7265X sensor;

#include <Wire.h>

void setup()
{
  Serial.begin(115200);
  Serial.println("AS7265x Spectral Triad Example");

  if (sensor.begin() == false)
  {
    Serial.println("Sensor does not appear to be connected. Please check wiring. Freezing...");
    while (1)
      ;
  }

  //Once the sensor is started we can increase the I2C speed
  Wire.setClock(400000);

  sensor.disableIndicator();

  Serial.println("A,B,C,D,E,F,G,H,R,I,S,J,T,U,V,W,K,L");
}

void loop()
{
  sensor.takeMeasurements(); //This is a hard wait while all 18 channels are measured

  Serial.print(sensor.getA()); //410nm
  Serial.print(",");
  Serial.print(sensor.getB()); //435nm
  Serial.print(",");
  Serial.print(sensor.getC()); //460nm
  Serial.print(",");
  Serial.print(sensor.getD()); //485nm
  Serial.print(",");
  Serial.print(sensor.getE()); //510nm
  Serial.print(",");
  Serial.print(sensor.getF()); //535nm
  Serial.print(",");

  Serial.print(sensor.getG()); //560nm
  Serial.print(",");
  Serial.print(sensor.getH()); //585nm
  Serial.print(",");
  Serial.print(sensor.getR()); //610nm
  Serial.print(",");
  Serial.print(sensor.getI()); //645nm
  Serial.print(",");
  Serial.print(sensor.getS()); //680nm
  Serial.print(",");
  Serial.print(sensor.getJ()); //705nm
  Serial.print(",");

  Serial.print(sensor.getT()); //730nm
  Serial.print(",");
  Serial.print(sensor.getU()); //760nm
  Serial.print(",");
  Serial.print(sensor.getV()); //810nm
  Serial.print(",");
  Serial.print(sensor.getW()); //860nm
  Serial.print(",");
  Serial.print(sensor.getK()); //900nm
  Serial.print(",");
  Serial.print(sensor.getL()); //940nm
  Serial.print(",");

  Serial.println();
}
