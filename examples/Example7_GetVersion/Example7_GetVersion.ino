/*
  Read the 18 channels of spectral light over I2C using the Spectral Triad
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 25th, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to output the current hardware and firmware version of the Triad.
  
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

  byte deviceType = sensor.getDeviceType();
  Serial.print("AMS Device Type: 0x");
  Serial.println(deviceType, HEX);

  byte hardwareVersion = sensor.getHardwareVersion();
  Serial.print("AMS Hardware Version: 0x");
  Serial.println(hardwareVersion, HEX);

  byte majorFirmwareVersion = sensor.getMajorFirmwareVersion();
  Serial.print("Major Firmware Version: 0x");
  Serial.println(majorFirmwareVersion, HEX);

  byte patchFirmwareVersion = sensor.getPatchFirmwareVersion();
  Serial.print("Patch Firmware Version: 0x");
  Serial.println(patchFirmwareVersion, HEX);

  byte buildFirmwareVersion = sensor.getBuildFirmwareVersion();
  Serial.print("Build Firmware Version: 0x");
  Serial.println(buildFirmwareVersion, HEX);
}

void loop() {
  //Do nothing
}

