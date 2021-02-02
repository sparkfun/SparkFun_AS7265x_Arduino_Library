/*
  Bulb test - see white bulb incorrectly turns off!

  Hardware Connections:
  Plug a Qwiic cable into the Spectral Triad and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output

  Quiic cable - Arduino Connection
  Black = GND
  Red = 3.3V
  Blue = SDA - A4
  Yellow = SCL - A5
*/
#include <SparkFun_AS7265X.h>
AS7265X sensor;

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("");
  Serial.println("AS7265x Spectral Triad Example");

  if(sensor.begin() == false)
  {
    Serial.println("Sensor does not appear to be connected. Please check wiring. Freezing...");
    while(1);
  }
}

void loop() {
  int i=1;

  Serial.print(i++); Serial.println(". Turn on white");
  sensor.enableBulb(AS7265x_LED_WHITE);
  delay(1000);

  Serial.print(i++); Serial.println(". Turn on UV (and/or IR)");
  //sensor.enableBulb(AS7265x_LED_IR);
  sensor.enableBulb(AS7265x_LED_UV);
  delay(1000);

  Serial.print(i++); Serial.println(". Turn off UV (and/or IR)");
  //sensor.disableBulb(AS7265x_LED_IR);
  sensor.disableBulb(AS7265x_LED_UV);
  delay(1000);
  
  Serial.print(i++); Serial.println(". Turn on indicator -- This turns off white!");
  sensor.enableIndicator();
  delay(1000);

  Serial.print(i++); Serial.println(". Turn on UV (and/or IR)");
  //sensor.enableBulb(AS7265x_LED_IR);
  sensor.enableBulb(AS7265x_LED_UV);
  delay(1000);

  Serial.print(i++); Serial.println(". Turn off indicator -- This turns on white!");
  sensor.disableIndicator();
  delay(1000);

  Serial.print(i++); Serial.println(". Go back");
  sensor.disableIndicator();
  sensor.disableBulb(AS7265x_LED_WHITE);
  sensor.disableBulb(AS7265x_LED_IR);
  sensor.disableBulb(AS7265x_LED_UV);
  delay(5000);
}
