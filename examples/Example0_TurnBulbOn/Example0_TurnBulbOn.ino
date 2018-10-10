/*
  This is a library written for the AS726X Spectral Sensor (Visible or IR) with I2C firmware
  specially loaded. SparkFun sells these at its website: www.sparkfun.com

  Written by Nathan Seidle & Andrew England @ SparkFun Electronics, July 12th, 2017

  https://github.com/sparkfun/Qwiic_Spectral_Sensor_AS726X

  Do you like this library? Help support SparkFun. Buy a board!

  Development environment specifics:
  Arduino IDE 1.8.1

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.


  ---AVAILABLE FUNCTIONS---
  AS726X(TwoWire &wirePort = Wire, byte gain = 3, byte measurementMode = 3);
  void takeMeasurements();
  void takeMeasurementsWithBulb();
  void printMeasurements();
  byte getTemperature();
  float getTemperatureF();
  void setMeasurementMode(byte mode);
  boolean dataAvailable();
  void enableIndicator();
  void disableIndicator();
  void setIndicatorCurrent(byte current);
  void enableBulb();
  void disableBulb();
  void setBulbCurrent(byte current);
  void softReset();
  void setGain(byte gain);
  void setIntegrationTime(byte integrationValue);
  void enableInterrupt();
  void disableInterrupt();

  //Get the various color readings
  int getViolet();
  int getBlue();
  int getGreen();
  int getYellow();
  int getOrange();
  int getRed();

  //Get the various NIR readings
  int getR();
  int getS();
  int getT();
  int getU();
  int getV();
  int getW();

  //Returns the various calibration data
  float getCalibratedViolet();
  float getCalibratedBlue();
  float getCalibratedGreen();
  float getCalibratedYellow();
  float getCalibratedOrange();
  float getCalibratedRed();

  float getCalibratedR();
  float getCalibratedS();
  float getCalibratedT();
  float getCalibratedU();
  float getCalibratedV();
  float getCalibratedW();
*/

#include "AS7265X.h"
//#include "Users/kevin.kuwata/Documents/GitHub/SparkFun_AS7265x_Arduino_Library/src/AS7265X.h"
AS7265X sensor;

void setup() {
  sensor.begin();
  sensor.setBulbCurrentAS72651(AS7265X_LED_CURRENT_LIMIT_25MA);
  sensor.setBulbCurrentAS72652(AS7265X_LED_CURRENT_LIMIT_25MA);
  sensor.setBulbCurrentAS72653(AS7265X_LED_CURRENT_LIMIT_25MA);

  Serial.begin(9600);
  Serial.println("started");
  Serial.println("reading status register");
  Serial.print(sensor.readRegister(AS7265X_STATUS_REG), BIN);
}

void loop() {
  delay(100);
  //sensor.enableBulbAS72651(); // lets make it 0x07 for the register. 
  //sensor.enableBulbAS72652(); // lets make it 0x07 for the register. 
 // sensor.enableBulb(AS72651_DEV_SELECT_NIR); // lets make it 0x07 for the register. 
  sensor.enableBulb(0x02); // lets make it 0x07 for the register. 
  sensor.enableBulb(0x01); // lets make it 0x07 for the register. 
  sensor.enableBulb(0x00); // lets make it 0x07 for the register. 
  //sensor.enableBulb(AS72652_DEV_SELECT_VISIBLE); // lets make it 0x07 for the register. 
  delay(100);
 // sensor.enableBulbAS72651();
  //sensor.enableBulbAS72652();
 // sensor.enableBulbAS72653();

  //sensor.disableBulbAS72652();
}
