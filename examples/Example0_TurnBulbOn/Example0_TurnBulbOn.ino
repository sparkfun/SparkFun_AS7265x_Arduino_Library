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

#include "SparkFun_AS7265X.h" //Click here to get the library: http://librarymanager/All#SparkFun_AS7265X
AS7265X sensor;

void setup() {
  Serial.begin(115200);
  Serial.println("AS7265x Spectral Triad Example");

  if(sensor.begin() == false)
  {
    Serial.println("Sensor does not appear to be connected. Please check wiring. Freezing...");
    while(1);
  }
  
/*  Serial.print("HW verH: 0x");
  Serial.print(sensor.virtualReadRegister(AS72651_HW_VERSION_HIGH), HEX);
  Serial.println(sensor.virtualReadRegister(AS72651_HW_VERSION_LOW), HEX);

  Serial.print("FW Min: 0x");
  Serial.println(sensor.virtualReadRegister(AS72651_FW_VERSION_MINOR), HEX);

  Serial.print("FW Maj: 0x");
  Serial.println(sensor.virtualReadRegister(AS72651_FW_VERSION_MAJOR), HEX);
*/
  //Serial.print("Temp: 0x");
  //Serial.println(sensor.getTemperature());

  //Serial.print("TempAvg: 0x");
  //Serial.println(sensor.getTemperatureAverage());

  Serial.print("Integration time: ");
  Serial.println(sensor.virtualReadRegister(AS7265X_INTERGRATION_TIME));

  sensor.setIntegrationTime(100);

  Serial.print("Integration time: ");
  Serial.println(sensor.virtualReadRegister(AS7265X_INTERGRATION_TIME));

  //sensor.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_25MA, AS72651_NIR);
  //sensor.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_25MA, AS72652_VISIBLE);
  //sensor.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_25MA, AS72653_UV);

  Serial.println("A,B,C,D,E,F,G,H,I,J,K,L,R,S,T,U,V,W");
}

void loop() {
  sensor.takeMeasurements(); //This is a hard wait while all 18 channels are measured

  Serial.print(sensor.getCalibratedA());
  Serial.print(",");
  Serial.print(sensor.getCalibratedB());
  Serial.print(",");
  Serial.print(sensor.getCalibratedC());
  Serial.print(",");
  Serial.print(sensor.getCalibratedD());
  Serial.print(",");
  Serial.print(sensor.getCalibratedE());
  Serial.print(",");
  Serial.print(sensor.getCalibratedF());
  Serial.print(",");

  Serial.print(sensor.getCalibratedG());
  Serial.print(",");
  Serial.print(sensor.getCalibratedH());
  Serial.print(",");
  Serial.print(sensor.getCalibratedI());
  Serial.print(",");
  Serial.print(sensor.getCalibratedJ());
  Serial.print(",");
  Serial.print(sensor.getCalibratedK());
  Serial.print(",");
  Serial.print(sensor.getCalibratedL());
  Serial.print(",");

  Serial.print(sensor.getCalibratedR());
  Serial.print(",");
  Serial.print(sensor.getCalibratedS());
  Serial.print(",");
  Serial.print(sensor.getCalibratedT());
  Serial.print(",");
  Serial.print(sensor.getCalibratedU());
  Serial.print(",");
  Serial.print(sensor.getCalibratedV());
  Serial.print(",");
  Serial.print(sensor.getCalibratedW());
  Serial.print(",");

  Serial.println();

}

