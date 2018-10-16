/*
  This is a library written for the Ublox NEO-M8P-2
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14586

  Written by Nathan Seidle @ SparkFun Electronics, September 6th, 2018

  The NEO-M8P-2 is a powerful GPS receiver capable of calculating correction data
  to achieve 2cm accuracy.

  This library handles the configuration of 'survey-in', RTCM messages, and to output
  the RTCM messages to the user's selected stream

  https://github.com/sparkfun/SparkFun_RTK_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_AS7265X.h"
#include "Arduino.h"

//Constructor
AS7265X::AS7265X()
{

}

//Initializes the sensor with basic settings
//Returns false if sensor is not detected
boolean AS7265X::begin(TwoWire &wirePort)
{
  _i2cPort = &wirePort;
  _i2cPort->begin(); //This reset any setClock() the user may have done

  if (isConnected() == false) return (false); //Check for sensor presence

  //Check to see if both slaves are detected
  uint8_t value = virtualReadRegister(AS7265X_DEV_SELECT_CONTROL);
  if (value & 0b00110000 == 0b00110000) return (false); //These bits should be set. Slaves not detected!

  // there are no given default values for what the hw version should be so we can't check on that.
  // //	_sensorVersion = virtualReadRegister(AS7265X_HW_VERSION);

  setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS72651_NIR);
  setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS72652_VISIBLE);
  setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS72653_UV);

  disableBulb(AS72651_NIR);
  disableBulb(AS72652_VISIBLE);
  disableBulb(AS72653_UV);

  setIndicatorCurrent(AS7265X_INDICATOR_CURRENT_LIMIT_8MA); //Set to 8mA (maximum)
  disableIndicator(); //Turn off lights to save power

  setIntegrationTime(50); //50 * 2.8ms = 140ms. 0 to 255 is valid.
  //If you use Mode 2 or 3 (all the colors) then integration time is double. 140*2 = 280ms between readings.

  setGain(3); //Set gain to 64x

  setMeasurementMode(3); //One-shot reading of VBGYOR

  return (true); //We're all setup!
}

//Returns true if I2C device ack's
boolean AS7265X::isConnected()
{
  _i2cPort->beginTransmission((uint8_t)AS7265X_ADDR);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}



//Tells IC to take all channel measurements and polls for data ready flag
void AS7265X::takeMeasurements()
{
  setMeasurementMode(3); //Set mode to all 6-channels, one-shot

  //Wait for data to be ready
  while (dataAvailable() == false) delay(AS7265X_POLLING_DELAY);

  //Readings can now be accessed via getViolet(), getBlue(), etc
}

//Turns on all bulbs, takes measurements of all channels, turns off all bulbs
void AS7265X::takeMeasurementsWithBulb()
{
  /* TODO: so takeMeasurements takes all channels. but how do we cycle the led's on?
   	I would think we would want to turn on the VISIBLE, measure, UV - measure, then NIR measure.
    	I guess that doesn't make sense because in all modes of operation multiple channels are being set.
    	so maybe we turn on all led's then take a measurement. after all this is a spectral analysis
    	and the light will def be separated.
   * */

  enableBulb(AS72651_NIR);
  enableBulb(AS72652_VISIBLE);
  enableBulb(AS72653_UV);
  //TODO: turn on 1 bulb and loop. or turn on all bulbs?

  takeMeasurements();

  disableBulb(AS72651_NIR); //Turn off bulb to avoid heating sensor
  disableBulb(AS72652_VISIBLE);
  disableBulb(AS72653_UV);
}

//Get the various color readings
uint16_t AS7265X::getG() {
  return (getChannel(AS7265X_R_G_A, AS72652_VISIBLE));
}
uint16_t AS7265X::getH() {
  return (getChannel(AS7265X_S_H_B, AS72652_VISIBLE));
}
uint16_t AS7265X::getI() {
  return (getChannel(AS7265X_T_I_C, AS72652_VISIBLE));
}
uint16_t AS7265X::getJ() {
  return (getChannel(AS7265X_U_J_D, AS72652_VISIBLE));
}
uint16_t AS7265X::getK() {
  return (getChannel(AS7265X_V_K_E, AS72652_VISIBLE));
}
uint16_t AS7265X::getL() {
  return (getChannel(AS7265X_W_L_F, AS72652_VISIBLE));
}

//Get the various NIR readings
uint16_t AS7265X::getR() {
  return (getChannel(AS7265X_R_G_A, AS72651_NIR));
}
uint16_t AS7265X::getS() {
  return (getChannel(AS7265X_S_H_B, AS72651_NIR));
}
uint16_t AS7265X::getT() {
  return (getChannel(AS7265X_T_I_C, AS72651_NIR));
}
uint16_t AS7265X::getU() {
  return (getChannel(AS7265X_U_J_D, AS72651_NIR));
}
uint16_t AS7265X::getV() {
  return (getChannel(AS7265X_V_K_E, AS72651_NIR));
}
uint16_t AS7265X::getW() {
  return (getChannel(AS7265X_W_L_F, AS72651_NIR));
}

//Get the various UV readings
uint16_t AS7265X::getA() {
  return (getChannel(AS7265X_R_G_A, AS72653_UV));
}
uint16_t AS7265X::getB() {
  return (getChannel(AS7265X_S_H_B, AS72653_UV));
}
uint16_t AS7265X::getC() {
  return (getChannel(AS7265X_T_I_C, AS72653_UV));
}
uint16_t AS7265X::getD() {
  return (getChannel(AS7265X_U_J_D, AS72653_UV));
}
uint16_t AS7265X::getE() {
  return (getChannel(AS7265X_V_K_E, AS72653_UV));
}
uint16_t AS7265X::getF() {
  return (getChannel(AS7265X_W_L_F, AS72653_UV));
}

//A the 16-bit value stored in a given channel registerReturns
uint16_t AS7265X::getChannel(uint8_t channelRegister, uint8_t device)
{
  selectDevice(device);
  uint16_t colorData = virtualReadRegister(channelRegister) << 8; //High uint8_t
  colorData |= virtualReadRegister(channelRegister + 1); //Low uint8_t
  return (colorData);
}

//Returns the various calibration data
float AS7265X::getCalibratedA() {
  return (getCalibratedValue(AS7265X_R_G_A_CAL));
}
float AS7265X::getCalibratedB() {
  return (getCalibratedValue(AS7265X_S_H_B_CAL));
}
float AS7265X::getCalibratedC() {
  return (getCalibratedValue(AS7265X_T_I_C_CAL));
}
float AS7265X::getCalibratedD() {
  return (getCalibratedValue(AS7265X_U_J_D_CAL));
}
float AS7265X::getCalibratedE() {
  return (getCalibratedValue(AS7265X_V_K_E_CAL));
}
float AS7265X::getCalibratedF() {
  return (getCalibratedValue(AS7265X_W_L_F_CAL));
}


//Returns the various calibration data
float AS7265X::getCalibratedG() {
  return (getCalibratedValue(AS7265X_R_G_A_CAL));
}
float AS7265X::getCalibratedH() {
  return (getCalibratedValue(AS7265X_S_H_B_CAL));
}
float AS7265X::getCalibratedI() {
  return (getCalibratedValue(AS7265X_T_I_C_CAL));
}
float AS7265X::getCalibratedJ() {
  return (getCalibratedValue(AS7265X_U_J_D_CAL));
}
float AS7265X::getCalibratedK() {
  return (getCalibratedValue(AS7265X_V_K_E_CAL));
}
float AS7265X::getCalibratedL() {
  return (getCalibratedValue(AS7265X_W_L_F_CAL));
}

float AS7265X::getCalibratedR() {
  return (getCalibratedValue(AS7265X_R_G_A_CAL));
}
float AS7265X::getCalibratedS() {
  return (getCalibratedValue(AS7265X_S_H_B_CAL));
}
float AS7265X::getCalibratedT() {
  return (getCalibratedValue(AS7265X_T_I_C_CAL));
}
float AS7265X::getCalibratedU() {
  return (getCalibratedValue(AS7265X_U_J_D_CAL));
}
float AS7265X::getCalibratedV() {
  return (getCalibratedValue(AS7265X_V_K_E_CAL));
}
float AS7265X::getCalibratedW() {
  return (getCalibratedValue(AS7265X_W_L_F_CAL));
}

//Given an address, read four bytes and return the floating point calibrated value
float AS7265X::getCalibratedValue(uint8_t calAddress)
{
  uint8_t b0, b1, b2, b3;
  b0 = virtualReadRegister(calAddress + 0);
  b1 = virtualReadRegister(calAddress + 1);
  b2 = virtualReadRegister(calAddress + 2);
  b3 = virtualReadRegister(calAddress + 3);

  //Channel calibrated values are stored big-endian
  uint32_t calBytes = 0;
  calBytes |= ((uint32_t)b0 << (8 * 3));
  calBytes |= ((uint32_t)b1 << (8 * 2));
  calBytes |= ((uint32_t)b2 << (8 * 1));
  calBytes |= ((uint32_t)b3 << (8 * 0));

  return (convertBytesToFloat(calBytes));
}

//Mode 0: 4 channels out of 6 (see datasheet)
//Mode 1: Different 4 channels out of 6 (see datasheet)
//Mode 2: All 6 channels continuously
//Mode 3: One-shot reading of all channels
void AS7265X::setMeasurementMode(uint8_t mode)
{
  if (mode > 0b11) mode = 0b11; //Error check

  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
  value &= 0b11110011; //Clear BANK bits
  value |= (mode << 2); //Set BANK bits with user's choice
  virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Sets the gain value
//Gain 0: 1x (power-on default)
//Gain 1: 3.7x
//Gain 2: 16x
//Gain 3: 64x
void AS7265X::setGain(uint8_t gain)
{
  if (gain > 0b11) gain = 0b11;

  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
  value &= 0b11001111; //Clear GAIN bits
  value |= (gain << 4); //Set GAIN bits with user's choice
  virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Sets the integration value
//Give this function a byte from 0 to 255.
//Time will be 2.8ms * [integration value]
void AS7265X::setIntegrationTime(uint8_t integrationValue)
{
  virtualWriteRegister(AS7265X_INTERGRATION_TIME, integrationValue); //Write
}

void AS7265X::enableInterrupt()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
  value |= (1 << 6); //Set INT bit
  virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Disables the interrupt pin
void AS7265X::disableInterrupt()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
  value &= ~(1 << 6); //Clear INT bit
  virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Checks to see if DRDY flag is set in the control setup register
boolean AS7265X::dataAvailable()
{
  uint8_t value = virtualReadRegister(AS7265X_CONFIG);
  return (value & (1 << 1)); //Bit 1 is DATA_RDY
}

//Given 4 bytes returns the floating point value
float AS7265X::convertBytesToFloat(uint32_t myLong)
{
  float myFloat;
  memcpy(&myFloat, &myLong, 4); //Copy bytes into a float
  return (myFloat);
}

//Enable the onboard 5700k or external incandescent bulb
void AS7265X::enableBulb(uint8_t device)
{
  selectDevice(device);
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
  value |= (1 << 3); //Set the bit
  virtualWriteRegister(AS7265X_LED_CONFIG, value);
}

//Disable the onboard 5700k or external incandescent bulb
void AS7265X::disableBulb(uint8_t device)
{
  selectDevice(device);
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
  value &= ~(1 << 3); //Clear the bit
  virtualWriteRegister(AS7265X_LED_CONFIG, value);
}

//Set the current limit of bulb/LED.
//Current 0: 12.5mA
//Current 1: 25mA
//Current 2: 50mA
//Current 3: 100mA
void AS7265X::setBulbCurrent(uint8_t current, uint8_t device)
{
  if (current > 0b11) current = 0b11; //Limit to two bits
  //Select which device to configure
  // bits 5/6 are read only, a direct write yields the desired outcome.
  virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, device);

  // set the current
  uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG); //Read
  value &= 0b11001111; //Clear ICL_DRV bits
  value |= (current << 4); //Set ICL_DRV bits with user's choice
  virtualWriteRegister(AS7265X_LED_CONFIG, value); //Write
}

//As we read various registers we have to point at the master or first/second slave
void AS7265X::selectDevice(uint8_t device) {
  //Set the bits 0:1. Just overwrite whatever is there because masking in the correct value doesn't work.
  virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, device);

  //This fails
  //uint8_t value = virtualReadRegister(AS7265X_DEV_SELECT_CONTROL);
  //value &= 0b11111100; //Clear lower two bits
  //if(device < 3) value |= device; //Set the bits
  //virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, value);
}

//Enable the onboard indicator LED
void AS7265X::enableIndicator()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
  value |= (1 << 0); //Set the bit
  virtualWriteRegister(AS7265X_LED_CONFIG, value);
}

//Disable the onboard indicator LED
void AS7265X::disableIndicator()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
  value &= ~(1 << 0); //Clear the bit
  virtualWriteRegister(AS7265X_LED_CONFIG, value);
}

//Set the current limit of onboard LED. Default is max 8mA = 0b11.
void AS7265X::setIndicatorCurrent(uint8_t current)
{
  if (current > 0b11) current = 0b11;
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG); //Read
  value &= 0b11111001; //Clear ICL_IND bits
  value |= (current << 1); //Set ICL_IND bits with user's choice
  virtualWriteRegister(AS7265X_LED_CONFIG, value); //Write
}


//Returns the temperature of a given device in C
uint8_t AS7265X::getTemperature(uint8_t deviceNumber)
{
  selectDevice(deviceNumber);
  return (virtualReadRegister(AS72651_DEVICE_TEMP));
}

//Returns an average of all the sensor temps in C
float AS7265X::getTemperatureAverage()
{
  float average = 0;

  for(uint8_t x = 0 ; x < 3 ; x++)
    average += getTemperature(x);
  
  return (average/3);
}

//Does a soft reset
//Give sensor at least 1000ms to reset
void AS7265X::softReset()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
  value |= (1 << 7); //Set RST bit, automatically cleared after reset
  virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Read a virtual register from the AS7265x
uint8_t AS7265X::virtualReadRegister(uint8_t virtualAddr)
{
  uint8_t status;

  //Do a prelim check of the read register
  status = readRegister(AS7265X_STATUS_REG);
  if ((status & AS7265X_RX_VALID) != 0) //There is data to be read
  {
    uint8_t incoming = readRegister(AS7265X_READ_REG); //Read the byte but do nothing with it
  }

  //Wait for WRITE flag to clear
  while (1)
  {
    status = readRegister(AS7265X_STATUS_REG);
    if ((status & AS7265X_TX_VALID) == 0) break; // If TX bit is clear, it is ok to write
    delay(AS7265X_POLLING_DELAY);
  }

  // Send the virtual register address (bit 7 should be 0 to indicate we are reading a register).
  writeRegister(AS7265X_WRITE_REG, virtualAddr);

  //Wait for READ flag to be set
  while (1)
  {
    status = readRegister(AS7265X_STATUS_REG);
    if ((status & AS7265X_RX_VALID) != 0) break; // Read data is ready.
    delay(AS7265X_POLLING_DELAY);
  }

  uint8_t incoming = readRegister(AS7265X_READ_REG);
  return (incoming);
}

//Write to a virtual register in the AS726x
void AS7265X::virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite)
{
  uint8_t status;

  //Wait for WRITE register to be empty
  while (1)
  {
    status = readRegister(AS7265X_STATUS_REG);
    if ((status & AS7265X_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
    delay(AS7265X_POLLING_DELAY);
  }

  // Send the virtual register address (setting bit 7 to indicate we are writing to a register).
  writeRegister(AS7265X_WRITE_REG, (virtualAddr | 1<<7));

  //Wait for WRITE register to be empty
  while (1)
  {
    status = readRegister(AS7265X_STATUS_REG);
    if ((status & AS7265X_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
    delay(AS7265X_POLLING_DELAY);
  }

  // Send the data to complete the operation.
  writeRegister(AS7265X_WRITE_REG, dataToWrite);
}

//Reads from a give location from the AS726x
uint8_t AS7265X::readRegister(uint8_t addr)
{
  _i2cPort->beginTransmission(AS7265X_ADDR);
  _i2cPort->write(addr);
  if (_i2cPort->endTransmission() != 0)
  {
    Serial.println("No ack!");
    return (0); //Device failed to ack
  }

  _i2cPort->requestFrom((uint8_t)AS7265X_ADDR, (uint8_t)1);
  if (_i2cPort->available()) {
    return (_i2cPort->read());
  }

  Serial.println("No ack!");
  return (0); //Device failed to respond
}

//Write a value to a spot in the AS726x
boolean AS7265X::writeRegister(uint8_t addr, uint8_t val)
{
  _i2cPort->beginTransmission(AS7265X_ADDR);
  _i2cPort->write(addr);
  _i2cPort->write(val);
  if (_i2cPort->endTransmission() != 0)
  {
    Serial.println("No ack!");
    return (false); //Device failed to ack
  }

  return (true);
}
