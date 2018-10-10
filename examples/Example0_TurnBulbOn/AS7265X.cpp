#include "AS7265X.h"
#include "Arduino.h"
//Sets up the sensor for constant read
//Returns the sensor version (AS7262 or AS7263)

AS7265X::AS7265X()
{
	
}

#define kevin
#ifdef kevin
void AS7265X::begin(TwoWire &wirePort)
{
	_i2cPort = &wirePort;
	_i2cPort->begin();
//	Serial.begin(115200); TODO: remove prints from this library
// there are no given default values for what the hw version should be so we can't check on that. 
// //	_sensorVersion = virtualReadRegister(AS7265X_HW_VERSION);


	setBulbCurrentAS72651(AS7265X_LED_CURRENT_LIMIT_12_5MA); //Set to 12.5mA (minimum)
//	setBulbCurrentAS72652(AS7265X_LED_CURRENT_LIMIT_12_5MA); //Set to 12.5mA (minimum)
//	setBulbCurrentAS72653(AS7265X_LED_CURRENT_LIMIT_12_5MA); //Set to 12.5mA (minimum)
	
	
	disableBulbAS72651();
//	disableBulbAS72652();
//	disableBulbAS72653();

	setIndicatorCurrent(AS7265X_INDICATOR_CURRENT_LIMIT_8MA); //Set to 8mA (maximum)
	disableIndicator(); //Turn off lights to save power

	setIntegrationTime(50); //50 * 2.8ms = 140ms. 0 to 255 is valid.
							//If you use Mode 2 or 3 (all the colors) then integration time is double. 140*2 = 280ms between readings.

	setGain(3); //Set gain to 64x

	setMeasurementMode(3); //One-shot reading of VBGYOR

	//TODO move this to the application code
/*	if (_sensorVersion == 0)
	{
		Serial.println("Sensor failed to respond. Check wiring.");
		while (1); //Freeze!
	}
*/
}
#endif

//Sets the measurement mode
//Mode 0: Continuous reading of STUV (AS72651 NIR), GHKI (AS72652 VISIBLE), ABEC (AS72653 UV)
//Mode 1: Continuous reading of RTUW (AS72651 NIR), GHJL (AS72652 VISIBLE), FABD (AS72653 UV)
//Mode 2: Continuous reading of STUVRW (AS72651 NIR), GHKIJL (AS72652 VISIBLE), ABCDEF (AS72653 UV)




#ifdef developing
//Prints all measurements
void AS7265X::printMeasurements()
{

	if (_sensorVersion == SENSORTYPE_AS7262)
	{
		//Visible readings
		Serial.print(" Reading: V[");
		Serial.print(getCalibratedViolet(), 2);
		Serial.print("] B[");
		Serial.print(getCalibratedBlue(), 2);
		Serial.print("] G[");
		Serial.print(getCalibratedGreen(), 2);
		Serial.print("] Y[");
		Serial.print(getCalibratedYellow(), 2);
		Serial.print("] O[");
		Serial.print(getCalibratedOrange(), 2);
		Serial.print("] R[");
		Serial.print(getCalibratedRed(), 2);
	}
	else if (_sensorVersion == SENSORTYPE_AS7263)
	{
		//Near IR readings
		Serial.print(" Reading: R[");
		Serial.print(getCalibratedR(), 2);
		Serial.print("] S[");
		Serial.print(getCalibratedS(), 2);
		Serial.print("] T[");
		Serial.print(getCalibratedT(), 2);
		Serial.print("] U[");
		Serial.print(getCalibratedU(), 2);
		Serial.print("] V[");
		Serial.print(getCalibratedV(), 2);
		Serial.print("] W[");
		Serial.print(getCalibratedW(), 2);
	}

	Serial.println();
}

void AS7265X::printUncalibratedMeasurements()
{

	if (_sensorVersion == SENSORTYPE_AS7262)
	{
		//Visible readings
		Serial.print(" Reading: V[");
		Serial.print(getViolet());
		Serial.print("] B[");
		Serial.print(getBlue());
		Serial.print("] G[");
		Serial.print(getGreen());
		Serial.print("] Y[");
		Serial.print(getYellow());
		Serial.print("] O[");
		Serial.print(getOrange());
		Serial.print("] R[");
		Serial.print(getRed());
	}
	else if (_sensorVersion == SENSORTYPE_AS7263)
	{
		//Near IR readings
		Serial.print(" Reading: R[");
		Serial.print(getR());
		Serial.print("] S[");
		Serial.print(getS());
		Serial.print("] T[");
		Serial.print(getT());
		Serial.print("] U[");
		Serial.print(getU());
		Serial.print("] V[");
		Serial.print(getV());
		Serial.print("] W[");
		Serial.print(getW());
	}

	Serial.println();
}

//Tells IC to take all channel measurements and polls for data ready flag
void AS7265X::takeMeasurements()
{
	setMeasurementMode(3);

	//Wait for data to be ready
	while (dataAvailable() == false) delay(POLLING_DELAY);

	//Readings can now be accessed via getViolet(), getBlue(), etc
}

//Turns on all bulbs, takes measurements of all channels, turns off all bulbs
void AS7265X::takeMeasurementsWithBulb()
{
	/* TODO: so takeMeasurements takes all channels. but how do we cycle the led's on?
	 *	I would think we would want to turn on the VISIBLE, measure, UV - measure, then NIR measure. 
	 * 	I guess that doesn't make sense because in all modes of operation multiple channels are being set.
	 * 	so maybe we turn on all led's then take a measurement. after all this is a spectral analysis 
	 * 	and the light will def be separated.
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
int AS7265X::getViolet() { return(getChannel(AS7262_V)); }
int AS7265X::getBlue() { return(getChannel(AS7262_B)); }
int AS7265X::getGreen() { return(getChannel(AS7262_G)); }
int AS7265X::getYellow() { return(getChannel(AS7262_Y)); }
int AS7265X::getOrange() { return(getChannel(AS7262_O)); }
int AS7265X::getRed() { return(getChannel(AS7262_R)); }

//Get the various NIR readings
int AS7265X::getR() { return(getChannel(AS7263_R)); }
int AS7265X::getS() { return(getChannel(AS7263_S)); }
int AS7265X::getT() { return(getChannel(AS7263_T)); }
int AS7265X::getU() { return(getChannel(AS7263_U)); }
int AS7265X::getV() { return(getChannel(AS7263_V)); }
int AS7265X::getW() { return(getChannel(AS7263_W)); }

//A the 16-bit value stored in a given channel registerReturns 
int AS7265X::getChannel(byte channelRegister, byte device)
{
	selectDevice(device);
	int colorData = virtualReadRegister(channelRegister) << 8; //High byte
	colorData |= virtualReadRegister(channelRegister + 1); //Low byte
	return(colorData);
}

//Returns the various calibration data
float AS7265X::getCalibratedViolet() { return(getCalibratedValue(AS7262_V_CAL)); }
float AS7265X::getCalibratedBlue() { return(getCalibratedValue(AS7262_B_CAL)); }
float AS7265X::getCalibratedGreen() { return(getCalibratedValue(AS7262_G_CAL)); }
float AS7265X::getCalibratedYellow() { return(getCalibratedValue(AS7262_Y_CAL)); }
float AS7265X::getCalibratedOrange() { return(getCalibratedValue(AS7262_O_CAL)); }
float AS7265X::getCalibratedRed() { return(getCalibratedValue(AS7262_R_CAL)); }

float AS7265X::getCalibratedR() { return(getCalibratedValue(AS7263_R_CAL)); }
float AS7265X::getCalibratedS() { return(getCalibratedValue(AS7263_S_CAL)); }
float AS7265X::getCalibratedT() { return(getCalibratedValue(AS7263_T_CAL)); }
float AS7265X::getCalibratedU() { return(getCalibratedValue(AS7263_U_CAL)); }
float AS7265X::getCalibratedV() { return(getCalibratedValue(AS7263_V_CAL)); }
float AS7265X::getCalibratedW() { return(getCalibratedValue(AS7263_W_CAL)); }

//Given an address, read four bytes and return the floating point calibrated value
float AS7265X::getCalibratedValue(byte calAddress)
{
	byte b0, b1, b2, b3;
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










#endif






/* stuff below this line has been looked at and should be ready for testing */
/* ======================================================================== */


//Mode 3: One-shot reading of all channels
void AS7265X::setMeasurementMode(byte mode)
{
	if (mode > 0b11) mode = 0b11;

	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_CONFIG); //Read
	value &= 0b11110011; //Clear BANK bits
	value |= (mode << 2); //Set BANK bits with user's choice
	virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Sets the gain value
//Gain 0: 1x (power-on default)
//Gain 1: 3.7x
//Gain 2: 16x
//Gain 3: 64x
void AS7265X::setGain(byte gain)
{
	if (gain > 0b11) gain = 0b11;

	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_CONFIG); //Read
	value &= 0b11001111; //Clear GAIN bits
	value |= (gain << 4); //Set GAIN bits with user's choice
	virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Sets the integration value
//Give this function a byte from 0 to 255.
//Time will be 2.8ms * [integration value]
void AS7265X::setIntegrationTime(byte integrationValue)
{
	virtualWriteRegister(AS7265X_INTERGRATION_TIME, integrationValue); //Write
}

void AS7265X::enableInterrupt()
{
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_CONFIG); //Read
	value |= (1<<6); //Set INT bit
	virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Disables the interrupt pin
void AS7265X::disableInterrupt()
{
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_CONFIG); //Read
	value &= ~(1<<6); //Clear INT bit
	virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Checks to see if DRDY flag is set in the control setup register
boolean AS7265X::dataAvailable()
{
	byte value = virtualReadRegister(AS7265X_CONFIG);
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
void AS7265X::enableBulb(byte device)
{
	selectDevice(device);
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_LED_CONTROL);
	value |= (1 << 3); //Set the bit
	virtualWriteRegister(AS7265X_LED_CONTROL, value);
}

//Disable the onboard 5700k or external incandescent bulb
void AS7265X::disableBulb(byte device)
{
	//TODO: make sure valid device set?
	selectDevice(device);
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_LED_CONTROL);
	value &= ~(1 << 3); //Clear the bit
	virtualWriteRegister(AS7265X_LED_CONTROL, value);
}

//Set the current limit of bulb/LED.
//Current 0: 12.5mA
//Current 1: 25mA
//Current 2: 50mA
//Current 3: 100mA
void AS7265X::setBulbCurrent(byte current, byte device)
{
	if (current > 0b11) current = 0b11; //Limit to two bits
//Select which device to configure
// bits 5/6 are read only, a direct write yields the desired outcome.
	virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, device);
	
// set the current	
	value = virtualReadRegister(AS7265X_LED_CONFIG); //Read
	value &= 0b11001111; //Clear ICL_DRV bits
	value |= (current << 4); //Set ICL_DRV bits with user's choice
	virtualWriteRegister(AS7265X_LED_CONFIG, value); //Write
}

void AS7265X::selectDevice(byte device){
	//Set the bits 0:1, because bits 5 and 6 are read only, a direct write yields the desired outcome.
	virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, device);
}

//Enable the onboard indicator LED
void AS7265X::enableIndicator()
{
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_LED_CONFIG);
	value |= (1 << 0); //Set the bit
	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}

//Disable the onboard indicator LED
void AS7265X::disableIndicator()
{
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_LED_CONFIG);
	value &= ~(1 << 0); //Clear the bit
	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}

//Set the current limit of onboard LED. Default is max 8mA = 0b11.
void AS7265X::setIndicatorCurrent(byte current)
{
	if (current > 0b11) current = 0b11;
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_LED_CONFIG); //Read
	value &= 0b11111001; //Clear ICL_IND bits
	value |= (current << 1); //Set ICL_IND bits with user's choice
	virtualWriteRegister(AS7265X_LED_CONFIG, value); //Write
}


//Returns the temperature in C
//Pretty inaccurate: +/-8.5C
byte AS7265X::getTemperature()
{
	return (virtualReadRegister(AS72653_DEVICE_TEMP));
}

//Does a soft reset
//Give sensor at least 1000ms to reset
void AS7265X::softReset()
{
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_CONFIG); //Read
	value |= (1 << 7); //Set RST bit, automatically cleared after reset
	virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Read a virtual register from the AS7265x
byte AS7265X::virtualReadRegister(byte virtualAddr)
{
	byte status;

	//Do a prelim check of the read register
	status = readRegister(AS7265X_STATUS_REG);
	if ((status & AS7265X_RX_VALID) != 0) //There is data to be read
	{
		//Serial.println("Premptive read");
		byte incoming = readRegister(AS7265X_READ_REG); //Read the byte but do nothing with it
	}

	//Wait for WRITE flag to clear
	while (1)
	{
		status = readRegister(AS7265X_STATUS_REG);
		if ((status & AS7265X_TX_VALID) == 0) break; // If TX bit is clear, it is ok to write
		delay(POLLING_DELAY);
	}

	// Send the virtual register address (bit 7 should be 0 to indicate we are reading a register).
	writeRegister(AS7265X_WRITE_REG, virtualAddr);

	//Wait for READ flag to be set
	while (1)
	{
		status = readRegister(AS7265X_STATUS_REG);
		if ((status & AS7265X_RX_VALID) != 0) break; // Read data is ready.
		delay(POLLING_DELAY);
	}

	byte incoming = readRegister(AS7265X_READ_REG);
	return (incoming);
}

//Write to a virtual register in the AS726x
void AS7265X::virtualWriteRegister(byte virtualAddr, byte dataToWrite)
{
	byte status;

	//Wait for WRITE register to be empty
	while (1)
	{
		status = readRegister(AS7265X_STATUS_REG);
		if ((status & AS7265X_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
		delay(POLLING_DELAY);
	}

	// Send the virtual register address (setting bit 7 to indicate we are writing to a register).
	writeRegister(AS7265X_WRITE_REG, (virtualAddr | 0x80)); // (1<<7) because 7th bit

	//Wait for WRITE register to be empty
	while (1)
	{
		status = readRegister(AS7265X_STATUS_REG);
		if ((status & AS7265X_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
		delay(POLLING_DELAY);
	}

	// Send the data to complete the operation.
	writeRegister(AS7265X_WRITE_REG, dataToWrite);
}

//Reads from a give location from the AS726x
byte AS7265X::readRegister(byte addr)
{
	_i2cPort->beginTransmission(AS7265X_ADDR);
	_i2cPort->write(addr);
	_i2cPort->endTransmission();

	_i2cPort->requestFrom(AS7265X_ADDR, 1);
	if (_i2cPort->available()) {
		return (_i2cPort->read());
	}
	else {	//TODO return a number or something to let us know an error occured, do print
		//outside of the function
		Serial.println("I2C Error");
		return (0xFF); //Error
	}
}

//Write a value to a spot in the AS726x
void AS7265X::writeRegister(byte addr, byte val)
{
	_i2cPort->beginTransmission(AS7265X_ADDR);
	_i2cPort->write(addr);
	_i2cPort->write(val);
	_i2cPort->endTransmission();
}
