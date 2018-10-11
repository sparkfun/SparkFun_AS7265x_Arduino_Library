#include "AS7265X.h"
#include "Arduino.h"
//Sets up the sensor for constant read
//Returns the sensor version (AS7262 or AS7263)

AS7265X::AS7265X()
{
	
}

void AS7265X::begin(TwoWire &wirePort)
{
	_i2cPort = &wirePort;
	_i2cPort->begin();
//	Serial.begin(115200); TODO: remove prints from this library
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

	//TODO move this to the application code
/*	if (_sensorVersion == 0)
	{
		Serial.println("Sensor failed to respond. Check wiring.");
		while (1); //Freeze!
	}
*/


	//TODO: return a boolean to force a spin when something goes wrong on the begin. 
}



//Prints all measurements
void AS7265X::printMeasurements()
{
/*
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
*/
}


void AS7265X::printUncalibratedMeasurements()
{
/*

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
*/
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
int AS7265X::getG() { return(getChannel(AS7265X_R_G_A, AS72652_VISIBLE)); }
int AS7265X::getH() { return(getChannel(AS7265X_S_H_B, AS72652_VISIBLE)); }
int AS7265X::getI() { return(getChannel(AS7265X_T_I_C, AS72652_VISIBLE)); }
int AS7265X::getJ() { return(getChannel(AS7265X_U_J_D, AS72652_VISIBLE)); }
int AS7265X::getK() { return(getChannel(AS7265X_V_K_E, AS72652_VISIBLE)); }
int AS7265X::getL() { return(getChannel(AS7265X_W_L_F, AS72652_VISIBLE)); }

//Get the various NIR readings
int AS7265X::getR() { return(getChannel(AS7265X_R_G_A, AS72651_NIR)); }
int AS7265X::getS() { return(getChannel(AS7265X_S_H_B, AS72651_NIR)); }
int AS7265X::getT() { return(getChannel(AS7265X_T_I_C, AS72651_NIR)); }
int AS7265X::getU() { return(getChannel(AS7265X_U_J_D, AS72651_NIR)); }
int AS7265X::getV() { return(getChannel(AS7265X_V_K_E, AS72651_NIR)); }
int AS7265X::getW() { return(getChannel(AS7265X_W_L_F, AS72651_NIR)); }

//Get the various UV readings
int AS7265X::getA() { return(getChannel(AS7265X_R_G_A, AS72653_UV)); }
int AS7265X::getB() { return(getChannel(AS7265X_S_H_B, AS72653_UV)); }
int AS7265X::getC() { return(getChannel(AS7265X_T_I_C, AS72653_UV)); }
int AS7265X::getD() { return(getChannel(AS7265X_U_J_D, AS72653_UV)); }
int AS7265X::getE() { return(getChannel(AS7265X_V_K_E, AS72653_UV)); }
int AS7265X::getF() { return(getChannel(AS7265X_W_L_F, AS72653_UV)); }


//A the 16-bit value stored in a given channel registerReturns 
int AS7265X::getChannel(byte channelRegister, byte device)
{
	selectDevice(device);
	int colorData = virtualReadRegister(channelRegister) << 8; //High byte
	colorData |= virtualReadRegister(channelRegister + 1); //Low byte
	return(colorData);
}


//Returns the various calibration data
float AS7265X::getCalibratedA() { return(getCalibratedValue(AS7265X_R_G_A_CAL)); }
float AS7265X::getCalibratedB() { return(getCalibratedValue(AS7265X_S_H_B_CAL)); }
float AS7265X::getCalibratedC() { return(getCalibratedValue(AS7265X_T_I_C_CAL)); }
float AS7265X::getCalibratedD() { return(getCalibratedValue(AS7265X_U_J_D_CAL)); }
float AS7265X::getCalibratedE() { return(getCalibratedValue(AS7265X_V_K_E_CAL)); }
float AS7265X::getCalibratedF() { return(getCalibratedValue(AS7265X_W_L_F_CAL)); }


//Returns the various calibration data
float AS7265X::getCalibratedG() { return(getCalibratedValue(AS7265X_R_G_A_CAL)); }
float AS7265X::getCalibratedH() { return(getCalibratedValue(AS7265X_S_H_B_CAL)); }
float AS7265X::getCalibratedI() { return(getCalibratedValue(AS7265X_T_I_C_CAL)); }
float AS7265X::getCalibratedJ() { return(getCalibratedValue(AS7265X_U_J_D_CAL)); }
float AS7265X::getCalibratedK() { return(getCalibratedValue(AS7265X_V_K_E_CAL)); }
float AS7265X::getCalibratedL() { return(getCalibratedValue(AS7265X_W_L_F_CAL)); }

float AS7265X::getCalibratedR() { return(getCalibratedValue(AS7265X_R_G_A_CAL)); }
float AS7265X::getCalibratedS() { return(getCalibratedValue(AS7265X_S_H_B_CAL)); }
float AS7265X::getCalibratedT() { return(getCalibratedValue(AS7265X_T_I_C_CAL)); }
float AS7265X::getCalibratedU() { return(getCalibratedValue(AS7265X_U_J_D_CAL)); }
float AS7265X::getCalibratedV() { return(getCalibratedValue(AS7265X_V_K_E_CAL)); }
float AS7265X::getCalibratedW() { return(getCalibratedValue(AS7265X_W_L_F_CAL)); }

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
	byte value = virtualReadRegister(AS7265X_LED_CONFIG);
	value |= (1 << 3); //Set the bit
	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}

//Disable the onboard 5700k or external incandescent bulb
void AS7265X::disableBulb(byte device)
{
	//TODO: make sure valid device set?
	selectDevice(device);
	//Read, mask/set, write
	byte value = virtualReadRegister(AS7265X_LED_CONFIG);
	value &= ~(1 << 3); //Clear the bit
	virtualWriteRegister(AS7265X_LED_CONFIG, value);
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
	byte value = virtualReadRegister(AS7265X_LED_CONFIG); //Read
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
// all temps can be read but this is the master
// TODO: make accessable for all temperatures
//Pretty inaccurate: +/-8.5C
byte AS7265X::getTemperature()
{
	return (virtualReadRegister(AS72651_DEVICE_TEMP));
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
