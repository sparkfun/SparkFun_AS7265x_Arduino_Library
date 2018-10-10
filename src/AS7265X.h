/*
 Name:		AS7265X.h
 Created:	7/11/2017 12:06:22 PM
 Author:	andrew.england
 Editor:	http://www.visualmicro.com
*/

#ifndef _AS7265X_h
#define _AS7265X_h
#include "Arduino.h"
#include "Wire.h"
class AS7265X {
public:
	AS7265X();
	void begin(TwoWire &wirePort = Wire);
	void takeMeasurements();
	void takeMeasurementsWithBulb();
	void printMeasurements();
	void printUncalibratedMeasurements();

/* Everything in here has been looked at already and should be ready for a test */
/*==============================================================================*/

	byte getTemperatureAS72651(); // NIR
	byte getTemperatureAS72652(); // Visible
	byte getTemperatureAS72653(); // UV

	void enableIndicator(); //only on the AS72651
	void disableIndicator();
	



		
	void enableBulbAS72651();
	void disableBulbAS72651();
	void setBulbCurrentAS72651(byte current);


	void enableBulbAS72652();
	void disableBulbAS72652();
	void setBulbCurrentAS72652(byte current);


	void enableBulbAS72653();
	void disableBulbAS72653();
	void setBulbCurrentAS72653(byte current);







/*==============================================================================*/



	void setMeasurementMode(byte mode);
	boolean dataAvailable();
	
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
	byte readRegister(byte addr);
private:
	TwoWire *_i2cPort;
	int getChannel(byte channelRegister);
	float getCalibratedValue(byte calAddress);
	float convertBytesToFloat(uint32_t myLong);
	boolean clearDataAvailable();
	byte virtualReadRegister(byte virtualAddr);
	void virtualWriteRegister(byte virtualAddr, byte dataToWrite);
	void writeRegister(byte addr, byte val);
	//:W
	//byte readRegister(byte addr);

#define AS7265X_ADDR 0x49 //7-bit unshifted default I2C Address

#define AS7265X_STATUS_REG		0x00
#define AS7265X_WRITE_REG		0X01
#define AS7265X_READ_REG		0x02

#define AS7265X_TX_VALID		0x02
#define AS7265X_RX_VALID		0x01

#define SENSORTYPE_AS72652 0x3E  //TODO: Verify
#define SENSORTYPE_AS72653 0x3F

	//Register addresses
#define AS72651_HW_Version 		0x00
#define AS72651_DEVICE_TYPE 		0x01

#define AS72652_HW_VERSION		0x02
#define AS72652_DEVICE_TYPE		0x03

#define AS72653_HW_VERSION		0x04
#define AS72653_HW_DEVICE_TYPE		0x05
#define AS72651_FW_VERSION_MINOR	0x06
#define AS72651_FW_VERSION_MAJOR	0x07


#define AS7265X_CONTROL_SETUP 		0x0C

#define AS7265X_INT_T	 		0x0F
#define AS72651_DEVICE_TEMP 		0x12
#define AS72652_DEVICE_TEMP 		0x13
#define AS72653_DEVICE_TEMP 		0x14

#define AS72651_LED_CONTROL_NIR 	0x15
#define AS72652_LED_CONTROL_VISIBLE 	0x16
#define AS72653_LED_CONTROL_UV 		0x17

#define AS7265X_I2C_CAL_SEL		0x3F


	// AS72651 NIR Registers
#define AS72652_CH_R		0x18
#define AS72652_CH_S		0x1A
#define AS72652_CH_T		0x1C
#define AS72652_CH_U		0x1E
#define AS72652_CH_V		0x20
#define AS72652_CH_W		0x22

#define AS72652_R_CAL		0x40 //I assume 0x40 is the address and 0x43 is the default value?
#define AS72652_S_CAL		0x44
#define AS72652_T_CAL		0x48
#define AS72652_U_CAL		0x4C
#define AS72652_V_CAL		0x50
#define AS72652_W_CAL		0x54

//AS72652 Visible Registers
#define AS72652_CH_G		0x24
#define AS72652_CH_H		0x26
#define AS72652_CH_I		0x28
#define AS72652_CH_J		0x2A
#define AS72652_CH_K		0x2c
#define AS72652_CH_L		0x2E

#define AS72652_G_CAL		0x40 //I assume 0x40 is the address and 0x43 is the default value?
#define AS72652_H_CAL		0x44
#define AS72652_I_CAL		0x48
#define AS72652_J_CAL		0x4C
#define AS72652_K_CAL		0x50
#define AS72652_L_CAL		0x54

	//AS72653 UV Registers
#define AS72653_CH_A		0x30
#define AS72653_CH_B		0x32
#define AS72653_CH_C		0x34
#define AS72653_CH_D		0x36
#define AS72653_CH_E		0x38
#define AS72653_CH_F		0x3A

#define AS72653_A_CAL		0x40
#define AS72653_B_CAL		0x44
#define AS72653_C_CAL		0x48
#define AS72653_D_CAL		0x4c
#define AS72653_E_cAL		0x50
#define AS72653_F_cAL		0x54

// THis is where I stopped. kevin
#define POLLING_DELAY 5 //Amount of ms to wait between checking for virtual register changes

#define AS7265X_LED_CURRENT_LIMIT_12_5MA	0b00
#define AS7265X_LED_CURRENT_LIMIT_25MA		0b01
#define AS7265X_LED_CURRENT_LIMIT_50MA		0b10
#define AS7265X_LED_CURRENT_LIMIT_100MA		0b11


#define AS7265X_INDICATOR_CURRENT_LIMIT_1MA		0b00
#define AS7265X_INDICATOR_CURRENT_LIMIT_2MA		0b01
#define AS7265X_INDICATOR_CURRENT_LIMIT_4MA		0b10
#define AS7265X_INDICATOR_CURRENT_LIMIT_8MA		0b11


	byte _sensorVersion = 0;
};

#endif
