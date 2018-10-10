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

	byte getTemperature(); // NIR

	void enableIndicator(); //only on the AS72651
	void disableIndicator();

	void enableBulb(byte device);
	void disableBulb(byte device);
		
	void setBulbCurrent(byte current, byte device);

	void setMeasurementMode(byte mode); //writes to 0x04

	void setGain(byte gain);

	void enableInterrupt();
	void disableInterrupt();
	
	void softReset();

	void setIndicatorCurrent(byte current);



	
	/*==============================================================================*/



	boolean dataAvailable();
	
	void enableBulb();
	void disableBulb();
	void setBulbCurrent(byte current);

	void setIntegrationTime(byte integrationValue);
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
	void selectDevice(byte device);
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

#define AS72651_NIR			0x00
#define AS72652_VISIBLE			0x01
#define AS72653_UV			0x02

#define SENSORTYPE_AS72652 0x3E  //TODO: Verify
#define SENSORTYPE_AS72653 0x3F

	//Register addresses
#define AS72651_HW_VERSION_HIGH		0x00
#define AS72651_HW_VERSION_LOW 		0x01


#define AS72651_FW_VERSION_MINOR	0x02
#define AS72651_FW_VERSION_MAJOR	0x03
#define AS7265X_CONFIG	 		0x04

#define AS7265X_INTERGRATION_TIME	0x05

#define AS72651_DEVICE_TEMP 		0x06

#define AS7265X_LED_CONFIG	 	0x07

//raw channel registers
#define AS7265X_R_G_A			0x08
#define AS7265X_S_H_B			0x0A
#define AS7265X_T_I_C			0x0C
#define AS7265X_U_J_D			0x0E
#define AS7265X_V_K_E			0x10
#define AS7265X_W_L_F			0x12

//calibrated channel registers
#define AS7265X_R_G_A_CAL		0x14
#define AS7265X_S_H_B_CAL		0x18
#define AS7265X_T_I_C_CAL		0x1C
#define AS7265X_U_J_D_CAL		0x20
#define AS7265X_V_K_E_CAL		0x24
#define AS7265X_W_L_F_CAL		0x28














#define AS7265X_FW_CONTROL		0x48
#define AS7265X_FW_HIGH			0x49
#define AS7265X_FW_LOw			0x4A
#define AS7265X_FW_PAYLOAD		0x4B
#define AS7265X_DEV_SELECT_CONTROL	0x4F

#define AS7265X_COEF_DATA_0		0x50
#define AS7265X_COEF_DATA_1		0x51
#define AS7265X_COEF_DATA_2		0x52
#define AS7265X_COEF_DATA_3		0x53
#define AS7265X_COEF_DATA_READ		0x54
#define AS7265X_COEF_DATA_WRITE		0x55




#define AS7265X_I2C_CAL_SEL		0x3F


	// AS72651 NIR Registers

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
