// Mandy.cpp
// Mauro tracker class
// MGP Jan 2016

#include "Arduino.h"
#include <Kinesense.h>


Kinesense::Kinesense(void){}

float Kinesense::uint32_reg_to_float (uint8_t *buf)
{
	union {
		uint32_t ui32;
		float f;
	} u;

	u.ui32 =     (((uint32_t)buf[0]) +
	(((uint32_t)buf[1]) <<  8) +
	(((uint32_t)buf[2]) << 16) +
	(((uint32_t)buf[3]) << 24));
	return u.f;
}



void Kinesense::float_to_bytes (float param_val, uint8_t *buf) {
	union {
		float f;
		uint8_t comp[sizeof(float)];
	} u;
	u.f = param_val;
	for (uint8_t i=0; i < sizeof(float); i++) {
		buf[i] = u.comp[i];
	}
	//Convert to LITTLE ENDIAN
	for (uint8_t i=0; i < sizeof(float); i++) {
		buf[i] = buf[(sizeof(float)-1) - i];
	}
}


void Kinesense::EM7180_set_gyro_FS (uint16_t gyro_fs) {
	uint8_t bytes[4], STAT;
	bytes[0] = gyro_fs & (0xFF);
	bytes[1] = (gyro_fs >> 8) & (0xFF);
	bytes[2] = 0x00;
	bytes[3] = 0x00;
	writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
	writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
	writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
	writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused
	writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
	writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
	STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
	while(!(STAT==0xCB)) {
		STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
	}
	writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
	writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

// I2C read/write functions for the MPU9250 and AK8963 sensors

void Kinesense::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t Kinesense::readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	//	Wire.requestFrom(address, 1);  // Read one byte from slave register address
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void Kinesense::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
	//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	//        Wire.requestFrom(address, count);  // Read bytes from slave register address
	Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
	while (Wire.available()) {
	dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}