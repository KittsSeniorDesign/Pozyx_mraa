#include "Wire.h"
#include <iostream>

void Wire::begin(void) {
	// bus 6 because thats what it said on http://iotdk.intel.com/docs/master/mraa/edison.html in November 2016 when this file was written
	i2c = new mraa::I2c(6);
	txBuffer = (uint8_t *)malloc(sizeof(uint8_t)*BUFFER_LENGTH);
	txBufferIndex = 0;
	txBufferLength = 0;
	rxBuffer = (uint8_t *)malloc(sizeof(uint8_t)*BUFFER_LENGTH);
	rxBufferIndex = 0;
	rxBufferLength = 0;
}

void Wire::beginTransmission(uint8_t address) {
	transmitting = 1;
	i2c->address(address);
	//i2c->frequency(mraa::I2C_STD);
	txBufferIndex = 0;
	txBufferLength = 0;
}

void Wire::beginTransmission(int address) {
	beginTransmission((uint8_t)address);
}

uint8_t Wire::endTransmission(uint8_t sendStop) {
	int8_t ret = i2c->write(txBuffer, (int)txBufferLength);
	std::cout << "ret=" << (int)ret << std::endl;
	txBufferIndex = 0;
	txBufferLength = 0;
	transmitting = 0;
	return ret;
}

uint8_t Wire::endTransmission(void) {
	return endTransmission(true);
}

size_t Wire::requestFrom(uint8_t address, size_t size, bool sendStop) {
	if(size > BUFFER_LENGTH) {
		size = BUFFER_LENGTH;
	}
	size_t read = (i2c->read(rxBuffer, (int)size) == (int)size) ? size : 0;
	rxBufferIndex = 0;
	rxBufferLength = read;
	return read;
}

uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop){
  return requestFrom(address, static_cast<size_t>(quantity), static_cast<bool>(sendStop));
}

uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity){
  return requestFrom(address, static_cast<size_t>(quantity), true);
}

uint8_t Wire::requestFrom(int address, int quantity){
  return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), true);
}

uint8_t Wire::requestFrom(int address, int quantity, int sendStop){
  return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), static_cast<bool>(sendStop));
}

size_t Wire::write(uint8_t data) {
	if(transmitting) {
		if(txBufferLength >= BUFFER_LENGTH){
			return 0;
		}
		txBuffer[txBufferIndex] = data;
		++txBufferIndex;
		txBufferLength = txBufferIndex;
	} else{
	// i2c_slave_transmit(data, quantity);
	}
	return 1;
}

size_t Wire::write(const uint8_t *data, size_t quantity) {
	if(transmitting){
		for(size_t i = 0; i < quantity; ++i){
			if(!write(data[i])) return i;
		}
	}else{
	// i2c_slave_transmit(data, quantity);
	}
	return quantity;
}

int Wire::available(void) {
	return rxBufferLength-rxBufferIndex;
}

int Wire::peek(void){
  int value = -1;
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }
  return value;
}

void Wire::flush(void){
  rxBufferIndex = 0;
  rxBufferLength = 0;
  txBufferIndex = 0;
  txBufferLength = 0;
}

int Wire::read(void) {
	int value = -1;
	if(rxBufferIndex < rxBufferLength){
		value = rxBuffer[rxBufferIndex];
		++rxBufferIndex;
	}
	return value;
}
