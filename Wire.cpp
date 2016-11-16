#include "Wire.h"

TwoWire::TwoWire(){}

TwoWire::begin(void) {
	// possibly make the bus 1
	i2c = new mraa::I2c(0);
}

void TwoWire::beginTransmission(uint8_t address) {
	i2c->address(address);
	i2c->frequency(I2C_STD);
	rxBuffer = (uint8_t *)malloc(sizeof(uint8_t)*BUFFER_LENGTH);
	rxBufferIndex = 0;
	rxBufferLength = 0;
}

void TwoWire::beginTransmission(int address) {
	beginTransmission((uint8_t)address);
}

uint8_t TwoWire::endTransmission(uint8_t sendStop) {}

uint8_t TwoWrie::endTransmission(void) {
	return endTransmission(true);
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, bool sendStop) {
	if(size > BUFFER_LENGTH) {
		size = BUFFER_LENGTH;
	}
	size_t read = (i2c->read(rxBuffer, (int)size) == (int)size) ? size : 0;
	rxBufferIndex = 0;
	rxBufferLength = read;
	return read;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop){
  return requestFrom(address, static_cast<size_t>(quantity), static_cast<bool>(sendStop));
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity){
  return requestFrom(address, static_cast<size_t>(quantity), true);
}

uint8_t TwoWire::requestFrom(int address, int quantity){
  return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop){
  return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), static_cast<bool>(sendStop));
}

size_t TwoWire::write(uint8_t data) {
	i2c->writeByte(data);
}

size_t TwoWire::write(const uint8_t *data, size_t quantity) {
	i2c->write(data, (int) quantity);
	return quantity;
}

int TwoWire::available(void) {
	return rxBufferLength-rxBufferIndex;
}

int TwoWire::read(void) {
	int value = -1;
	if(rxBufferIndex < rxBufferLength){
		value = rxBuffer[rxBufferIndex];
		++rxBufferIndex;
	}
	return value;
}