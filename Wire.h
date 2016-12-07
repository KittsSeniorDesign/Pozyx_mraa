// make this implement the Wire.h functionalities, but with mraa in c++
#ifndef Wire_h
#define Wire_h

#define BUFFER_LENGTH 32

#include "mraa.hpp"

class Wire {
	mraa::I2c *i2c;
	uint8_t *rxBuffer;
	uint8_t rxBufferIndex;
	uint8_t rxBufferLength;
	uint8_t *txBuffer;
	uint8_t txBufferIndex;
	uint8_t txBufferLength;
	uint8_t transmitting;
public:
	void begin(void);
	void beginTransmission(uint8_t);
	void beginTransmission(int);
	uint8_t endTransmission(uint8_t);
	uint8_t endTransmission(void);
	size_t requestFrom(uint8_t, size_t, bool);
	uint8_t requestFrom(uint8_t, uint8_t);
	uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
	uint8_t requestFrom(int, int);
	uint8_t requestFrom(int, int, int);

	size_t write(uint8_t);
	size_t write(const uint8_t *, size_t);
	int available(void);
	int peek(void);
	int flush(void);
	int read(void);

	inline size_t write(unsigned long n) { return write((uint8_t)n); }
	inline size_t write(long n) { return write((uint8_t)n); }
	inline size_t write(unsigned int n) { return write((uint8_t)n); }
	inline size_t write(int n) { return write((uint8_t)n); }
};

#endif
