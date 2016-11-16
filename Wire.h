// make this implement the Wire.h functionalities, but with mraa in c++
#include <mraa.h>

#ifndef TwoWire_h
#define TwoWire_h

#define BUFFER_LENGTH 32

class TwoWire {
	mraa::I2C *i2c;
	uint8_t *rxBuffer;
	int rxBufferIndex;
	int rxBufferLength;
public:
	void begin();
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
	int read(void);

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
	inline size_t write(int n) { return write((uint8_t)n); }
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TWOWIRE)
extern TwoWire Wire;
#endif

#endif