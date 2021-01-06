#include "E24.h"

#define E24_PAGE_WRITE_CYCLE	5
#define WRITE_BUFFER_LENGTH		BUFFER_LENGTH - 2 //sequential write will fail if using the full TWI BUFFER_LENGTH
#define READ_BUFFER_LENGTH		BUFFER_LENGTH

E24::E24(void)
: _deviceAddr(E24_DEFAULT_ADDR)
, _size(E24Size_t::E24_256K)
{
}

E24::E24(E24Size_t size, uint8_t deviceAddr = E24_DEFAULT_ADDR)
{
	_deviceAddr = deviceAddr;
	_size = size;
}

E24::~E24() {}

uint16_t E24::sequentialWrite(uint16_t addr, const uint8_t* data, uint8_t length)
{
	E24_YIELD
	uint8_t bSize = length;

	if (WRITE_BUFFER_LENGTH < length) bSize = WRITE_BUFFER_LENGTH;
	
	Wire.beginTransmission(_deviceAddr);
	Wire.write(highByte(addr));
	Wire.write(lowByte(addr));
	size_t w = Wire.write(data, bSize); //min(WRITE_BUFFER_LENGTH, length)
	
	Wire.endTransmission();

	//wait until the full page is being written
	delay(E24_PAGE_WRITE_CYCLE);

	return w;
}

uint16_t E24::sequentialRead(uint16_t addr, uint8_t* data, uint8_t length)
{
	E24_YIELD
	uint16_t offset = 0;
	uint8_t bSize = length;

	if (READ_BUFFER_LENGTH < length) bSize = READ_BUFFER_LENGTH;

	Wire.beginTransmission(_deviceAddr);
	Wire.write(highByte(addr));
	Wire.write(lowByte(addr));
	Wire.endTransmission();
	delay(E24_PAGE_WRITE_CYCLE);

	Wire.requestFrom(_deviceAddr, bSize); //min(READ_BUFFER_LENGTH, length)
	while(Wire.available()) { //while(Wire.available()) data[offset++] = Wire.read();
        	data[offset++] = Wire.read();
        	delay(E24_PAGE_WRITE_CYCLE);
            E24_YIELD;
		}

	return offset;
}

uint8_t E24::read()
{
	E24_YIELD
	Wire.beginTransmission(_deviceAddr);
	delay(E24_PAGE_WRITE_CYCLE);
	Wire.endTransmission();
	Wire.requestFrom(_deviceAddr, (uint8_t)1);
	delay(E24_PAGE_WRITE_CYCLE);
	
	return Wire.read();
}

uint8_t E24::read(uint16_t addr)
{
	E24_YIELD
	Wire.beginTransmission(_deviceAddr);
	Wire.write(highByte(addr));
	Wire.write(lowByte(addr));
	delay(E24_PAGE_WRITE_CYCLE);

	Wire.endTransmission();
	Wire.requestFrom(_deviceAddr, (uint8_t)1);
	delay(E24_PAGE_WRITE_CYCLE);

	return Wire.read();
}

uint16_t E24::read(uint16_t addr, uint8_t* data, uint16_t length)
{
	E24_YIELD
	uint8_t pageSize = E24_PAGE_SIZE(_size);
	uint8_t bSize = pageSize;
	uint8_t read = 0;
	uint16_t offset = 0;

	do {
		//avoid to overflow max read buffer size
		if (length < pageSize) bSize = length;

		read = sequentialRead(addr, data + offset, bSize);
		length -= read;
		addr += read;
		offset += read;

	} while (length > 0);

	return offset;
}

void E24::write(uint16_t addr, uint8_t data)
{
	E24_YIELD
	Wire.beginTransmission(_deviceAddr);
	Wire.write(highByte(addr));
	Wire.write(lowByte(addr));
	delay(E24_PAGE_WRITE_CYCLE);

	Wire.write(data);
	delay(E24_PAGE_WRITE_CYCLE);
	
	Wire.endTransmission();

	//wait until the full page is being written
	delay(E24_PAGE_WRITE_CYCLE);
}

uint16_t E24::write(uint16_t addr, const uint8_t* data, uint16_t length)
{
	E24_YIELD
	uint16_t endAddress = addr + length - 1;
	
	if (endAddress >  E24_MAX_ADDRESS(_size) || endAddress < addr) return -1; //endAddress < addr == overlap => > E24_MAX_ADDRESS for 512k chip

	uint8_t pageSize = E24_PAGE_SIZE(_size);
	uint8_t written = 0;
	uint16_t offset = 0;
	uint8_t bSize = 0;

	//compute the next buffer size using nextPageAddress - addr, only need first time
	bSize = ((addr + pageSize) & ~(pageSize - 1)) - addr;
	if (length < bSize) bSize = length;

	do {
		//avoid to overflow content length & max write buffer size
		if (length < pageSize) bSize = length;

		written = sequentialWrite(addr, data + offset, bSize);
		length -= written;
		addr += written;
		offset += written;
		bSize = pageSize;

	} while (length > 0);

	return offset;
}

//at the cost of the local buffer, this version allows to reuse the write implementation
//and can erase the whole chip in one call if needed.
uint16_t E24::fill(uint16_t addr, uint8_t data, uint16_t length) 
{
	E24_YIELD
	uint8_t buffer[WRITE_BUFFER_LENGTH];
	uint16_t written = 0;
	uint16_t offset = 0;
	uint8_t bSize = 0;

	memset(buffer, data, WRITE_BUFFER_LENGTH);
	bSize = WRITE_BUFFER_LENGTH;
	
	do {
		//bSize = min(WRITE_BUFFER_LENGTH, length);
		if  (length < bSize) bSize = length;

		written = write(addr, buffer, bSize);
		if(written == -1) break;

		length -= written,
		addr += written;
		offset += written;
		
	} while(length > 0);

	return offset;
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_EEPROM)
E24 EEPROM;
#endif
