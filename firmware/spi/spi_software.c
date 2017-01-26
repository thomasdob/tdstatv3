#include "spi_software.h"

#define _XTAL_FREQ 48000000     // 48 MHz CPU clock frequency

void InitializeSPI()
{
	// Initialize the chip select lines as inactive
	CS1_LAT = HIGH;
	CS2_LAT = HIGH;
	// Configure the chip select lines as outputs
	CS1_DIR = OUTPUT;
	CS2_DIR = OUTPUT;
	// The clock line should be an output; initialize it to a low state
	CLOCK_DIR = OUTPUT;
	CLOCK_LAT = LOW;
	// Initialize the data lines as inputs
	DATA1_DIR = INPUT;
	DATA2_DIR = INPUT;
}

uint8_t MCP3550_Read(uint8_t* adc_data)
{
	uint8_t data_ready = 0;
	// Poll conversion status
	CS2_LAT = LOW;
	SPIDelay();
	if(!DATA1_PIN) // conversions are ready
	{
		Read2BytesSPI(adc_data,adc_data+3);
		Read2BytesSPI(adc_data+1,adc_data+4);
		Read2BytesSPI(adc_data+2,adc_data+5);
		data_ready = 1;
		// Initiate a new conversion
		CS2_LAT = HIGH;
		SPIDelay();
		CS2_LAT = LOW;
		SPIDelay();
	}
	CS2_LAT = HIGH;
	SPIDelay();
	return data_ready;
}

void DAC1220_Reset()
{
	CS1_LAT = LOW;
	SPIDelay();
	CLOCK_LAT = HIGH;
	__delay_us(264);
	CLOCK_LAT = LOW;
	SPIDelay();
	CLOCK_LAT = HIGH;
	__delay_us(570);
	CLOCK_LAT = LOW;
	SPIDelay();
	CLOCK_LAT = HIGH;
	__delay_us(903);
	CLOCK_LAT = LOW;
	SPIDelay();
	CS1_LAT = HIGH;
	SPIDelay();
}

void DAC1220_Write2Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2)
{
	CS1_LAT = LOW;
	SPIDelay();
	DATA1_DIR = OUTPUT;
	WriteByteSPI(32+address);
	WriteByteSPI(byte1);
	WriteByteSPI(byte2);
	DATA1_DIR = INPUT;
	CS1_LAT = HIGH;
	SPIDelay();
}

void DAC1220_Write3Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2, const uint8_t byte3)
{
	CS1_LAT = LOW;
	SPIDelay();
	DATA1_DIR = OUTPUT;
	WriteByteSPI(64+address);
	WriteByteSPI(byte1);
	WriteByteSPI(byte2);
	WriteByteSPI(byte3);
	DATA1_DIR = INPUT;
	CS1_LAT = HIGH;
	SPIDelay();
}

void DAC1220_Read2Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2)
{
	CS1_LAT = LOW;
	SPIDelay();
	DATA1_DIR = OUTPUT;
	WriteByteSPI(160+address);
	DATA1_DIR = INPUT;
	SPIDelay();
	*byte1 = ReadByteSPI();
	*byte2 = ReadByteSPI();
	CS1_LAT = HIGH;
	SPIDelay();
}

void DAC1220_Read3Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2, uint8_t* byte3)
{
	CS1_LAT = LOW;
	SPIDelay();
	DATA1_DIR = OUTPUT;
	WriteByteSPI(192+address);
	DATA1_DIR = INPUT;
	SPIDelay();
	*byte1 = ReadByteSPI();
	*byte2 = ReadByteSPI();
	*byte3 = ReadByteSPI();
	CS1_LAT = HIGH;
	SPIDelay();
}

void DAC1220_Init()
{
	DAC1220_Write2Bytes(4, 32, 160); // command register: 20-bit resolution; straight binary
	DAC1220_Write3Bytes(0, 128, 0, 0); // set midscale output
}

void DAC1220_SelfCal()
{
	DAC1220_Write2Bytes(4, 32, 161); // command register: 20-bit resolution; straight binary, self calibration mode
}

void Read2BytesSPI(uint8_t* data1_byte, uint8_t* data2_byte)
{
	*data1_byte = 0;      // data to be read in
	*data2_byte = 0;      // data to be read in
	uint8_t bit_counter = 8;     // set bit count for byte
	do
	{
		ClockPulse();            // generate a clock pulse
		*data1_byte <<= 1;        // shift composed byte by 1
		*data2_byte <<= 1;
		*data1_byte &= 0xFE;      // clear bit 0
		*data2_byte &= 0xFE;
		if(DATA1_PIN)            // is data line high
			*data1_byte |= 0x01;  // set bit 0 to logic 1
		if(DATA2_PIN)            // is data line high
			*data2_byte |= 0x01;  // set bit 0 to logic 1
	} while (--bit_counter);     // repeat until 8 bits have been acquired
}

uint8_t ReadByteSPI()
{
	uint8_t data_byte = 0;      // data to be read in
	uint8_t bit_counter = 8;     // set bit count for byte
	do
	{
		ClockPulse();            // generate a clock pulse
		data_byte <<= 1;         // shift composed byte by 1
		data_byte &= 0xFE;       // clear bit 0
		if(DATA1_PIN)            // is data line high
			data_byte |= 0x01;   // set bit 0 to logic 1
	} while (--bit_counter);     // repeat until 8 bits have been acquired
	return data_byte;
}

void WriteByteSPI(uint8_t data_byte)
{
	uint8_t bit_counter = 8;     // set bit count for byte
	do
	{
		DATA1_LAT = (data_byte&0x80)?HIGH:LOW;  // output most significant bit
		ClockPulse();                           // generate a clock pulse
		data_byte <<= 1;                        // shift byte to the left
	} while (--bit_counter);                    // repeat until 8 bits have been transmitted
}

void ClockPulse()
{
	// Generate clock pulse
	CLOCK_LAT = HIGH;
	SPIDelay();
	CLOCK_LAT = LOW;
	SPIDelay();
}

void SPIDelay()
{
	_delay(200); // delay of 100 instruction cycles (=17 us at Fosc=48 MHz)
}
