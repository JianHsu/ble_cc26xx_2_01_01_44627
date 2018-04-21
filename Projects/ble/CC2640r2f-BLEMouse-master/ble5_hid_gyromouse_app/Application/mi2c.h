#include <stdbool.h>
#include <stdint.h>

int8_t mI2C_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
int8_t mI2C_readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data);
int8_t mI2C_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
int8_t mI2C_readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data);
int8_t mI2C_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
int8_t mI2C_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data);
int8_t mI2C_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t mI2C_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

bool mI2C_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
bool mI2C_writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
bool mI2C_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
bool mI2C_writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
bool mI2C_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
bool mI2C_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
bool mI2C_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
bool mI2C_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);
