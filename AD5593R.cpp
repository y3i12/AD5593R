//
//    FILE: AD5593R.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.0
//    DATE: 2024-01-30
// PURPOSE: Arduino library for AD5593R, I2C, 8 channel ADC / DAC / GPIO device.
//     URL: https://github.com/RobTillaart/AD5593R


#include "AD5593R.h"


//  CONFIG REGISTERS (aka pointer bytes)

#define AD5593_NOP                      0b00000000 // NOP. No operation.
#define AD5593_ADC_SEQ                  0b00000010 // ADC sequence register. Selects ADCs for conversion.
#define AD5593_GEN_CTRL_REG             0b00000011 // General-purpose control register. DAC and ADC control register.
#define AD5593_ADC_CONFIG               0b00000100 // ADC pin configuration. Selects which pins are ADC inputs.
#define AD5593_DAC_CONFIG               0b00000101 // DAC pin configuration. Selects which pins are DAC outputs.
#define AD5593_PULLDOWN_CONFIG          0b00000110 // Pull-down configuration. Selects which pins have an 85 kΩ pull-down resistor to GND.
#define AD5593_LDAC_MODE                0b00000111 // LDAC mode. Selects the operation of the load DAC.
#define AD5593_GPIO_CONFIG              0b00001000 // GPIO write configuration. Selects which pins are general-purpose outputs.
#define AD5593_GPIO_OUTPUT              0b00001001 // GPIO write data. Writes data to general-purpose outputs.
#define AD5593_GPIO_INPUT               0b00001010 // GPIO read configuration. Selects which pins are general-purpose inputs.
#define AD5593_POWERDOWN_REF_CTRL       0b00001011 // Power-down/reference control. Powers down the DACs and enables/disables the reference.
#define AD5593_GPIO_OPENDRAIN_CONFIG    0b00001100 // Open-drain configuration. Selects open-drain or push-pull for general-purpose outputs.
#define AD5593_IO_TS_CONFIG             0b00001101 // Three-state pins. Selects which pins are three-stated.
#define AD5593_SW_RESET                 0b00001111 // Software reset. Resets the AD5593R

//  IO REGISTERS
#define AD5593_DAC_WRITE(x)             (0x10 + (x))
#define AD5593_ADC_READ                 0x40
#define AD5593_DAC_READ(x)              (0x50 + (x))
#define AD5593_GPIO_READ                0x60
#define AD5593_GPIO_READ_CONFIG         0x70

// LDAC REGISTERS
#define AD5593_LDAC_INSTANT (0b00000000 | AD5593_LDAC_MODE) // Data written to an input register is immediately copied to a DAC register, and the DAC output updates.
#define AD5593_LDAC_LOAD    (0b10000000 | AD5593_LDAC_MODE) // Data written to an input register is not copied to a DAC register. The DAC output is not updated.
#define AD5593_LDAC_FLUSH   (0b01000000 | AD5593_LDAC_MODE) // Data in the input registers is copied to the corresponding DAC registers. When the data has been transferred, the DAC outputs are updated simultaneously.




AD5593R::AD5593R(const uint8_t deviceAddress, TwoWire *wire)
{
  _address    = deviceAddress;
  _wire       = wire;
  _error      = AD5593R_OK;
}


bool AD5593R::begin()
{
  if (! isConnected()) return false;
  return true;
}


bool AD5593R::isConnected()
{
  _wire->beginTransmission(_address);
  return ( _wire->endTransmission() == 0);
}


uint8_t AD5593R::getAddress()
{
  return _address;
}

////////////////////////////////////////////////////////////
//
//  MODE
//
int AD5593R::setADCmode(uint8_t bitMask)
{
  //  Page 25 / 32
  return writeRegister(AD5593_ADC_CONFIG, bitMask);
}

int AD5593R::setDACmode(uint8_t bitMask)
{
  //  Page 35
  return writeRegister(AD5593_DAC_CONFIG, bitMask);
}

int AD5593R::setINPUTmode(uint8_t bitMask)
{
  //  Page 26
  //  1's => INPUT
  return writeRegister(AD5593_GPIO_INPUT, bitMask);
}

int AD5593R::setOUTPUTmode(uint8_t bitMask)
{
  //  Page 26
  //  1's => OUTPUT
  return writeRegister(AD5593_GPIO_CONFIG, bitMask);

  //  not implemented yet (flag or 2nd bitmap?)
  //  GPIO_OPENDRAIN_CONFIG  Page 26/42
  //  set default values for output? write8(0x0000);
  //  IO_TS_CONFIG           Page 42  3e bitMask?
}

int AD5593R::setPULLDOWNmode(uint8_t bitMask)
{
  //  page 36
  return writeRegister(AD5593_PULLDOWN_CONFIG, bitMask);
}


////////////////////////////////////////////////////////////
//
//  DIGITAL
//
uint16_t AD5593R::write1(uint8_t pin, uint8_t value)
{
  if (pin > 7) return AD5593R_PIN_ERROR;
  uint8_t bitMask = readRegister(AD5593_GPIO_OUTPUT);
  if (value == LOW) bitMask &= ~(1 << pin);
  else              bitMask |= (1 << pin);
  return writeRegister(AD5593_GPIO_OUTPUT, bitMask);
}

uint16_t AD5593R::read1(uint8_t pin)
{
  if (pin > 7) return AD5593R_PIN_ERROR;
  uint8_t bitMask = readRegister(AD5593_GPIO_READ);
  return (bitMask >> pin) & 0x01;
}

uint16_t AD5593R::write8(uint8_t bitMask)
{
  return writeRegister(AD5593_GPIO_OUTPUT, bitMask);
}

uint16_t AD5593R::read8()
{
  return readRegister(AD5593_GPIO_READ);
}


////////////////////////////////////////////////////////////
//
//  ANALOG
//
uint16_t AD5593R::writeDAC(uint8_t pin, uint16_t value)
{
  if (pin > 7) return AD5593R_PIN_ERROR;

  value &= 0x0FFF;

  return writeRegister(AD5593_DAC_WRITE(pin), value);
}

void AD5593R::beginLDAC(uint8_t instant)
{
  _wire->beginTransmission(_address);
  _wire->write(instant ? AD5593_LDAC_INSTANT : AD5593_LDAC_LOAD);
}

uint16_t AD5593R::writeLDAC(uint8_t pin, uint16_t data )
{
  
  if (pin > 7) return AD5593R_PIN_ERROR;

  value &= 0x0FFF;

  _wire->write(AD5593_DAC_WRITE(pin));
  _wire->write(data >> 8);
  _wire->write(data & 0xFF);
}

uint16_t AD5593R::endLDAC()
{
  _wire->write(reg);
  
  _error = _wire->endTransmission();
  return _error;
}

uint16_t AD5593R::writeDAC(uint16_t v0, uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4, uint16_t v5, uint16_t v6, uint16_t v7) 
{
  beginLDAC();
  ldac(0, v0);
  ldac(1, v1);
  ldac(2, v2);
  ldac(3, v3);
  ldac(4, v4);
  ldac(5, v5);
  ldac(6, v6);
  ldac(7, v7);
  return endLDAC();
}

uint16_t AD5593R::readDAC(uint8_t pin)
{
  if (pin > 7) return AD5593R_PIN_ERROR;
  return readRegister(AD5593_DAC_READ(pin));
}

uint16_t AD5593R::readADC(uint8_t pin)
{
  if (pin > 7) return AD5593R_PIN_ERROR;
  //  add all to the sequence including temperature.
  //  0x0200 = REPeat bit
  //  0x0100 = TEMPerature include bit
  writeRegister(AD5593_ADC_SEQ, 0x0000 | (1 << pin));
  //  read one ADC conversion.
  return readRegister(AD5593_ADC_READ);
}


////////////////////////////////////////////////////////////
//
//  V-REFERENCE
//
int AD5593R::setExternalReference(bool flag)
{
  //  Page 40
  uint8_t bitMask = readRegister(AD5593_POWERDOWN_REF_CTRL);
  if (flag) bitMask &= ~(0x0200);
  else      bitMask |= (0x0200);
  return writeRegister(AD5593_POWERDOWN_REF_CTRL, bitMask);
}

int AD5593R::powerDown()
{
  //  Page 40
  uint8_t bitMask = readRegister(AD5593_POWERDOWN_REF_CTRL);
  bitMask |= (0x0400);
  return writeRegister(AD5593_POWERDOWN_REF_CTRL, bitMask);
}

int AD5593R::wakeUp()
{
  //  Page 40
  uint8_t bitMask = readRegister(AD5593_POWERDOWN_REF_CTRL);
  bitMask &= ~(0x0400);
  return writeRegister(AD5593_POWERDOWN_REF_CTRL, bitMask);
}


////////////////////////////////////////////////////////////
//
//  RESET
//
int AD5593R::reset()
{
  //  page 19
  return writeRegister(AD5593_SW_RESET, 0x0DAC);
}


////////////////////////////////////////////////////////////
//
//  TEMPERATURE
//
int AD5593R::getTemperature()
{
  //  see readADC
  //  0x0100 = TEMPerature include bit, no other ADC's
  writeRegister(AD5593_ADC_SEQ, 0x0100);
  //  read one ADC conversion.
  return readRegister(AD5593_ADC_READ);
}


////////////////////////////////////////////////////////////
//
//  PROTECTED
//
int AD5593R::writeRegister(uint8_t reg, uint16_t data)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(data >> 8);
  _wire->write(data & 0xFF);
  _error = _wire->endTransmission();
  return _error;
}


uint16_t AD5593R::readRegister(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _error = _wire->endTransmission();
  if (_wire->requestFrom(_address, (uint8_t)2) != 2)
  {
    _error = AD5593R_I2C_ERROR;
    return 0;
  }
  uint16_t data = _wire->read() << 8;
  data += _wire->read();
  return data;
}

//  -- END OF FILE --

