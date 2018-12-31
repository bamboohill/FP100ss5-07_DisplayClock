/* PCF857x_ESP -- library for using the I2C-driven 8-pin GPIO-expander
   ORIGINAL AUTHOR: Rob Tillaart
   Library modified by WereCatf */

#include "pcf8574_esp.h"
#include <Wire.h>

PCF857x::PCF857x(uint8_t address, bool is8575)
{
  _address = address;
  _is8575 = is8575;
}

void PCF857x::begin(uint16_t defaultValues)
{
  if(_is8575) PCF857x::write16(defaultValues);
  else PCF857x::write8(defaultValues);
}

uint8_t PCF857x::read8()
{
  Wire.beginTransmission(_address);
  if(_is8575)
  {
    PCF857x::read16();
    return (uint8_t) _data;
  }

  if(Wire.requestFrom(_address, (uint8_t) 1) != 1)
  {
    _error = PCF857x_I2C_ERROR;
    return (uint8_t) _data;
  }
  _data = Wire.read();
  Wire.endTransmission();
  return _data;
}

uint16_t PCF857x::read16()
{
  Wire.beginTransmission(_address);
  if(!_is8575) return 0x00;

  if(Wire.requestFrom(_address, (uint8_t) 2) != 1)
  {
    _error = PCF857x_I2C_ERROR;
    return _data;
  }
  _data = 0;
  _data = Wire.read();
  _data |= Wire.read() << 8;
  Wire.endTransmission();
  return _data;
}

void PCF857x::resetInterruptPin()
{
  if(_is8575) PCF857x::read16();
  else PCF857x::read8();
}

void PCF857x::write8(uint8_t value)
{
  Wire.beginTransmission(_address);
  _pinModeMask &=0xff00;
  _pinModeMask |= value;
  _data = _pinModeMask;
  Wire.write((uint8_t) _data);
  if(_is8575) Wire.write((uint8_t) (_data >> 8));
  _error = Wire.endTransmission();
}

void PCF857x::write16(uint16_t value)
{
  if(!_is8575) return;
  Wire.beginTransmission(_address);
  _pinModeMask = value;
  _data = _pinModeMask;
  Wire.write((uint8_t) _data);
  Wire.write((uint8_t) (_data >> 8));
  _error = Wire.endTransmission();
}

uint8_t PCF857x::read(uint8_t pin)
{
  if(_is8575)
  {
    if(pin > 15)
    {
      _error = PCF857x_PIN_ERROR;
      return 0;
    }
    PCF857x::read16();
  }
  else
  {
    if(pin > 7)
    {
      _error = PCF857x_PIN_ERROR;
      return 0;
    }
    PCF857x::read8();
  }
  return (_data & (1<<pin)) > 0;
}

void PCF857x::write(uint8_t pin, uint8_t value)
{
  if(_is8575)
  {
    if(pin > 15)
    {
      _error = PCF857x_PIN_ERROR;
      return;
    }
  }
  else if(pin > 7)
  {
    _error = PCF857x_PIN_ERROR;
    return;
  }
  uint8_t _val = value & 1;
  if(_val) _pinModeMask |= _val << pin;
  else _pinModeMask &= ~(1 << pin);
  if(_is8575) PCF857x::write16(_pinModeMask);
  else PCF857x::write8(_pinModeMask);
}

void PCF857x::toggle(uint8_t pin)
{
  if(_is8575)
  {
    if(pin > 15)
    {
      _error = PCF857x_PIN_ERROR;
      return;
    }
  }
  else if(pin > 7)
  {
    _error = PCF857x_PIN_ERROR;
    return;
  }
  if(_is8575) PCF857x::write16(_pinModeMask);
  else PCF857x::write8(_pinModeMask);
}

void PCF857x::toggleAll()
{
  _pinModeMask = ~_pinModeMask;
  if(_is8575) PCF857x::write16(_pinModeMask);
  else PCF857x::write8(_pinModeMask);
}

void PCF857x::shiftRight(uint8_t n)
{
  if(_is8575)
  {
    if (n == 0 || n > 15 ) return;
  }
  else if (n == 0 || n > 7 ) return;
  _pinModeMask >>= n;
  if(_is8575) PCF857x::write16(_pinModeMask);
  else PCF857x::write8(_pinModeMask);
}

void PCF857x::shiftLeft(uint8_t n)
{
  if(_is8575)
  {
    if (n == 0 || n > 15 ) return;
  }
  else if (n == 0 || n > 7 ) return;
  _pinModeMask <<= n;
  if(_is8575) PCF857x::write16(_pinModeMask);
  else PCF857x::write8(_pinModeMask);
}

void PCF857x::rotateRight(uint8_t n)
{
  if(_is8575){
    uint8_t r = n & 15;
    _pinModeMask = (_pinModeMask >> r) | (_pinModeMask << (16-r));
    PCF857x::write16(_pinModeMask);
  } else {
    uint8_t r = n & 7;
    _pinModeMask = (_pinModeMask >> r) | (_pinModeMask << (8-r));
    PCF857x::write8(_pinModeMask);
  }
}

void PCF857x::rotateLeft(uint8_t n)
{
  if(_is8575) PCF857x::rotateRight(16- (n & 15));
  else PCF857x::rotateRight(8- (n & 7));
}

int PCF857x::lastError()
{
  int e = _error;
  _error = 0;
  return e;
}
