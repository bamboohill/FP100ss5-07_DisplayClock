/* PCF8574_ESP -- library for using the I2C-driven 8-pin GPIO-expander
   ORIGINAL AUTHOR: Rob Tillaart
   Library modified by WereCatf */

#ifndef _PCF8574_ESP_H
#define _PCF8574_ESP_H


#include <Arduino.h>
#include <Wire.h>

#define PCF857x_OK          0x00
#define PCF857x_PIN_ERROR   0x81
#define PCF857x_I2C_ERROR   0x82

class PCF857x
{
  public:
    // Defaults to 8574, set is8575 to true if you have a 8575 instead.
    PCF857x(uint8_t address, bool is8575 = false);

    void begin(uint16_t defaultValues=0xffff);
    uint8_t read8();
    uint16_t read16();
    uint8_t read(uint8_t pin);

    void write8(uint8_t value);
    void write16(uint16_t value);
    void write(uint8_t pin, uint8_t value);

    void toggle(uint8_t pin);
    void toggleAll();
    void shiftRight(uint8_t n=1);
    void shiftLeft(uint8_t n=1);
    void rotateRight(uint8_t n=1);
    void rotateLeft(uint8_t n=1);
    void resetInterruptPin();

    int lastError();

  private:
    TwoWire *_Wire;
    uint8_t _address;
    uint16_t _data;
    uint16_t _pinModeMask;
    int _error;
    bool _is8575;
};

#endif
