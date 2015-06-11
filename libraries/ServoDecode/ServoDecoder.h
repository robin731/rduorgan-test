#ifndef _SERVODECODER_H_
#define _SERVODECODER_H_

#include <Arduino.h>

class ServoDecoder {
  public:
    static void PCINT2_INT();
    static void init();
    static volatile uint32_t getWidth(uint8_t ch) {
      return width[ch]; }
    static volatile uint32_t getRisingEdge(uint8_t ch) {
      return risingEdge[ch]; }
    static volatile uint32_t getBogus() {
      return bogus; }
    static void setSlew(uint8_t ch, uint16_t rate) {
      slew[ch] = rate; }
  protected:
    static volatile uint8_t prev; // remembers state of input bits from previous interrupt
    static volatile uint32_t risingEdge[6]; // time of last rising edge for each channel
    static volatile uint32_t width[6]; // the latest measured pulse width for each channel
    static uint16_t slew[6]; // maximum slew-rate for each channel
    static volatile uint32_t bogus;  // the last out-of-range time for any channel
};

class ServoChannel {
  public:
    ServoChannel(uint8_t pin);
    bool validData() const;
    uint32_t rawData() const;
    void setNosync(uint32_t val) {
            nosyncVal = val; }
    void setSlew(uint16_t rate);

  protected:
    uint8_t   channel;
    uint32_t  nosyncVal;
};

class AnalogChannel : public ServoChannel {
  public:
    AnalogChannel(uint8_t pin, int minVal = 1000, int maxVal = 2000);
    int value();

  protected:
    int minVal, maxVal;
    int last;
};

class BooleanChannel : public ServoChannel {
  public:
    BooleanChannel(uint8_t pin);
    bool value();
  
  protected:
    bool last;
};

#endif