/*****************************************************************************
// Interrupt-based code to read and decode R/C servo channels on pins 2-7
//
// Written 2015 by Robin Mitchell
// Adapted from servo decoder article at http://http://ceptimus.co.uk/?p=66
*****************************************************************************/
#include "ServoDecoder.h"

#define BAD_PULSE(x) (((x) < 750) || ((x) > 2250))

void ServoDecoder::init() {
  bogus = millis();
  
  for (int pin = 2; pin <= 7; pin++) { // enable pins 2 to 7 as our 6 input bits
    pinMode(pin, INPUT);
  }

  PCMSK2 |= 0xFC; // set the mask to allow those 6 pins to generate interrupts
  PCICR |= 0x04;  // enable interupt for port D
}

ISR(PCINT2_vect) { // one or more of pins 2~7 have changed state
  ServoDecoder::PCINT2_INT();
}

void ServoDecoder::PCINT2_INT() {
  uint32_t now = micros();
  uint8_t curr = PIND; // current state of the 6 input bits
  uint8_t changed = curr ^ prev;
  int channel = 0;
  for (uint8_t mask = 0x04; mask; mask <<= 1) {
    if (changed & mask) { // this pin has changed state
      if (curr & mask) { // +ve edge so remember time
        risingEdge[channel] = now;
      }
      else { // -ve edge so store pulse width
        uint32_t uSec = now - risingEdge[channel];
        // flag as bogus if it's out of range
        if (BAD_PULSE(uSec))
          bogus = millis();
        else {
          // if no slew-rate limit, just store the value
          if (0 == slew[channel])
            width[channel] = uSec;
          // else slew-rate limit it
          int32_t diff = uSec - width[channel];
          if (abs(diff) <= slew[channel])
            width[channel] = uSec;
          else
            if (diff < 0)
              width[channel] -= slew[channel];
            else
              width[channel] += slew[channel];             
        }
      }
    }
    channel++;
  }
  prev = curr;
}

volatile uint8_t ServoDecoder::prev = 0;
volatile uint32_t ServoDecoder::risingEdge[6] = {0, 0, 0, 0, 0, 0};
volatile uint32_t ServoDecoder::width[6] = {1500, 1500, 1500, 1500, 1500, 1500};
uint16_t ServoDecoder::slew[6]  = {0, 0, 0, 0, 0, 0};
volatile uint32_t ServoDecoder::bogus = 0;
 
  
ServoChannel::ServoChannel(uint8_t pin) {
  channel = (pin > 1) ? pin-2 : pin;
  nosyncVal = 1500;
}

// data is considered valid if we've gone 500mS without a bogus value on any channel,
// have had a pulse on this channel in last 200mS, and width is good
bool ServoChannel::validData() const {
  // need to get this first so ISR doesn't update it in the middle of comparison
  uint32_t edge = ServoDecoder::getRisingEdge(channel);
  return ((millis() - ServoDecoder::getBogus()) > 500) && ((micros() - edge) < 200000UL) && 
            !BAD_PULSE(ServoDecoder::getWidth(channel));
}

uint32_t ServoChannel::rawData() const {
  return validData() ? ServoDecoder::getWidth(channel) : nosyncVal;
}

void ServoChannel::setSlew(uint16_t rate) {
  ServoDecoder::setSlew(channel, rate);
}

AnalogChannel::AnalogChannel(uint8_t pin, int minVal, int maxVal) :
  ServoChannel(pin), minVal(minVal), maxVal(maxVal) {
  last = (maxVal - minVal) / 2;
}

int AnalogChannel::value() {
  if (!validData())
    return last;
  return map(constrain(rawData(), 1000, 2000), 1000, 2000, minVal, maxVal);
}


BooleanChannel::BooleanChannel(uint8_t pin) :
  ServoChannel(pin), last(false) {
}

bool BooleanChannel::value() {
  if (!validData())
    return last;
  // provide some hysteresis so we don't chatter when the value is near midpoint
  if (last && (rawData() < 1400)) {
    last = false;
  }
  else
    if ((!last) && (rawData() > 1600)) {
      last = true;
    }
  return last;
}