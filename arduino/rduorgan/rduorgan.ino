/*****************************************************************************
// Code for calliope controller in AXR robot.  Reads servo values from R/C
// receiver and serial commands from Raspberry Pi and controls calliope
// valves and other local functions.
//
// Written for Arduino Uno or compatlble; compatibility not tested on other
// boards (especially the PWM code and pin assignment).
//
// Copyright 2015 by Robin Mitchell
//
*****************************************************************************/

#include <Arduino.h>

// comment this out if you're running without the expansion shield
#define SHIELD_EXISTS
// comment this out if you're driving the actual calliope valve assembly
//#define SPEAKER_MODE
// comment this out to not use the servos
#define READING_SERVOS
// leave this defined to get serial debugging output
#define DEBUG_OUTPUT

#ifdef SPEAKER_MODE
#include <PWM.h>
#endif

#ifdef SHIELD_EXISTS
#include <Wire.h>
#include <Adafruit_MCP23008.h>
#endif

#ifdef READING_SERVOS
#include <ServoDecoder.h>
#endif

void debugPrint(const char *msg) {
#ifdef DEBUG_OUTPUT
  Serial.print(msg);
#endif
}

void debugPrintln(const char *msg) {
#ifdef DEBUG_OUTPUT
  Serial.println(msg);
#endif
}


/******************** Shift-Register Manipulation ********************/
#ifdef SHIELD_EXISTS
//Pin connected to RCK of TPIC6B595 (pin 12)
const int srLatchPin = 8;
//Pin connected to SRCK of TPIC6B595 (pin 13)
const int srClockPin = 12;
////Pin connected to SER IN of TPIC6B595 (pin 3)
const int srDataPin = 11;

void writeSR16(uint16_t data) {
  // take the latchPin low so the outputs don't change while you're sending in bits:
  digitalWrite(srLatchPin, LOW);
  // write the high byte first
  shiftOut(srDataPin, srClockPin, MSBFIRST, data >> 8);  
  // now the low byte
  shiftOut(srDataPin, srClockPin, MSBFIRST, data & 0xFF);  
  //take the latch pin high to output the new data
  digitalWrite(srLatchPin, HIGH);
} 

void initSR() {
  //set pins to output so we can control the shift register
  pinMode(srLatchPin, OUTPUT);
  pinMode(srClockPin, OUTPUT);
  pinMode(srDataPin, OUTPUT);
  writeSR16(0);
}
#endif


/******************** Frequency-Generator Code ********************/
#ifdef SPEAKER_MODE

const int speaker_pin = 9;

const uint16_t noteFreqs[] = { 0, 311, 329, 349, 369, 391, 440, 493, 466,
                              523, 554, 587, 659, 622, 698, 739, 783 };

void initFG() {
  InitTimersSafe(); 
}

void playSpeaker(uint16_t note) {
  if (0 == note)
    pwmWrite(speaker_pin, 0);
  else {
    SetPinFrequencySafe(speaker_pin, noteFreqs[constrain(note, 1, 16)]);
    pwmWrite(speaker_pin, 128);
  }
}

#endif

/******************** MCP I/O Expander Code ********************/
#ifdef SHIELD_EXISTS

Adafruit_MCP23008 mcp;

void initMCP() {
  mcp.begin(0); // assumes MCP23008 chip set to I2C address 0
}

#endif

void mcpWrite(int pin, uint8_t val) {
#ifdef SHIELD_EXISTS
  mcp.digitalWrite(pin, val);
#endif
}

/******************** Calliope-Player Code ********************/

// note-to-pipe conversion

const uint16_t pipeTable[] = { 0, 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
                                  0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };

struct Note {
  uint8_t  pipes;
  uint8_t   beats;
};

class Calliope {
  public:
    Calliope();
    void play(const Note *tune, int16_t tempo);
    void stop();
    bool playing() const {
      return running; }
    bool cycle();
    
  protected:
    void playNote(const Note *note);
    void setPipes(uint8_t pipes);
    
    bool    running;
    bool    resting;
    const Note    *currentNote;
    uint16_t  tempo;
    uint16_t  restInterval;
    uint32_t  timer;
};

Calliope::Calliope() {
  restInterval = 15;  // 15ms rest between notes
  stop();
}

void Calliope::play(const Note *tune, int16_t tempo) {
  if (tune->beats != 0) {
    running = true;
    resting = false;
    currentNote = tune;
    this->tempo = tempo;
    playNote(currentNote);
  }
}

void Calliope::stop() {
  // this assumes 0 turns all pipes off
  setPipes(0);
  running = false;
  debugPrintln("END");
}

// returns true if tune still running
bool Calliope::cycle() {
  if (running) {
    // see if it's time to do something
    if (timer <= millis())
      if (resting) {
        // we were in an inter-note pause, so move to the next note
        resting = false;
        currentNote++;
        if (0 == currentNote->beats)
          stop();
        else
          playNote(currentNote);
      }
      else {
      // we were playing a note, so stop and rest
      resting = true;
      setPipes(0);
      timer = millis() + restInterval;
    }
  }
  return running;
}

void Calliope::playNote(const Note *note) {
  setPipes(note->pipes);
  timer = millis() + (tempo * note->beats);
}

void Calliope::setPipes(uint8_t pipes) {
  if (!resting) {
#ifdef DEBUG_OUTPUT
    Serial.print(pipes);
    Serial.print("  ");
#endif
  }
#ifdef SHIELD_EXISTS
  writeSR16(pipeTable[constrain(pipes, 0, 16)]);
#endif
#ifdef SPEAKER_MODE
  playSpeaker(pipes);
#endif
}

/**************** Bell and Smoke Drivers *****************/

int testPulseWidth = 100;

class BellRinger {
  public:
    BellRinger(int pin) :
      pin(pin), state(STOPPED) {};
    void start();
    void stop();
    bool cycle();
    bool active();

  protected:
    enum      BellState { STOPPED, RINGING };
    BellState state;
    int       pin;
};

void BellRinger::start() {
  state = RINGING;
  mcpWrite(pin, HIGH);
}

void BellRinger::stop() {
    state = STOPPED;
    mcpWrite(pin, LOW);
 }

bool BellRinger::active() {
  return STOPPED != state;
}

bool BellRinger::cycle() {
  return active();
}

class SmokeGenerator {
  public:
    SmokeGenerator(int valve_pin, int heater_pin) :
      valve_pin(valve_pin), heater_pin(heater_pin), state(STOPPED) {};
    void start();
    void stop();
    bool cycle();
    bool active();

  protected:
    enum      SmokeState { STOPPED, STARTING, HEATING, RESTING, STOPPING };
    SmokeState state;
    int       valve_pin, heater_pin;
    uint32_t  timer;
    uint32_t  cycleTimer;  // how long have we been heating?
};

void SmokeGenerator::start() {
  if ((STOPPED == state) || (STOPPING == state)) {
    // start fan for 1 second before making smoke
    state = STARTING;
    mcpWrite(valve_pin, HIGH);
    timer = millis() + 1000;  // run fan for 1 second at start
    // remember when this cycle started so we know when to cool off
    cycleTimer = millis();
  }
}

void SmokeGenerator::stop() {
  if (active()) {
    // shut heater off, run fan for cooloff
    mcpWrite(valve_pin, HIGH);
    mcpWrite(heater_pin, LOW);
    state = STOPPING;
    timer = millis() + 5000;
  }
}

bool SmokeGenerator::active() {
  return STOPPED != state;
}

bool SmokeGenerator::cycle() {
  // see if it's time to do anything
  if ((STOPPED != state) && (timer <= millis())) {
    switch (state) {
      case HEATING:   // if heating, stop
         mcpWrite(heater_pin, LOW);
        state = RESTING;
        // either a short or a long rest, depending on how long we've been running
        if ((millis() - cycleTimer) >= 48000UL) {
          timer = millis() + 30000UL;  // 30-second cooloff
          cycleTimer = millis() + 30000UL;
        }
        else
          timer = millis() + 4000UL; // 4 sec between heat pulses in normal mode
        break;
      case STARTING:
      case RESTING:   // if resting, heat
        mcpWrite(heater_pin, HIGH);
        state = HEATING;
        timer = millis() + 4000UL;  // 4 seconds of heating
        break;
      case STOPPING:   // done, so shut down
        mcpWrite(valve_pin, LOW);
        state = STOPPED;
    }
  }
  return active();
}

/************************ Blinker ************************/

class LEDBlinker {
  public:
    LEDBlinker(int pin, uint32_t period = 500);
    void pulse();
    void cycle();

  protected:
    int       pin;
    uint32_t  period; 
    uint32_t  timer;
    uint32_t  pulseTimer;
    bool      lit;
};

LEDBlinker::LEDBlinker(int pin, uint32_t period) :
  pin(pin), period(period) {
  lit = false;
  timer = millis();
  pulseTimer = 0;
}

void LEDBlinker::pulse() {

  timer = millis();
  pulseTimer = millis() + period;
}
  
void LEDBlinker::cycle() {
  if (timer <= millis()) {
    if (lit)
      mcpWrite(pin, LOW);
    else
      mcpWrite(pin, HIGH);
    lit = !lit;
    if (pulseTimer > millis())
      timer = millis() + period / 8;
    else
      timer = millis() + period;
  }
} 

/*********************** Main Code ***********************/

const int bell_pin = 0;
const int smokeValve_pin = 1;
const int smokeHeater_pin = 2;
const int aux1_pin = 3;
const int testButton_pin = 7;
const int testLED_pin = 6;

enum cmd_type { NO_CMD, PLAY_1, PLAY_2, PLAY_3, PLAY_4, STOP_PLAY, SMOKE_ON, SMOKE_OFF, BELL_ON, BELL_OFF,
                AUX1_ON, AUX1_OFF, AUX2_ON, AUX2_OFF };

Calliope organ;
BellRinger bell(bell_pin);
SmokeGenerator smoker(smokeValve_pin, smokeHeater_pin);
LEDBlinker blinker(testLED_pin);

bool  aux1Active;

#ifdef READING_SERVOS
  AnalogChannel servo4(4);
  AnalogChannel servo6(6);
//  BooleanChannel calliopeTrigger(5);
  bool lastCalliopeTrigger, lastBellTrigger, lastSmokeTrigger;
#endif

const uint16_t vaderTempo = 90;
const Note vaderTune[] = {
    {5,8}, {5,8}, {5,8}, {1,6}, {8,2},   {5,8}, {1,6}, {8,2}, {5,16}, {0,2},   {11,8}, {11,8}, {11,8}, {13,6}, {8,2},
    {4,8}, {1,6}, {8,2}, {5,16}, {0,2},   {16,8}, {5,6}, {5,2}, {16,8}, {15,6}, {14,2},   {12,2}, {13,2}, {12,2}, {0,2}, {5,4}, {10,8}, {9,6}, {7,2},
    {8,2}, {6,2}, {8,4}, {0,2}, {1,4}, {5,8}, {1,6}, {5,2},   {8,8}, {5,6}, {8,2}, {11,16}, {0,2},  {16,8}, {5,6}, {5,2}, {16,8}, {15,6}, {14,2},
    {12,2}, {13,2}, {12,4}, {0,2}, {5,4}, {10,8}, {9,6}, {7,2},   {8,2}, {6,2}, {8,4}, {0,2}, {1,4}, {5,8}, {1,6}, {8,2},   {5,8}, {1,6}, {8,2}, {5,16},
    {0,0}};

/* Original, as notated by Joe
const uint16_t starWarsTempo = 110;
const Note starWarsTune[] = {
    {1,8}, {8,4}, {6,4}, {5,4}, {3,4}, {13,8}, {8,4}, {6,4}, {5,4}, {3,4},
    {13,8}, {8,4}, {6,4}, {5,4}, {6,4}, {3,12}, {0,4}, {1,8}, {8,4}, {6,4}, {5,4}, {3,4},
    {13,8}, {8,4}, {6,4}, {5,4}, {3,4}, {13,8}, {8,4}, {6,4}, {5,4}, {6,4}, {3,12},
  {0,0} };
*/
const uint16_t starWarsTempo = 60;
const Note starWarsTune[] = {
    {1,16}, {8,12}, {6,7}, {5,7}, {3,7}, {13,16}, {8,12}, {6,7}, {5,7}, {3,7},
    {13,16}, {8,12}, {6,7}, {5,7}, {6,7}, {3,24}, {0,8}, {1,16}, {8,12}, {6,7}, {5,7}, {3,7},
    {13,16}, {8,12}, {6,7}, {5,7}, {3,7}, {13,16}, {8,12}, {6,7}, {5,7}, {6,7}, {3,24},
  {0,0} };
    
const uint16_t ewoksTempo = 180;
const Note ewoksTune[] = {
    {5,4}, {5,4}, {2,4}, {5,4}, {1,4}/*???*/, {5,4}, {2,4}, {5,4}, {5,4}, {5,4}, {6,4}, {7,4}, {9,4},
 {0,0} };

const uint16_t scalesTempo = 500;
const Note scalesTune[] = { {1,1}, {2,1}, {3,1}, {4,1}, {5,1}, {6,1}, {7,1}, {8,1}, {9,1}, {10,1}, {11,1}, {12,1},
                            {13,1}, {14,1}, {15,1}, {16,1}, {0,0} };


void setup() {
  Serial.begin(9600);
  #ifdef DEBUG_OUTPUT
  Serial.print("Starting - build "); Serial.print(__DATE__); Serial.print("  "); Serial.println(__TIME__);
  #endif
  #ifdef SPEAKER_MODE
  initFG();
  #endif
  #ifdef SHIELD_EXISTS
  initSR();
  initMCP();
  mcp.pinMode(bell_pin, OUTPUT);
  mcp.digitalWrite(bell_pin, LOW);
  mcp.pinMode(smokeValve_pin, OUTPUT);
  mcp.digitalWrite(smokeValve_pin, LOW);
  mcp.pinMode(smokeHeater_pin, OUTPUT);
  mcp.digitalWrite(smokeHeater_pin, LOW);
  mcp.pinMode(aux1_pin, OUTPUT);
  mcp.digitalWrite(aux1_pin, LOW);
  mcp.pinMode(testButton_pin, INPUT);
  mcp.pullUp(testButton_pin, HIGH);
  mcp.pinMode(testLED_pin, OUTPUT);
  mcp.digitalWrite(testLED_pin, LOW);
  #endif
  #ifdef READING_SERVOS
  ServoDecoder::init();
  servo4.setSlew(25);
  servo6.setSlew(20);
  lastCalliopeTrigger = false;
  lastBellTrigger = false;
  lastSmokeTrigger = false;
  #endif
  aux1Active = false;
}

void loop() {
  // very first thing to do is see if the test button is pushed
#ifdef SHIELD_EXISTS
  if (!mcp.digitalRead(testButton_pin) && !organ.playing()) {
    debugPrintln("Test button!");
    organ.play(scalesTune, scalesTempo);
  }
#endif

  // next thing to do is see if we have received a command
  cmd_type cmd = NO_CMD;
  // look for serial input first
  if (Serial.available() > 0) {
    char ch = Serial.read();
    switch (ch) {
      case 'p':
      case 'P':
        testPulseWidth = Serial.parseInt();
        Serial.print("Width = "); Serial.println(testPulseWidth);
        break;
      case '0':
        cmd = STOP_PLAY;
        break;
      case '1':
        cmd = PLAY_1;
        break;
      case '2':
        cmd = PLAY_2;
        break;
      case '3':
        cmd = PLAY_3;
        break;
      case '4':
        cmd = PLAY_4;
        break;
      case 'B':
        cmd = BELL_ON;
        break;
      case 'b':
        cmd = BELL_OFF;
        break;
      case 'S':
        cmd = SMOKE_ON;
        break;
      case 's':
        cmd = SMOKE_OFF;
        break;
      case 'X':
        cmd = AUX1_ON;
        break;
      case 'x':
        cmd = AUX1_OFF;
        break;
    }
  }
#ifdef READING_SERVOS
  // if no serial command, try the servos
  if (NO_CMD == cmd) {
    if (servo6.validData()) {
    // channel 6 is used as a boolean toggle for the organ, channel 4 controls bell & smoke
    int ch6val = servo6.value();
    if ((ch6val < 1350) && !lastCalliopeTrigger) {
      lastCalliopeTrigger = true;
      if (!organ.playing())
        cmd = PLAY_2;   // Imperial March is the only tune that really works
    }
/*      
    bool trigger = calliopeTrigger.value();
    if (trigger && !lastCalliopeTrigger) { 
      lastCalliopeTrigger = true;
//Serial.println("Servo trigger");
      if (!organ.playing()) {
        int ch6val = servo6.value();
        if (ch6val > 1800)
          cmd = PLAY_1;
        else if (ch6val > 1600)
          cmd = PLAY_2;
        else
          cmd = PLAY_3;
      }
    }
*/
    else
      if (lastCalliopeTrigger && (ch6val > 1750 )) {
        lastCalliopeTrigger = false;
        if (organ.playing())
          cmd = STOP_PLAY;
      }
    }
//    if (!servo4.validData())
//      Serial.println("BAD CH4");
    if ((NO_CMD == cmd) && servo4.validData()) {
      int ch4val = servo4.value();
//      Serial.println(ch4val);
      if ((ch4val < 1300) && !lastSmokeTrigger) {
        cmd = SMOKE_ON;
        lastSmokeTrigger = true;
      }
      else
        if ((ch4val > 1400) & lastSmokeTrigger) {
          cmd = SMOKE_OFF;
          lastSmokeTrigger = false;
        }
        else
          if ((ch4val > 1700) && !lastBellTrigger) {
            cmd = BELL_ON;
            lastBellTrigger = true;
          }
          else
            if ((ch4val < 1600) && lastBellTrigger) {
              cmd = BELL_OFF;
              lastBellTrigger = false;
            }
    }
  }
#endif

  if (NO_CMD != cmd)
    blinker.pulse();

  // if we have a command, execute it
  switch(cmd) {
    case STOP_PLAY:
      debugPrintln("Stop organ");
      organ.stop();
      break;
      
    case PLAY_1:
      debugPrintln("Play 1");
      organ.play(starWarsTune, starWarsTempo);
      break;
    case PLAY_2:
    case PLAY_3:  // here for now because Ewoks tune is broken
      debugPrintln("Play 2");
     organ.play(vaderTune, vaderTempo);
      break;
/*
    case PLAY_3:
      debugPrintln("Play 3");
      organ.play(ewoksTune, ewoksTempo);
      break;
*/
    case PLAY_4:
      debugPrintln("Play 4");
      organ.play(scalesTune, scalesTempo);
      break;
      
    case BELL_ON:
      debugPrintln("Bell on");
      bell.start();
      break;
          
    case BELL_OFF:
      debugPrintln("Bell off");
      bell.stop();
      break; 
         
    case SMOKE_ON:
      debugPrintln("Smoke on");
      smoker.start();
      break;
          
    case SMOKE_OFF:
      debugPrintln("Smoke off");
      smoker.stop();
      break;    

    case AUX1_ON:
      debugPrintln("Aux1 on");
      mcpWrite(aux1_pin, HIGH);
      aux1Active = true;
      break;
          
    case AUX1_OFF:
      debugPrintln("Aux1 off");
      mcpWrite(aux1_pin, LOW);
      aux1Active = false;
      break;    
  }
  // do maintenance for background processes
  organ.cycle();
  bell.cycle();
  smoker.cycle();
  blinker.cycle();
}
