#include <WiFi.h>
#include "time.h"
#include "sntp.h"
#include "Streaming.h"
#include "WiFiManager.h"

#define ESP_DRD_USE_SPIFFS    true
#include <ESP_DoubleResetDetector.h>

// Number of seconds after reset during which a 
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 3 

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

DoubleResetDetector* drd;

WiFiManager wifiManager;

#define _BINZ(a,b) _WIDTHZ(_BIN(a), b)

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";

const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Amsterdam including daylight adjustment rules (optional)
uint16_t    ms;

hw_timer_t  *Clock, *PW, *PWCharlie;
int pwDigits=500, pwLEDs[4];

uint8_t segPin[7]       = { 18, 5, 17, 16, 4, 2, 15 };      // pins connected to segments a-g of all digits
uint8_t posPin[4]       = { 23, 22, 21, 19 };               // pins connected to common of digits, right to left

#define CHARLIE_PINS 4
#define CHARLIE_LEDS (CHARLIE_PINS * (CHARLIE_PINS-1))
uint8_t charliePins[CHARLIE_PINS]   = { 12, 14, 27, 26 };
uint8_t charlieOrder[CHARLIE_LEDS]  = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
//uint8_t charliePins[CHARLIE_PINS]   = { 23, 22, 18, 5 };
//uint8_t charlieOrder[CHARLIE_LEDS]  = { 4, 0, 1, 5, 2, 3, 6, 7, 8, 9, 10, 11 };
uint8_t charlieLEDs[4]; // Maximum 4 LEDs lighting at 'once'

int Brightness[100];

enum ShapeNames {
  _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _a, _b, _c, _d, _e, _f, _UNDER, _MINUS, _OVER, _h, _j, _l, _n, _o, _p, _q, _r, _t, _u, _y, _SPACE, _numShapes
};

uint8_t shape[_numShapes] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b01110111, // a
  0b01111100, // b
  0b00111001, // c
  0b01011110, // d
  0b01111001, // e
  0b01110001, // f
  0b00001000, // _
  0b01000000, // -
  0b00000001, // upperscore
  0b01110110, // H
  0b00001110, // J
  0b00111000, // L
  0b01010100, // N
  0b01011100, // O
  0b01110011, // P
  0b01100111, // Q
  0b01010000, // R
  0b01111000, // T
  0b00111110, // U
  0b01101110, // Y
  0b00000000, // SPACE
};

uint32_t  rawShape[_numShapes];
uint32_t  rawSegs[7];
uint32_t  rawSegmentsOff;
uint32_t  rawPos[4];
uint32_t  rawPosOff;
uint32_t  rawDigit[4];
uint32_t  rawCharlie[CHARLIE_LEDS], rawCharlieOE[CHARLIE_LEDS], rawCharlieOff;

#define _g _9
#define _i _1
#define _s _5


void print7Seg(byte ll, byte lm=_SPACE, byte rm=_SPACE, byte rr=_SPACE) {
  rawDigit[3] = rawShape[ll];
  rawDigit[2] = rawShape[lm];
  rawDigit[1] = rawShape[rm];
  rawDigit[0] = rawShape[rr];
}


void precalcShapes(void) {
  int sh, bit, pos;
  
  for (sh=0; sh<_numShapes; sh++ ) {
    uint32_t  currShape=0;
    for (int seg=0; seg<7; seg++) {
      if (shape[sh] & (1<<seg)) currShape |= (1L<<segPin[seg]);
    }
    rawShape[sh]  = currShape;
  }
  
  rawSegmentsOff  = 0;
  for (bit=0; bit<7; bit++) {
    rawSegmentsOff  |= 1L << segPin[bit];
  }

  rawPosOff = 0;
  for (pos=0; pos<4; pos++) {
    rawPos[pos] = 1L << posPin[pos];
    rawPosOff |= rawPos[pos];
  }
}

void precalcCharlie(void) {
  int   i, j, LED;
  
  LED = 0;
  for (i=0; i<CHARLIE_PINS; i++) {
    rawCharlieOff |= (1L<<charliePins[i]);
    for (j=0; j<CHARLIE_PINS; j++) {
      if (i==j) continue;
      rawCharlie[charlieOrder[LED]]   = (1L<<charliePins[i]);
      rawCharlieOE[charlieOrder[LED]] = (1L<<charliePins[i]) | (1L<<charliePins[j]);
      LED++;
    }
  }
}


void printLocalTime()
{ 
  static int  lastLED;
  int         LED;
  
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    print7Seg(
      timeinfo.tm_hour / 10 ? timeinfo.tm_hour / 10 : _SPACE, // don't print leading zero
      timeinfo.tm_hour % 10, 
      timeinfo.tm_min / 10, 
      timeinfo.tm_min % 10
    );

    LED             = timeinfo.tm_sec / 6;
    if (LED != lastLED) ms=0;   // synchronize 6000 ms counter
    lastLED         = LED;
    charlieLEDs[0]  = LED;
    charlieLEDs[1]  = (LED + 1) % 10;
    int lastSec = ms / 60;      // range 0..99
    pwLEDs[0] = Brightness[99 - lastSec];
    pwLEDs[1] = Brightness[lastSec];

    lastSec = (ms/10) % 200;
    if (lastSec>=100) lastSec = 199 - lastSec;
    pwLEDs[2] = Brightness[lastSec];
    pwLEDs[3] = Brightness[lastSec];
    
  } else {
      // print7Seg(_MINUS, _MINUS, _MINUS, _MINUS);
  }
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  Serial << "Got time adjustment from NTP in " << millis() << " ms!\n";
}

bool IRAM_ATTR onClock(void) {
  static int  digit; 

  if (++ms>=6000) ms=0;

  WRITE_PERI_REG(GPIO_OUT_W1TC_REG, rawPosOff);        // disable all digits, Common Anode, high active
  WRITE_PERI_REG(GPIO_OUT_W1TS_REG, rawSegmentsOff);   // disble all segments, low active
  ++digit &= 3;                                        // next digit
  WRITE_PERI_REG(GPIO_OUT_W1TC_REG, rawDigit[digit]);  // display new digit
  WRITE_PERI_REG(GPIO_OUT_W1TS_REG, rawPos[digit]);    // enable common anode

  timerRestart(PW);
  timerAlarmWrite(PW, pwDigits, false); // 100 us timer, no auto reload.
  timerAlarmEnable(PW);

  WRITE_PERI_REG(GPIO_OUT_W1TC_REG, rawCharlieOff);           // disable all ('seconds')-LEDS data ...
  WRITE_PERI_REG(GPIO_ENABLE_W1TC_REG, rawCharlieOff);        // ... and Output Enable

  if (pwLEDs[digit]) { // If this digit has non-zero pulsewidth
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG,    rawCharlie[charlieLEDs[digit]]);   // Light one of the LEDS data ...
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, rawCharlieOE[charlieLEDs[digit]]); // ... and Output Enable

    timerRestart(PWCharlie);
    timerAlarmWrite(PWCharlie, pwLEDs[digit], false); // 100 us timer, no auto reload.
    timerAlarmEnable(PWCharlie);
  }

  return false; // do not perform task switch. We handled all.
}


bool IRAM_ATTR onPW(void) {
  WRITE_PERI_REG(GPIO_OUT_W1TC_REG, rawPosOff);        // disable all digits, Common Anode, high active
  WRITE_PERI_REG(GPIO_OUT_W1TS_REG, rawSegmentsOff);   // disble all segments, low active
  return false; // do not perform task switch. We handled all.
}


bool IRAM_ATTR onPWCharlie(void) {
  WRITE_PERI_REG(GPIO_OUT_W1TC_REG,    rawCharlieOff);        // disable all ('seconds')-LEDS data ...
  WRITE_PERI_REG(GPIO_ENABLE_W1TC_REG, rawCharlieOff);        // ... and Output Enable
  return false; // do not perform task switch. We handled all.
}


void configModeCallback(WiFiManager *myWiFiManager) {
  print7Seg(_a,_p);
}


void setup()
{
  int i;

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  Serial.begin(115200);

  precalcShapes();
  precalcCharlie();

  print7Seg(_b,_o,_o,_t);

// init output pins
  for (i=0; i<7; i++) {
    gpio_set_drive_capability((gpio_num_t)segPin[i], GPIO_DRIVE_CAP_1);
    pinMode(segPin[i], OUTPUT);
    digitalWrite(segPin[i], HIGH); // segments are off when high
  }
  for (i = 0; i < 4; i++) {
    gpio_set_drive_capability((gpio_num_t)posPin[i], GPIO_DRIVE_CAP_3);
    pinMode(posPin[i], OUTPUT);
    digitalWrite(posPin[i], LOW);
  }

  // Initialize timer
  Clock = timerBegin(0, 80, true);  // 1 MHz repeating timer. Channel 0, prescaler 80, count up.
  timerAttachInterrupt(Clock, (void (*)())onClock, false);
  timerAlarmWrite(Clock, 1000, true); // 1 ms timer, auto reload.
  timerAlarmEnable(Clock);

  PW = timerBegin(1, 80, true);  // Pulse width timer. Channel 1, prescaler 80, count up.
  timerAttachInterrupt(PW, (void (*)())onPW, false);
  timerAlarmWrite(PW, 100, false); 
  timerAlarmEnable(PW);

  PWCharlie = timerBegin(3, 80, true);  // Pulse width timer. Channel 2, prescaler 80, count up.
  timerAttachInterrupt(PWCharlie, (void (*)())onPWCharlie, false);
  timerAlarmWrite(PWCharlie, 100, false); // 100 us timer, no auto reload.
  timerAlarmEnable(PWCharlie);

  delay(500);
  print7Seg(_c,_o,_n,_n);
  wifiManager.setAPCallback(configModeCallback);
  if (drd->detectDoubleReset()) {
    wifiManager.startConfigPortal("PaulKlok");
  } else {
    wifiManager.autoConnect("PaulKlok");
  }
  /**
     A more convenient approach to handle TimeZones with daylightOffset
     would be to specify a environmnet variable with TimeZone definition including daylight adjustmnet rules.
     A list of rules for your zone could be obtained from https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
  */
  print7Seg(_n,_t,_p);
  sntp_set_time_sync_notification_cb( timeavailable );
  configTzTime(time_zone, ntpServer1, ntpServer2);

  charlieLEDs[2]  = 10;  // Must be 10 and 11 in final version
  charlieLEDs[3]  = 11;
  
  pwLEDs[0] = 0;
  pwLEDs[1] = 0;
  pwLEDs[2] = 0;
  pwLEDs[3] = 0;

  double br=10;
  for (i=1; i<100; i++) { // Fill brightness with 100 logarithmic steps from 1-900
    Brightness[i] = (int)br;
    // br  *= 1.0465;
    br  = br * 1.019 + 3;
  }
/*
  for (i=0; i<12; i++) {
    Serial << _BINZ(rawCharlie[i], 32) << endl << _BINZ(rawCharlieOE[i], 32) << endl << endl;
  }
*/
}


void loop() {
  static int i;
  static float pw=500, pwMul=1.002;

  drd->loop();
  
  printLocalTime();
  
  pw  *= pwMul;
  pwDigits=(int)pw;
  if (pwDigits>900 || pwDigits<10) pwMul = 1/pwMul;

  // Serial << charlieLEDs[0] << " " << charlieLEDs[1] << " " << pwLEDs[0] << " " << pwLEDs[1] << " " << ms/60 << endl;
  delay(50);
}
