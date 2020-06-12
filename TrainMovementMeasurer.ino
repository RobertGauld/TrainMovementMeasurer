/*
 * This code can be used under the BSD License (reproduced below).
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * - Neither the name of the project nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Train Movement Measurer
 * Uses three light gates to calculate the scale velocity and acceleration for a model train.
 * Maximum current draw is 0.5A (assuming a stick of 8 LEDs).
 * Full documentation for getting started with ths project can be found at
 * https://github.com/robertgauld/TrainMovementMeasurer
 * 
 * You should only need to make changes in the "User" defines section below, if you also model
 * in British N gauge and have built the tunnel and electronics as described then you shouldn't
 * need to make any changes. If things don't quite work for you search the code for "#define TEST_"
 * and you'll find some test modes which may help you trouble shoot. Simply uncomment the relevant
 * #define line and reload the program to the Arduino.
 * 
 * 
 * Three light gates placed along a track allow for two velocities to be measured. Knowing these two
 * velocities and time between them an acceleration can also be calculated, this is then scaled and displayed
 * on the screen.
 *                   |    <- DISTANCE_0 ->    |    <- DISTANCE_1 ->    |
 *          =========|========================|========================|========
 *                 Gate 0                   Gate 1                   Gate 2
 * 
 * You can use two light gates placed along a track you measure just velocity.
 * Set DISTANCE_1 to zero if you wish to do this.
 *                   |                 <- DISTANCE_0 ->               |
 *          =========|================================================|=========
 *                 Gate 0                                           Gate 1
 * 
 * 
 * Alternativly four occupancy blocks can be used to measure the two velocities and accelerations.
 *                   |    <- DISTANCE_0 ->    |    <- DISTANCE_1 ->    |
 *          =========|========================|========================|========
 *           Block 0          Block 1                   Block 2          Block 3
 * Or three can measure speed only in either direction, over the same block.
 *                          |         <- DISTANCE_0 ->         |
 *          ================|==================================|================
 *              Block 0                    Block 1                   Block 2
 * Or two can measure speed only in either direction, with each block wirking in one direction only.
 *                   |    <- DISTANCE_0 ->    |    <- DISTANCE_1 ->    |
 *                   |  Speed measured ---->  |  <---- Speed measured  |
 *          =========|========================|========================|========
 *                            Block 0                   Block 1
 * 
 * 
 * After calculating the scale velocity and acceleration are displayed on the right had side of the screen
 * and output to the serial port. This information will remain on the screen for INTER_TRAIN_DELAY
 * milliseconds (originally 15000) or until the train has completly left the tunnel (whichever is longer).
 * 
 * A series of addressable LEDs can also be attached which will show:
 *   - During timing:
 *     ~ The state of the gates (first 3 LEDs) whilst waiting for or timing a train [Green is clear, Red is blocked].
 *     ~ The state of the captured times (last 3 LEDs) [Yellow is uncaptured, blue is captured].
 *     ~ The direction of the trains travel (last half of the middle for 0->1->2, first half for 2->1->0).
 *   - A velocity bar, controlled by velocityBarValues[].
 * 
 * The onboard LED has three states:
 *   - On - Running setup
 *   - Flashing (2Hz) - Terminal error
 *   - Off - Running normally
 */


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>


// User defines
#define DISTANCE_0    100           // Distance 0 (millimeters) from the above diagrams
#define DISTANCE_1    100           // Distance 1 (millimeters) from the above diagrams
#define SCALE         148           // Scale for calculating scale velocity/acceleration (1:SCALE)
#undef VELOCITY_ONLY                // Whether to measure only velocity (requires less triggers)


// Defines for triggers
const PROGMEM uint8_t TRIGGER_PINS[] = {5, 4, 3}; // Trigger pins
#define TRIGGER_COUNT    3          // The number of pins in the above list
#define TRIGGER_MODE     INPUT      // Trigger type (INPUT or INPUT_PULLUP)
#define TRIGGER_DETECT   HIGH       // State of the trigger when a train is detected (HIGH or LOW)
#define TRIGGER_GATES               // What sort of triggers you're using (TRIGGER_GATES or TRIGGER_BLOCKS)
#define INTER_TRAIN_DELAY  15000    // Display the train's measaurements for at least this long (milliseconds)


// Defines for light gates
// Comment out both these settings if you're not using light gates
#define GATE_LED_PIN  11            // Output pin for controlling the light gate LEDs
#define GATE_LED_RATE 0             // Rate at which to flash the light gate LEDs (Hz, 62-8000000, 0 = constantly on)


// Defines for LED stick
#define LEDS_BRIGHT   24            // How bright to have the LEDs (0 - 255)
#define LEDS_PIN      12            // Data pin for the LEDs
#define LEDS_COUNT    8             // Number of LEDs (must be at least 8)
                                    // be sure to adjust the velocity bars below too !!!
#undef LEDS_REVERSE                 // Reverse the direction of the LED numbering, such that 0 is on the right not the left
#define LEDS_TYPE     WS2812        // Type of LEDs
#define LEDS_ORDER    GRB           // Color order of LED data
CRGB leds[LEDS_COUNT];


// Defines for screen - you should only need to change SCREEN_WIDTH and/or SCREEN_HEIGHT
// It's intended that an ADAfruit OLED will be used - either 128x64 or 128x32 although 96x64 or 64x32 (not advised!) should work too.
#define SCREEN_WIDTH       128     // Height of screen in pixels (must be at least 64)
#define SCREEN_HEIGHT      64      // Width of screen in pixels (must be at least 32)
#define SCREEN_ROTATION    0       // Adjust the screen rotation in steps of 90deg (0 - 3), if rotating by 1 or 3 swap _WIDTH and _HEIGHT above
#define SCREEN_RESET_PIN   -1      // Reset pin (or -1 if sharing Arduino reset pin)
#define SCREEN_I2C_ADDRESS 0x3C    // Screen's I2C address (if using Adafruit's 128x64 then it's 0x3D)
#define SCREEN_CHAR_WIDTH  6       // Character height on OLED display
#define SCREEN_CHAR_HEIGHT 8       // Character height on OLED display


// Defines and bitmap data for the logo
#define LOGO_HEIGHT 29
#define LOGO_WIDTH  64
const PROGMEM unsigned char logoBmp[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 
  0x03, 0x04, 0x1f, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x06, 0x04, 0x0f, 0xff, 0xff, 0xff, 0xfc, 0x00, 
  0x0c, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1a, 0x00, 0x0b, 0x30, 0x06, 0x00, 0x63, 0x80, 
  0x11, 0x00, 0x11, 0x30, 0x06, 0x00, 0x61, 0xc0, 0x20, 0x00, 0x00, 0xb0, 0x06, 0x00, 0x60, 0xe0, 
  0x20, 0x00, 0x40, 0xb0, 0x06, 0x00, 0x60, 0x70, 0x60, 0x00, 0x80, 0xf0, 0x06, 0x00, 0x60, 0x38, 
  0x40, 0x01, 0x00, 0x70, 0x06, 0x00, 0x60, 0x1c, 0x40, 0x02, 0x00, 0x70, 0x06, 0x00, 0x60, 0x0e, 
  0x78, 0x04, 0x03, 0xf0, 0x06, 0x00, 0x60, 0x0f, 0x40, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 
  0x40, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xfe, 
  0x20, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x20, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xf8, 
  0x11, 0x00, 0x11, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1a, 0x00, 0x0b, 0xff, 0xff, 0xff, 0xff, 0xf0, 
  0x0c, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x0c, 0x00, 0x00, 0x07, 0xff, 0x80, 
  0x03, 0x00, 0x18, 0x00, 0x00, 0x03, 0xcf, 0x00, 0x00, 0xe0, 0xe0, 0x00, 0x00, 0x01, 0x86, 0x00, 
  0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Text to put beside logo - maximum 3 lines of 10 chars.
const PROGMEM char BANNER_LINE_0[] = "Train";
const PROGMEM char BANNER_LINE_1[] = "Movement";
const PROGMEM char BANNER_LINE_2[] = "Measurer";
const PROGMEM char *const BANNER_LINES[] = {BANNER_LINE_0, BANNER_LINE_1, BANNER_LINE_2};


// Defines for velocity bars
const PROGMEM byte velocityBar[LEDS_COUNT * 4] = {
  // velocity, red, green, blue
  0,   0, 255, 0,
  20,  0, 255, 0,
  40,  0, 255, 0,
  60,  0, 255, 255,
  80,  0, 255, 255,
  100, 255, 255, 0,
  120, 255, 255, 0,
  140, 255, 0, 0
};


/* Use test LEDs mode.
 * Line 1: The current test and stage of the test:
 *          - Scrolling and the number of white LEDs before the red one
 *          - Brightning and the brightness
 * Line 3: LED Strip pin, count, brightness
 * The LEDs first show Red, Green, Blue, Yellow, Magenta, Cyan with the rest White
 * this scrolls off the end of the strip. You then get Black, Red, Green, Blue, Yellow,
 * Magenta, Cyan, White and the rest Black with the brightness changing.
 * If the colours are wrong then you'll need to adjust the line #define LEDS_ORDER RGB
*/
//#define TEST_LEDS

/* Use test triggers mode.
 * Line 1:  Distance 0 and Distance 1
 * Line 2:  LED control pin and constant/frequency of IR LEDs (if light gates are being used)
 * Line 3:  triggers 0, .. n pins (black on white = input blocked, white on black = input unblocked)
 * LED 0:   Trigger 0 state (green = never blocked, red = blocked, blue = unblocked (but previously has been))
 * LED 1:   Trigger 1 state (green = never blocked, red = blocked, blue = unblocked (but previously has been))
 * LED 2:   Trigger 2 state (green = never blocked, red = blocked, blue = unblocked (but previously has been))
 * LED 3:   Trigger 3 state (green = never blocked, red = blocked, blue = unblocked (but previously has been))
 * LED 4:
 * LED 5:
 * LED 6+7: Direction determination
*/
//#define TEST_TRIGGERS


/* Pins used:
 *  Digital 0:  Serial RX
 *  Digital 1:  Serial TX
 *  Digital 2:  TRIGGER_3_PIN
 *  Digital 3:  TRIGGER_2_PIN
 *  Digital 4:  TRIGGER_1_PIN
 *  Digital 5:  TRIGGER_0_PIN
 *  Digital 6:  
 *  Digital 7:  
 *  Digital 8:  software uartable
 *  Digital 9:  software uartable
 *  Digital 10: 
 *  Digital 11: Light gates LEDs
 *  Digital 12: LED Stick
 *  Digital 13: LED_BUILTIN
 *  Analog  0:  
 *  Analog  1:  
 *  Analog  2:  
 *  Analog  3:  
 *  Analog  4:  I2C Data
 *  Analog  5:  I2C Clock
 *  Analog  6:  
 *  Analog  7:  
 */

// Setup screen
#if SCREEN_HEIGHT >= 64
  #define SCREEN_LINE_0 (SCREEN_HEIGHT - (4 * SCREEN_CHAR_HEIGHT))  // Pixel position for top of 1st line at text size 1
#else
  #define SCREEN_LINE_0 0                                           // Pixel position for top of 1st line at text size 1
#endif
#define SCREEN_LINE_1 (SCREEN_LINE_0 + SCREEN_CHAR_HEIGHT)          // Pixel position for top of 2nd line at text size 1
#define SCREEN_LINE_2 (SCREEN_LINE_1 + SCREEN_CHAR_HEIGHT)          // Pixel position for top of 3rd line at text size 1
#define SCREEN_LINE_3 (SCREEN_LINE_2 + SCREEN_CHAR_HEIGHT)          // Pixel position for top of 4th line at text size 1

#if SCREEN_ROTATION == 1 || SCREEN_ROTATION == 3
  Adafruit_SSD1306 display(SCREEN_HEIGHT, SCREEN_WIDTH, &Wire, SCREEN_RESET_PIN);
#else
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, SCREEN_RESET_PIN);
#endif

// Setup LEDs
#ifdef LEDS_REVERSE
  #define LED_INDEX(i) (i)
  #define LED_RINDEX(i) (LEDS_COUNT - (i) - 1)
#else
  #define LED_INDEX(i) (LEDS_COUNT - (i) - 1)
  #define LED_RINDEX(i) (i)
#endif


// Final check of settings above
#if DISTANCE_0 < 1
  #error "DISTANCE_0 must be at least 1"
#endif
#if (!defined(VELOCITY_ONLY) || defined(TRIGGER_BLOCKS)) && DISTANCE_1 < 1 // required if using blocks or measuring acceleration
  #error "DISTANCE_1 must be at least 1"
#endif
#if !defined(TRIGGER_GATES) && !defined(TRIGGER_BLOCKS)
  #error "Either TRIGGER_GATES or TRIGGER_BLOCKS must be defined"
#endif
#if defined(TRIGGER_GATES) && defined(TRIGGER_BLOCKS)
  #error "Only one of TRIGGER_GATES or TRIGGER_BLOCKS may be defined"
#endif
#if LEDS_COUNT < 8
  #error "There must be at least 8 addressable LEDs connected"
#endif
#if SCREEN_WIDTH < 64
  #error "Screen must be at least 64 pixels wide"
#endif
#if SCREEN_HEIGHT < 32
  #error "Screen must be at least 32 pixels high"
#endif
#if !defined(GATE_LED_RATE) && GATE_LED_RATE != 0
  #if GATE_LED_RATE < 62
    #error "Light Gate LED frequency must be above 62Hz"
  #endif
  #if GATE_LED_RATE > 500000
    #error "Light Gate LED frequency must be below 500KHz"
  #endif
#endif


void setup()  {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(9600, SERIAL_8N1);
  FastLED.addLeds<LEDS_TYPE, LEDS_PIN, LEDS_ORDER>(leds, LEDS_COUNT);
  FastLED.clear();
  FastLED.setBrightness(LEDS_BRIGHT);
  FastLED.show();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDRESS)) {
    terminalError(F("ERROR: SSD1306 allocation failed"), false);
  }
  display.setRotation(SCREEN_ROTATION);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setTextWrap(false);
  display.drawBitmap((SCREEN_WIDTH - LOGO_WIDTH)/2, (SCREEN_HEIGHT - LOGO_HEIGHT)/2, logoBmp, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
  display.display();

  extraSetup();

  Serial.print(F("READY: SCALE:1:"));
  Serial.print(SCALE);
  Serial.print(F(" DISTANCE_0:"));
  Serial.print(DISTANCE_0);
  Serial.print(F("mm DISTANCE_1:"));
  Serial.print(DISTANCE_1);
  Serial.print(F("mm INTER_TRAIN_DELAY:"));
  Serial.print(INTER_TRAIN_DELAY);
  #ifdef VELOCITY_ONLY
    Serial.print(F("ms MEASURING:VELOCITY"));
  #else
    Serial.print(F("ms MEASURING:ACCELERATION_VELOCITY"));
  #endif
  Serial.print(F(" TRIGGERS:"));
  Serial.print(TRIGGER_COUNT);
  #ifdef TRIGGER_GATES
    Serial.print(F("_GATES"));
  #endif
  #ifdef TRIGGER_BLOCKS
    Serial.print(F("_BLOCKS"));
  #endif
  Serial.println();

  afterSetupAnimation();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  #ifdef TEST_LEDS
    testLeds();
  #endif
  #ifdef TEST_TRIGGERS
    testTriggers();
  #endif

  #ifdef VELOCITY_ONLY
    doVelocity();
  #else
    doVelocityAcceleration();
  #endif
}

void testLeds() {
  Serial.println(F("STATUS: Testing LEDs."));

  // Maximum width of 64 (10 characters)
  const unsigned int col0 = 0;
  const unsigned int col1 = SCREEN_WIDTH / 3;
  const unsigned int col2 = col1 + col1;
  const byte brightnesses[] = {1, 2, 4, 8, 16, 24, 32, 64, 96, 128, 192, 255};

  display.setTextSize(1);
  FastLED.clear();

  // Dispaly line 0: Title
  display.fillRect(0, SCREEN_LINE_0, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_WHITE);
  display.setCursor((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 9)) / 2, SCREEN_LINE_0);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.print(F("TEST LEDs"));
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  // Display line 3: LED Strip pin, count and brightness
  display.setCursor(col0, SCREEN_LINE_3);
  display.print(LEDS_PIN);
  display.setCursor(col1, SCREEN_LINE_3);
  display.print(LEDS_COUNT);
  display.setCursor(col2, SCREEN_LINE_3);
  display.print(LEDS_BRIGHT);

  display.display();

  while(true) {
    display.setCursor(col0, SCREEN_LINE_1);
    display.print(F("Scrolling      "));
    FastLED.setBrightness(LEDS_BRIGHT);
    for(int i = 0; i <= LEDS_COUNT; i++) {
      display.setCursor(66, SCREEN_LINE_1);
      display.print(i);
      display.display();
      for(int j = 0; j < i; j++) {leds[LED_INDEX(j)] = CRGB::White;}
      if(i < LEDS_COUNT - 0) {leds[LED_INDEX(i + 0)] = CRGB::Red;}
      if(i < LEDS_COUNT - 1) {leds[LED_INDEX(i + 1)] = CRGB::Green;}
      if(i < LEDS_COUNT - 2) {leds[LED_INDEX(i + 2)] = CRGB::Blue;}
      if(i < LEDS_COUNT - 3) {leds[LED_INDEX(i + 3)] = CRGB::Yellow;}
      if(i < LEDS_COUNT - 4) {leds[LED_INDEX(i + 4)] = CRGB::Magenta;}
      if(i < LEDS_COUNT - 5) {leds[LED_INDEX(i + 5)] = CRGB::Cyan;}
      for(int j = i + 6; j < LEDS_COUNT; j++) {leds[LED_INDEX(j)] = CRGB::White;}
      FastLED.show();
      delay(500);
    }

    display.setCursor(col0, SCREEN_LINE_1);
    display.print(F("Brightning     "));
    for(int i = 0; i < 12; i++) {
      display.setCursor(66, SCREEN_LINE_1);
      display.print(brightnesses[i]);
      display.display();
      FastLED.clear();
      FastLED.setBrightness(brightnesses[i]);
      leds[LED_INDEX(0)] = CRGB::Black;
      leds[LED_INDEX(1)] = CRGB::Red;
      leds[LED_INDEX(2)] = CRGB::Green;
      leds[LED_INDEX(3)] = CRGB::Blue;
      leds[LED_INDEX(4)] = CRGB::Yellow;
      leds[LED_INDEX(5)] = CRGB::Magenta;
      leds[LED_INDEX(6)] = CRGB::Cyan;
      leds[LED_INDEX(7)] = CRGB::White;
      FastLED.show();
      delay(750);      
    }
  }
}

void testTriggers() {
  Serial.println(F("STATUS: Testing triggers."));

  // Maximum width of 64 (10 characters)
  const unsigned int col0 = 0;
  const unsigned int col1 = SCREEN_WIDTH / 2;

  #ifdef TRIGGER_GATES
    irLeds(true);
  #endif
  display.setTextSize(1);
  FastLED.clear();

  // Display line 0: Title
  display.fillRect(0, SCREEN_LINE_0, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_WHITE);
  display.setCursor((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 15)) / 2, SCREEN_LINE_0);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.print(F("TEST TRIGGERS"));
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  // Display line 1: Distance 0->1 and 1->2
  display.setCursor(col0, SCREEN_LINE_1);
  display.print(DISTANCE_0);
  display.setCursor(col1, SCREEN_LINE_1);
  display.print(DISTANCE_1);

  // Display Line 2: Light gate IR LED pin constant/flash rate
  #ifdef TRIGGER_GATES
    #ifdef GATE_LED_PIN
      display.setCursor(col0, SCREEN_LINE_2);
      display.print(GATE_LED_PIN);
      display.setCursor(col1, SCREEN_LINE_2);
      #if GATE_LED_RATE == 0
        display.print(F("Constant"));
      #else
        display.print(GATE_LED_RATE / 1000.0);
        display.print(F("KHz"));
      #endif
    #else
      display.setCursor(col0, SCREEN_LINE_2);
      display.print(F("No LED control set."));
    #endif
  #endif

  boolean triggered[TRIGGER_COUNT] = {};  // Whether a given input has ever been triggered
  while(true) {
    // Display
    for(unsigned int i = 0; i < TRIGGER_COUNT; i++) {
      // Show pin number and state of trigger on the display
      if(triggerDetect(i)) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      } else {
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      }
      display.setTextSize(1);
      display.setCursor((col0 + (SCREEN_CHAR_WIDTH * (3 * i))), SCREEN_LINE_3);
      display.print(pgm_read_byte(TRIGGER_PINS + i));
    }

    // Show direction indication
    leds[LED_RINDEX(1)] = triggerDetect(0) ? CRGB::Magenta : CRGB::Black;
    leds[LED_RINDEX(0)] = triggerDetect(TRIGGER_COUNT - 1) ? CRGB::Magenta : CRGB::Black;

    // LEDs
    for(unsigned int i = 0; i < TRIGGER_COUNT; i++) {
      // Show current state of the light gate on LED strip
      if(triggerDetect(i)) {
        leds[LED_INDEX(TRIGGER_COUNT - 1 - i)] =  CRGB::Red;
        triggered[i] = true;
      } else {
        leds[LED_INDEX(TRIGGER_COUNT - 1 - i)] = triggered[i] ? CRGB::Blue : CRGB::Green;        
      }
    }
    display.display();
    FastLED.show();
  }
}

// Calculate results and output to screen, LEDs and serial
// Both times in milliseconds, both distances in millimeters
// Then wait for INTER_TRAIN_DELAY ms
void resultsVelocityAcceleration(unsigned long time1, unsigned long time2, unsigned int distance1, unsigned int distance2) {
  updateStatus("Calculating");
  const unsigned long time = time1 + time2;
  const float velocity = ((distance1 + distance2) / 1000.0) / (time / 1000.0);  // m/s
  const float velocity1 = (distance1 / 1000.0) / (time1 / 1000.0);              // m/s
  const float velocity2 = (distance2 / 1000.0) / (time2 / 1000.0);              // m/s
  const float deltaV = velocity2 - velocity1;                                   // m/s
  const float deltaT = time / 2000.0;                                           // s   (0.5*t1 + 0.5*t2 is average of both times)
  const float acceleration = deltaV / deltaT;                                   // m/s2
  const float scaleVelocity = velocity * 2.236936 * SCALE;                      // mph
  const float scaleAcceleration = acceleration * 2.236936 * SCALE;              // mph/s

  // Show results
  const unsigned int colCalculatedNumber = SCREEN_WIDTH - 64;
  const unsigned int colCalculatedUnit = colCalculatedNumber + (3 * 2 * SCREEN_CHAR_WIDTH);
  clearSubScreen();
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  // Scale Velocity
  display.setTextSize(1);
  display.setCursor(colCalculatedUnit, SCREEN_LINE_1);
  display.print(F("mph"));
  display.setTextSize(2);
  display.setCursor((scaleVelocity > 10 && scaleVelocity < 100 ? (colCalculatedNumber + (2 * SCREEN_CHAR_WIDTH)) : colCalculatedNumber), SCREEN_LINE_0);
  display.print(scaleVelocity, (abs(scaleVelocity) < 10 ? 1 : 0));
  // Scale Accelleration
  display.setTextSize(1);
  display.setCursor(colCalculatedUnit, SCREEN_LINE_3);
  display.print(F("mph/s"));
  display.setTextSize(2);
  if (scaleAcceleration < 0) {
    unsigned int col = abs(scaleAcceleration) >= 10 && abs(scaleAcceleration) < 100 ? (colCalculatedNumber + 4) : (colCalculatedNumber - 8);
    display.fillRect(col, (SCREEN_LINE_3 - 1), 6, 2, SSD1306_WHITE);
  }
  display.setCursor((abs(scaleAcceleration) >= 10 && abs(scaleAcceleration) < 100 ? (colCalculatedNumber + (2 * SCREEN_CHAR_WIDTH)) : colCalculatedNumber), SCREEN_LINE_2);
  display.print(abs(scaleAcceleration), (abs(scaleAcceleration) < 10 ? 1 : 0));
  display.display();

  // Output results to serial port
  // VA_DATA: <scale> <distance 1> <distance 2> <direction> <total time> <1st pair time> <2nd pair time>
  // <total velocity> <1st pair velocity> <2nd pair velocity> <acceleration>
  // <scale velocty metric> <scale acceleration metric> <scale velocty imperial> <scale acceleration imperial>
  Serial.print(F("VA_DATA: 1:"));
  Serial.print(SCALE);
  Serial.print(F(" "));
  Serial.print(distance1);
  Serial.print(F("mm "));
  Serial.print(distance2);
  Serial.print(F("mm "));
  Serial.print(time);
  Serial.print(F("ms "));
  Serial.print(time1);
  Serial.print(F("ms "));
  Serial.print(time2);
  Serial.print(F("ms "));
  Serial.print(velocity, 2);
  Serial.print(F("m/s "));
  Serial.print(velocity1, 2);
  Serial.print(F("m/s "));
  Serial.print(velocity2, 2);
  Serial.print(F("m/s "));
  Serial.print(acceleration, 2);
  Serial.print(F("m/s/s "));
  Serial.print((velocity * 3.60 * SCALE), 2);
  Serial.print(F("km/h "));
  Serial.print((acceleration * 3.60 * SCALE), 2);
  Serial.print(F("km/h/s "));
  Serial.print(scaleVelocity, 2);
  Serial.print(F("mph "));
  Serial.print(scaleAcceleration, 2);
  Serial.println(F("mph/s "));

  showVelocityBar(scaleVelocity);
  delay(INTER_TRAIN_DELAY);
}

// Calculate results and output to screen, LEDs and serial
// time in milliseconds
// Then wait for INTER_TRAIN_DELAY ms
void resultsVelocity(unsigned long time, unsigned int distance) {
  updateStatus("Calculating");
  const float velocity = (distance / 1000.0) / (time / 1000.0); // m/s
  const float scaleVelocity = velocity * 2.236936 * SCALE;        // mph

  // Show results
  const unsigned int colCalculatedNumber = SCREEN_WIDTH - 64;
  const unsigned int colCalculatedUnit = colCalculatedNumber + (3 * 2 * SCREEN_CHAR_WIDTH);
  clearSubScreen();
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  // Scale Velocity
  display.setTextSize(1);
  display.setCursor(colCalculatedUnit, SCREEN_LINE_2);
  display.print(F("mph"));
  display.setTextSize(2);
  display.setCursor((scaleVelocity > 10 && scaleVelocity < 100 ? (colCalculatedNumber + (2 * SCREEN_CHAR_WIDTH)) : colCalculatedNumber), SCREEN_LINE_1);
  display.print(scaleVelocity, (abs(scaleVelocity) < 10 ? 1 : 0));
  display.display();

  // Output results to serial port
  // V_DATA: <scale> <distance 1> <distance 2> <direction> <total time> <1st pair time> <2nd pair time>
  // <total velocity> <1st pair velocity> <2nd pair velocity> <acceleration>
  // <scale velocty metric> <scale acceleration metric> <scale velocty imperial> <scale acceleration imperial>
  Serial.print(F("V_DATA: 1:"));
  Serial.print(SCALE);
  Serial.print(F(" "));
  Serial.print(distance);
  Serial.print(F("mm "));
  Serial.print(time);
  Serial.print(F("ms "));
  Serial.print(velocity, 2);
  Serial.print(F("m/s "));
  Serial.print((velocity * 3.60 * SCALE), 2);
  Serial.print(F("km/h "));
  Serial.print(scaleVelocity, 2);
  Serial.println(F("mph "));

  showVelocityBar(scaleVelocity);
  delay(INTER_TRAIN_DELAY);
}

// Show velocity on LEDs
void showVelocityBar(float scaleVelocity) {
  FastLED.clear();
  for (unsigned int i = 0; i < LEDS_COUNT; i++) {
    const byte velocity = pgm_read_byte(velocityBar + (i * 4) + 0);
    const byte red =      pgm_read_byte(velocityBar + (i * 4) + 1);
    const byte green =    pgm_read_byte(velocityBar + (i * 4) + 2);
    const byte blue =     pgm_read_byte(velocityBar + (i * 4) + 3);
    if(scaleVelocity > velocity) {
      leds[LED_RINDEX(i)] = CRGB(red, green, blue);
    }
  }
  FastLED.show();
}

void updateStatus(String status) {
  int col = (SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * status.length())) / 2;
  if(col > SCREEN_WIDTH) { col = 0; }

  display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setCursor(col, SCREEN_LINE_3);
  display.print(status);
  display.display();

  Serial.print(F("STATUS: "));
  Serial.print(status);
  Serial.println(F("."));
}

void showError(String message) {
  int col = (SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * message.length())) / 2;
  if(col > SCREEN_WIDTH) { col = 0; }

  display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(col, SCREEN_LINE_3);
  display.print(message);
  display.display();

  Serial.print(F("ERROR: "));
  Serial.print(message);
  Serial.println(F("."));
}

void terminalError(String message) {
  terminalError(message, true);
}
void terminalError(String message, boolean useDisplay) {
  if(useDisplay) {
    display.clearDisplay();
    display.dim(false);
    display.setTextWrap(true);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, 0);
    display.print(message);
  }

  Serial.print(F("ERROR: "));
  Serial.print(message);
  Serial.println(F("."));

  for(boolean inverted = false; true; inverted = !inverted) {
    if(useDisplay) {
      display.invertDisplay(inverted);
      display.display();
    }

    for(unsigned int i = 0; i < LEDS_COUNT; i++) {
      leds[i] = (i % 2) == inverted ? CRGB::Red : CRGB::White;
    }
    FastLED.show();

    digitalWrite(LED_BUILTIN, !inverted);
    delay(250);
  }
}

void afterSetupAnimation() {
  if(SCREEN_HEIGHT < 64) { return; }

  int top = (display.height() - LOGO_HEIGHT) / 2;
  int left = (display.width() - LOGO_WIDTH) / 2;
  int target_top = 0;
  int target_left = (SCREEN_WIDTH >= 128) ? 0 : (SCREEN_WIDTH - LOGO_WIDTH) / 2;
  int delta_top = (top - target_top) / 8;
  int delta_left = (left - target_left) / 8;

  while (top > target_top || left > target_left) {
    top = constrain(top - delta_top, 0, SCREEN_HEIGHT - LOGO_HEIGHT);
    left = constrain(left - delta_left, 0, SCREEN_WIDTH - LOGO_WIDTH);
    display.clearDisplay();
    display.drawBitmap(left, top, logoBmp, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    display.display();
    delay(25);
  }

  if(SCREEN_WIDTH >= 128) {
    display.setTextSize(1);
    int lineSpacing = (LOGO_HEIGHT - (3 * SCREEN_CHAR_HEIGHT)) / 4;
    top = -SCREEN_CHAR_HEIGHT - (lineSpacing / 2);
    left = SCREEN_WIDTH - (10 * SCREEN_CHAR_WIDTH);
    char line[11];  // Buffer for holding each line - maximum length of 10 + 1 for teminating null byte

    display.setTextColor(SSD1306_WHITE);
    for(int i = 0; i < 3; i++) {
      top += SCREEN_CHAR_HEIGHT + lineSpacing;
      display.setCursor(left, top);

      strcpy_P(line, (char*)pgm_read_word(&(BANNER_LINES[i]))); // Read line from PROGMEM
      // Then displat the line once character at a time
      for(int j = 0; (j < 10 && line[j] != 0); j++) {
        display.print(line[j]);
        display.display();
        delay(10);
      }
    }
  }
}

// Clear the part of the screen not used by the top banner.
void clearScreen() {
  display.fillRect(0, SCREEN_LINE_0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_BLACK);
}

// Clear the right 64 pixel width of the screen not used by the banner and bottom line
void clearSubScreen() {
  display.fillRect((SCREEN_WIDTH - 64), SCREEN_LINE_0, 64, SCREEN_HEIGHT, SSD1306_BLACK);
  display.fillRect(0, SCREEN_LINE_3, 64, SCREEN_HEIGHT, SSD1306_BLACK);
}

// Check if a gate/block is detecting a train.
boolean triggerDetect(byte trigger) {
  return digitalRead(pgm_read_byte(TRIGGER_PINS + trigger)) == TRIGGER_DETECT;
}

// Check if a train is present anywhere in the gates/blocks.
boolean trainPresent() {
  for(unsigned int i = 0; i < TRIGGER_COUNT; i++) {
    if(triggerDetect(i)) {
      return true;
    }
  }
  return false;
}

// Draw the state of all gates/blocks on the screen.
void drawTriggerStates() {
  const uint16_t spacing = SCREEN_WIDTH / (TRIGGER_COUNT + 1);
  for(unsigned int i = 0; i < TRIGGER_COUNT; i++) {
    drawTriggerState(i, (spacing * (i + 1)), (SCREEN_LINE_2 - 1));
  }
}

// Show the state of all gates/blocks on the LEDs.
void showTriggerStates() {
  for(unsigned int i = 0; i < TRIGGER_COUNT; i++) {
    leds[LED_INDEX(TRIGGER_COUNT - 1 - i)] = triggerDetect(i) ? CRGB::Red : CRGB::Green;
  }
  FastLED.show();
}

// Draw the gate/block as a circle on the display
void drawTriggerState(byte trigger, uint16_t x, uint16_t y) {
  const uint16_t radiusOuter = SCREEN_CHAR_HEIGHT - 1;
  const uint16_t radiusInner = radiusOuter - 1;

  display.fillCircle(x, y, radiusOuter, SSD1306_WHITE);
  if(triggerDetect(trigger)) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  } else {
    display.fillCircle(x, y, radiusInner, SSD1306_BLACK);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  display.setTextSize(1);
  display.setCursor((x - 2), (y - 3));
  display.print(trigger);
}


// These functions must be implemented by the code for handling different timing methods.
//void extraSetup();              // Any extra setup that's needed
//void doVelocityAcceleration();  // Get 2 times in ms, 2 distances in mm and call resultsVelocityAcceleration(time1, time2, distance1, distance2);
//void doVelocity();              // Get 1 time in msand call resultsVelocity(time);


/*
 * (LIGHT) GATE ONLY STUFF
 */
#ifdef TRIGGER_GATES

  #ifdef VELOCITY_ONLY
    #if TRIGGER_COUNT != 2
      #error "There must be 2 triggers to measure velocity in gate mode"
    #endif
  #else
    #if TRIGGER_COUNT != 3
      #error "There must be 3 triggers to measure velocity and acceleration in gate mode"
    #endif
  #endif

/*
 *          |    <- DISTANCE_0 ->    |    <- DISTANCE_1 ->    |
 * =========|========================|========================|========
 *        Gate 0                   Gate 1                   Gate 2
 */
  void doVelocityAcceleration() {
    unsigned long times[3];     // The value of millis() when the first, second and third light gate was triggered
    unsigned int distances[2];  // The distances to use in calculations (assigned based on direction of travel)
    irLeds(true);
    clearScreen();
    FastLED.clear();

    // Ensure all light gates are clear
    // Once they're all clear ensure they stay that way for at least INTER_TRAIN_DELAY milliseconds
    if (trainPresent()) {
      showError(F("Sensors not clear"));
      unsigned long continueAt = -1;
      while (continueAt > millis()) {
        if (trainPresent()) {
          continueAt = millis() + INTER_TRAIN_DELAY;
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_WHITE);
          display.setTextSize(1);
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 17)) / 2), SCREEN_LINE_3);
          display.print(F("Clear the sensors"));
        } else {
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_BLACK);
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 16)) / 2), SCREEN_LINE_3);
          display.print(F("Please wait "));
          display.print((continueAt - millis()) / 1000.0, 1);
        }
        drawTriggerStates();
        display.display();
        showTriggerStates();
      }
      clearScreen();
    }

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, SCREEN_LINE_0);
    display.print(F("1:"));
    display.print(SCALE);

    updateStatus(F("Waiting for train"));
    leds[LED_RINDEX(0)] = CRGB::Yellow;
    leds[LED_RINDEX(1)] = CRGB::Yellow;
    leds[LED_RINDEX(2)] = CRGB::Yellow;
    FastLED.show();

    // Wait for a train to enter
    while (!triggerDetect(0) && !triggerDetect(2)) {showTriggerStates();}
    times[0] = millis();
    const byte endGate = triggerDetect(0) ? 2 : 0;
    if(endGate == 2) {
      // Train is moving 0 -> 1 -> 2
      for(unsigned int i = 3; i < LEDS_COUNT / 2; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
      distances[0] = DISTANCE_0;
      distances[1] = DISTANCE_1;
    } else {
      // Train is moving 2 -> 1 -> 0
      for(unsigned int i = LEDS_COUNT / 2; i < LEDS_COUNT - 3; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
      distances[0] = DISTANCE_1;
      distances[1] = DISTANCE_0;
    }
    leds[LED_RINDEX(0)] = CRGB::Blue;
    FastLED.show();
    updateStatus("Timing train");

    // Wait for the train to reach the middle
    while(!triggerDetect(1)) { showTriggerStates(); }
    times[1] = millis();
    leds[LED_RINDEX(1)] = CRGB::Blue;
    FastLED.show();

    // Wait for the train to reach the end
    while(!triggerDetect(endGate)) { showTriggerStates(); }
    times[2] = millis();
    leds[LED_RINDEX(2)] = CRGB::Blue;
    FastLED.show();

    // Calculate and display results
    irLeds(false);
    resultsVelocityAcceleration((times[1] - times[0]), (times[2] - times[1]), distances[0], distances[1]);
    irLeds(true);

    // Wait for light gates to be clear
    while (trainPresent());
  }

/*
 *          |    <- DISTANCE_0 ->    |
 * =========|========================|========
 *        Gate 0                   Gate 1
 */
  void doVelocity() {
    unsigned long times[2];     // The value of millis() when the first and second light gate was triggered
    irLeds(true);
    clearScreen();
    FastLED.clear();

    // Ensure all light gates are clear
    // Once they're all clear ensure they stay that way for at least INTER_TRAIN_DELAY milliseconds
    if (trainPresent()) {
      showError(F("Sensors not clear"));
      unsigned long continueAt = -1;
      while(continueAt > millis()) {
        if (trainPresent()) {
          continueAt = millis() + INTER_TRAIN_DELAY;
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_WHITE);
          display.setTextSize(1);
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 17)) / 2), SCREEN_LINE_3);
          display.print(F("Clear the sensors"));
        } else {
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_BLACK);
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 16)) / 2), SCREEN_LINE_3);
          display.print(F("Please wait "));
          display.print((continueAt - millis())/1000.0, 1);
        }
        drawTriggerStates();
        display.display();
        showTriggerStates();
      }
      clearScreen();
    }

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, SCREEN_LINE_0);
    display.print(F("1:"));
    display.print(SCALE);

    updateStatus(F("Waiting for train"));
    leds[LED_RINDEX(0)] = CRGB::Yellow;
    leds[LED_RINDEX(1)] = CRGB::Yellow;
    FastLED.show();

    // Wait for a train to enter
    while (!triggerDetect(0) && !triggerDetect(1)) {showTriggerStates();}
    times[0] = millis();
    const byte endGate = triggerDetect(0) ? 1 : 0;
    if(endGate == 1) {
      // Train is moving 0 -> 1
      for(unsigned int i = 2; i < LEDS_COUNT / 2; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
    } else {
      // Train is moving 1 -> 0
      for(unsigned int i = LEDS_COUNT / 2; i < LEDS_COUNT - 2; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
    }
    leds[LED_RINDEX(0)] = CRGB::Blue;
    FastLED.show();
    updateStatus("Timing train");

    // Wait for the train to reach the end
    while(!triggerDetect(endGate)) { showTriggerStates(); }
    times[1] = millis();
    leds[LED_RINDEX(1)] = CRGB::Blue;
    FastLED.show();

    // Calculate and display results
    irLeds(false);
    resultsVelocity((times[1] - times[0]), DISTANCE_0);
    irLeds(true);

    // Wait for light gates to be clear
    while (trainPresent());
  }

  // Setup the LEDs to flash at provided rate (0 = constantly on)
  // Usable range is 62Hz to 16MHz
  // Taken (and adapted) from https://github.com/blippy/rpi/blob/master/timer/README.md
  void extraSetup() {
    #ifdef GATE_LED_PIN
      pinMode(GATE_LED_PIN, OUTPUT);

      if(GATE_LED_RATE == 0) {}   // The LEDs will be constantly on - no need to setup timers and switching
      else {
        //cli();                    // Stop interrupts
        static const unsigned int scales[] = {1, 8, 32, 64, 128, 256, 1024};

        TCCR2A = 0;
        TCCR2B = 0;
        TCNT2 = 0;

        TCCR2A |= _BV(WGM21);     // Turn on CTC mode
        // TIMSK2 |= _BV(OCIE2A);    // Enable timer compare interrupt
        // TCCR2A = TCCR2A | _BV(COM2A0);  // Toggle OC2A (pin 11) on compare match

        // Calculate prescaler and match register comparison
        unsigned long frequency = GATE_LED_RATE * 2; // We're actually interested in switching frequency not whole "wave" frequency
        unsigned int prescaler;
        for (prescaler = 0; prescaler < 8; prescaler++) if (scales[prescaler] >= (62500 / frequency)) break;
        OCR2A = (62500 / scales[prescaler]) * 256 / frequency; // Set Compare Match Register
        TCCR2B += ++prescaler; // the prescaler

        //sei();                    // Allow interrupts
      }
    #endif
  }

  // Toggle light gate LEDs when timer 2 overflows
  #if defined(GATE_LED_PIN) && GATE_LED_RATE > 0
    ISR(TIMER2_COMPA_vect) {
      static boolean state = true;
      digitalWrite(GATE_LED_PIN, state);
      state = !state;
    }
  #endif

  // Switch the IR LEDs on or off
  void irLeds(boolean state) {
    #ifdef GATE_LED_PIN
      #if GATE_LED_RATE > 0
        if(state) {
            TIMSK2 = TIMSK2 | _BV(OCIE2A);  // Enable timer 2 compare interrupt
        } else {
            TIMSK2 = TIMSK2 ^ _BV(OCIE2A);  // Disable timer 2 compare interrupt
        }
      #else
        digitalWrite(GATE_LED_PIN, state);
      #endif

      if(state) {delay(10);} // Give detectors time to settle once turned on
    #endif
  }
#endif


/*
 * BLOCK ONLY STUFF
 */
#ifdef TRIGGER_BLOCKS

  #ifdef VELOCITY_ONLY
    #if TRIGGER_COUNT != 2 && TRIGGER_COUNT != 3
      #error "There must be 2 or 3 triggers to measure velocity in block mode"
    #endif
  #else
    #if TRIGGER_COUNT != 4
      #error "There must be 4 triggers to measure velocity and acceleration in block mode"
    #endif
  #endif

  void extraSetup(){} // No extra setup required

  void doVelocity() {
    switch(TRIGGER_COUNT) {
      case 2 : doVelocity2();  // With 2 blocks
      case 3 : doVelocity3();  // With 3 blocks
    }
  }

/*
 *          |    <- DISTANCE_0 ->    |    <- DISTANCE_1 ->    |
 * =========|========================|========================|========
 *  Block 0          Block 1                   Block 2          Block 3
 */
  void doVelocityAcceleration(){
    unsigned long times[3];     // The value of millis() when the first, second and third light gate was triggered
    unsigned int distances[2];  // The distances to use in calculations (assigned based on direction of travel)
    clearScreen();
    FastLED.clear();

    // Ensure all light gates are clear
    // Once they're all clear ensure they stay that way for at least INTER_TRAIN_DELAY milliseconds
    if (trainPresent()) {
      showError(F("Sensors not clear"));
      unsigned long continueAt = -1;
      while (continueAt > millis()) {
        if (trainPresent()) {
          continueAt = millis() + INTER_TRAIN_DELAY;
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_WHITE);
          display.setTextSize(1);
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 17)) / 2), SCREEN_LINE_3);
          display.print(F("Clear the sensors"));
        } else {
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_BLACK);
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 16)) / 2), SCREEN_LINE_3);
          display.print(F("Please wait "));
          display.print((continueAt - millis()) / 1000.0, 1);
        }
        drawTriggerStates();
        display.display();
        showTriggerStates();
      }
      clearScreen();
    }

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, SCREEN_LINE_0);
    display.print(F("1:"));
    display.print(SCALE);

    updateStatus(F("Waiting for train"));
    leds[LED_RINDEX(0)] = CRGB::Yellow;
    leds[LED_RINDEX(1)] = CRGB::Yellow;
    leds[LED_RINDEX(2)] = CRGB::Yellow;
    FastLED.show();

    // Wait for a train to enter
    while (!triggerDetect(1) && !triggerDetect(2)) {showTriggerStates();}
    times[0] = millis();
    const byte endBlock = triggerDetect(0) ? 3 : 0;
    const byte nextBlock = endBlock == 3 ? 2 : 1;
    if(endBlock == 3) {
      // Train is moving 0 -> 1 -> 2 -> 3
      for(unsigned int i = 3; i < LEDS_COUNT / 2; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
      distances[0] = DISTANCE_0;
      distances[1] = DISTANCE_1;
    } else {
      // Train is moving 3 -> 2 -> 1 -> 0
      for(unsigned int i = LEDS_COUNT / 2; i < LEDS_COUNT - 3; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
      distances[0] = DISTANCE_1;
      distances[1] = DISTANCE_0;
    }
    leds[LED_RINDEX(0)] = CRGB::Blue;
    FastLED.show();
    updateStatus("Timing train");

    // Wait for the train to reach the middle
    while(!triggerDetect(nextBlock)) { showTriggerStates(); }
    times[1] = millis();
    leds[LED_RINDEX(1)] = CRGB::Blue;
    FastLED.show();

    // Wait for the train to reach the end
    while(!triggerDetect(endBlock)) { showTriggerStates(); }
    times[2] = millis();
    leds[LED_RINDEX(2)] = CRGB::Blue;
    FastLED.show();

    // Calculate and display results
    resultsVelocityAcceleration((times[1] - times[0]), (times[2] - times[1]), distances[0], distances[1]);

    // Wait for blocks to be clear
    while (trainPresent());
  }

/*
 *          |    <- DISTANCE_0 ->    |    <- DISTANCE_1 ->    |
 *          |  Speed measured ---->  |  <---- Speed measured  |
 * =========|========================|========================|========
 *                   Block 0                   Block 1
 */
  void doVelocity2(){
    unsigned long times[2];     // The value of millis() when the first and second light gate was triggered
    clearScreen();
    FastLED.clear();

    // Ensure all light gates are clear
    // Once they're all clear ensure they stay that way for at least INTER_TRAIN_DELAY milliseconds
    if (trainPresent()) {
      showError(F("Sensors not clear"));
      unsigned long continueAt = -1;
      while(continueAt > millis()) {
        if (trainPresent()) {
          continueAt = millis() + INTER_TRAIN_DELAY;
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_WHITE);
          display.setTextSize(1);
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 17)) / 2), SCREEN_LINE_3);
          display.print(F("Clear the sensors"));
        } else {
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_BLACK);
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 16)) / 2), SCREEN_LINE_3);
          display.print(F("Please wait "));
          display.print((continueAt - millis())/1000.0, 1);
        }
        drawTriggerStates();
        display.display();
        showTriggerStates();
      }
      clearScreen();
    }

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, SCREEN_LINE_0);
    display.print(F("1:"));
    display.print(SCALE);

    updateStatus(F("Waiting for train"));
    leds[LED_RINDEX(0)] = CRGB::Yellow;
    leds[LED_RINDEX(1)] = CRGB::Yellow;
    FastLED.show();

    // Wait for a train to enter
    while (!triggerDetect(0) && !triggerDetect(1)) {showTriggerStates();}
    if(triggerDetect(0) && triggerDetect(1)) {return;}  // We have no idea what direction the train is moving
    times[0] = millis();
    const byte endBlock = triggerDetect(0) ? 1 : 0;
    if(endBlock == 1) {
      // Train is moving 0 -> 1
      for(unsigned int i = 2; i < LEDS_COUNT / 2; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
    } else {
      // Train is moving 1 -> 0
      for(unsigned int i = LEDS_COUNT / 2; i < LEDS_COUNT - 2; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
    }
    leds[LED_RINDEX(0)] = CRGB::Blue;
    FastLED.show();
    updateStatus("Timing train");

    // Wait for the train to reach the end
    while(!triggerDetect(endBlock)) { showTriggerStates(); }
    times[1] = millis();
    leds[LED_RINDEX(1)] = CRGB::Blue;
    FastLED.show();

    // Calculate and display results
    resultsVelocity((times[1] - times[0]), (endBlock == 1 ? DISTANCE_0 : DISTANCE_1));

    // Wait for blocks to be clear
    while (trainPresent());
  }

/*
 *                 |         <- DISTANCE_0 ->         |
 * ================|==================================|================
 *     Block 0                    Block 1                   Block 2
 */
  void doVelocity3(){
    unsigned long times[2];     // The value of millis() when the first and second light gate was triggered
    clearScreen();
    FastLED.clear();

    // Ensure all light gates are clear
    // Once they're all clear ensure they stay that way for at least INTER_TRAIN_DELAY milliseconds
    if (trainPresent()) {
      showError(F("Sensors not clear"));
      unsigned long continueAt = -1;
      while(continueAt > millis()) {
        if (trainPresent()) {
          continueAt = millis() + INTER_TRAIN_DELAY;
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_WHITE);
          display.setTextSize(1);
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 17)) / 2), SCREEN_LINE_3);
          display.print(F("Clear the sensors"));
        } else {
          display.fillRect(0, SCREEN_LINE_3, SCREEN_WIDTH, SCREEN_CHAR_HEIGHT, SSD1306_BLACK);
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
          display.setCursor(((SCREEN_WIDTH - (SCREEN_CHAR_WIDTH * 16)) / 2), SCREEN_LINE_3);
          display.print(F("Please wait "));
          display.print((continueAt - millis())/1000.0, 1);
        }
        drawTriggerStates();
        display.display();
        showTriggerStates();
      }
      clearScreen();
    }

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, SCREEN_LINE_0);
    display.print(F("1:"));
    display.print(SCALE);

    updateStatus(F("Waiting for train"));
    leds[LED_RINDEX(0)] = CRGB::Yellow;
    leds[LED_RINDEX(1)] = CRGB::Yellow;
    FastLED.show();

    // Wait for a train to enter
    while (!triggerDetect(1)) {showTriggerStates();}
    times[0] = millis();
    const byte endBlock = triggerDetect(0) ? 2 : 0;
    if(endBlock == 2) {
      // Train is moving 0 -> 1 -> 2
      for(unsigned int i = 2; i < LEDS_COUNT / 2; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
    } else {
      // Train is moving 2 -> 1 -> 0
      for(unsigned int i = LEDS_COUNT / 2; i < LEDS_COUNT - 2; i++) { leds[LED_INDEX(i)] = CRGB::Magenta; }
    }
    leds[LED_RINDEX(0)] = CRGB::Blue;
    FastLED.show();
    updateStatus("Timing train");

    // Wait for the train to reach the end
    while(!triggerDetect(endBlock)) { showTriggerStates(); }
    times[1] = millis();
    leds[LED_RINDEX(1)] = CRGB::Blue;
    FastLED.show();

    // Calculate and display results
    resultsVelocity((times[1] - times[0]), DISTANCE_0);

    // Wait for blocks to be clear
    while (trainPresent());
  }
#endif
