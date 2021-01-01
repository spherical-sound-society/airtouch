// try CompositeSerial.println(inc);


#include <USBComposite.h>
#include <Wire.h>
#include "Adafruit_MPR121_STM32.h"
#include <SPI.h>

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#define MUX_A_Pin           PB3
#define MUX_B_Pin           PB4
#define MUX_C_Pin           PB5
#define GATE_OUT_PIN        PB0
#define IRQ_A_Pin           PB8
#define IRQ_B_Pin           PB9
#define POT_Pin             PA0
#define Buttons_Pin         PB11
#define WS                  PA3
#define BCK                 PA5
#define DATA                PA7
#define OCTAVEPLUS_PIN      PB15
#define OCTAVEMINUS_PIN      PA10
#define isDebugging  false

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121_STM32 cap1 = Adafruit_MPR121_STM32();
Adafruit_MPR121_STM32 cap2 = Adafruit_MPR121_STM32();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched1 = 0;
uint16_t currtouched1 = 0;
uint16_t lasttouched2 = 0;
uint16_t currtouched2 = 0;

int debounceRange = 7;

int pot[] = {0, 0, 0, 0, 0, 0, 0, 0};
int lastPot[] = {0, 0, 0, 0, 0, 0, 0, 0};
bool isButtonPressed[] = {0, 0, 0, 0, 0, 0, 0, 0};
bool lastButtonPressed[] = {0, 0, 0, 0, 0, 0, 0, 0};
bool buttonLatching[] = {0, 0, 0, 0, 0, 0, 0, 0};
byte lastButton = 0;
bool lastOMButtonState = 0;
bool lastOPButtonState = 0;
bool octMButtonState = 0;
bool octPButtonState = 0;

byte octave = 3;

int j = 0;
long lastmillis;

int BPMs = 1;
byte channel = 0;
byte midiPage = 0;
uint16_t DACOut2, DACOut1;

USBMIDI midi;
HardwareTimer timer(3);


void setup() {

  // pinMode(ledPIN, OUTPUT);
  pinMode(MUX_A_Pin, OUTPUT);
  pinMode(MUX_B_Pin, OUTPUT);
  pinMode(MUX_C_Pin, OUTPUT);
  pinMode(IRQ_A_Pin, INPUT_PULLDOWN);
  pinMode(IRQ_B_Pin, INPUT_PULLDOWN);
  pinMode(POT_Pin, INPUT_ANALOG);
  pinMode(Buttons_Pin, INPUT_PULLUP);
  pinMode(OCTAVEPLUS_PIN, INPUT_PULLUP);
  pinMode(OCTAVEMINUS_PIN, INPUT_PULLUP);
  pinMode(GATE_OUT_PIN, OUTPUT);

  // DAC INIT

  SPI.begin(); // Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); // Set the SPI_2 data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV4); // Speed (72 / 4 = 18 MHz SPI_1 speed) PT8211 up to 20MHZ clock
  pinMode(WS, OUTPUT); // Set the Word Select pin (WS) as output.
  SPI.setDataSize (SPI_CR1_DFF); // Set SPI data size to 16Bit

  // timer3 setup
  timer.pause(); // Pause the timer while we're configuring it
  timer.setPeriod(20); // Set up period in microseconds -- 40 Microseconds gives 100Hz with 256 samples per second, 20 Microseconds give 100Hz with 512 samps/s
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE); // Set up an interrupt on channel 1
  timer.setCompare(TIMER_CH1, 1); // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(update_sample); // The interrupt routine we shall run
  timer.refresh(); // Refresh the timer's count, prescale, and overflow
  timer.resume(); // Start the timer counting
  //END DAC INIT

  if (isDebugging) {

    Serial.begin(9600);
    while (!Serial) { // needed to keep leonardo/micro from starting too fast!
      delay(10);
    }
    Serial.println("Adafruit MPR121 Capacitive Touch sensor test");
    // Default address is 0x5A, if tied to 3.3V its 0x5B
    // If tied to SDA its 0x5C and if SCL then 0x5D
    if (!cap1.begin(0x5A)) {
      Serial.println("MPR121 not found, check wiring?");
      while (1);
    }
    Serial.println("MPR121 found!");
  } else {

    USBComposite.setProductId(0x0031);
    midi.begin();
    delay(500);
    // while (!USBComposite); // crashes if not usb data found
  }

  cap1.begin(0x5A);
  cap2.begin(0x5B);
  cap1.setThreshholds(40, 20);
  cap2.setThreshholds(40, 20);
}

void loop() {

  readPots();
  touchPlates();
  readButtons();
  executeButtons();

  if (lastmillis + BPMs < millis()) {
    lastmillis = millis();
    // a slow loop here
    // nextChannel();
  }
}
void executeButtons() {

  // only octave buttons here

  octPButtonState = digitalRead(OCTAVEPLUS_PIN);
  octMButtonState = digitalRead(OCTAVEMINUS_PIN);

  if (octPButtonState != lastOPButtonState && octave < 6) {
    if (octPButtonState == LOW) {
      octave++;
      //Serial.print("octave ++ ");
      //Serial.println(octave);
    } else {
      //Serial.println("off plus");
    }
    //delay bouncing
  }
  lastOPButtonState = octPButtonState;


  /////

  if (octMButtonState != lastOMButtonState && octave > 1) {
    if (octMButtonState == LOW) {
      octave--;
     // Serial.print("octave -- ");
     // Serial.println(octave);
    } else {
     // Serial.println("off minus");
    }
    //delay bouncing
  }
  lastOMButtonState = octMButtonState;

  // end octave buttons, execute the rest
  if (isButtonPressed[5]) {
    midiPage = 0;
  }
  if (isButtonPressed[4]) {
    midiPage = 1;
  }
  if (isButtonPressed[3]) {
    midiPage = 2;
  }
}

void readButtons() {
  lastButton = 100;
  for (byte i = 0; i <= 7; i ++) {
    isButtonPressed[i] = digitalReadMux(i);
    if (isButtonPressed[i] != lastButtonPressed[i]) {

      //update the switch state
      lastButtonPressed[i] = isButtonPressed[i];


      //"HIGH condition code"
      //
      if (isButtonPressed[i] == true) {

        buttonLatching[i] = !buttonLatching[i];
        lastButton = i;
        //select channel with keyboard

        // channel=i;
      }
    }
    // bool out = buttonLatching[i];
    bool out = isButtonPressed[i];
    // Serial.print(out);
    // Serial.print("   ");
  }
  // Serial.println("   ");
}

void nextChannel() {
  channel++;
  if (channel > 8)channel = 0;
}

void readPots() {
  // delay(100);
  for (byte i = 0; i <= 7; i ++) {
    //  for (byte i = 0; i <= 15; i ++) {
    lastPot[i] = pot[i];
    pot[i] = softDebounce(analogReadMux(i), pot[i]);
    // pot[i] = analogReadMux(i);

    // we do >>2 to bitcrush from 10bits of the ADC to 7 bits used in midi
    if (lastPot[i] >> 2 != pot[i] >> 2) {

      if (isDebugging) {
        Serial.print( i); Serial.print(" >>>>>>");
        Serial.print( pot[i] >> 2);
        Serial.print(" ");
      } else {

        midi.sendControlChange(channel, (10 + i) + (midiPage * 8), pot[i] >> 2);
      }
      if (isDebugging) {
        Serial.println(" ");
      }
    }

  }

}



int analogReadMux(byte chan) {
  //we select what pot to read
  digitalWrite(MUX_A_Pin, bitRead(chan, 0));
  digitalWrite(MUX_B_Pin, bitRead(chan, 1));
  digitalWrite(MUX_C_Pin, bitRead(chan, 2));
  delayMicroseconds(30);
  return (analogRead(POT_Pin) >> 3);

}


int digitalReadMux(byte chan) {
  //we select what button to read
  digitalWrite(MUX_A_Pin, bitRead(chan, 2));
  digitalWrite(MUX_B_Pin, bitRead(chan, 0));
  digitalWrite(MUX_C_Pin, bitRead(chan, 1));
  delayMicroseconds(30);
  return (!digitalRead(Buttons_Pin));

}

int  softDebounce(int  readCV, int  oldRead) {
  if (abs(readCV - oldRead) > debounceRange) {
    return readCV;
  }
  return oldRead;
}

void touchPlates() {
  // Get the currently touched pads
  currtouched1 = cap1.touched();
  currtouched2 = cap2.touched();

  int baseNote = octave * 12;

  for (uint8_t i = 0; i < 12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched1 & _BV(i)) && !(lasttouched1 & _BV(i)) ) {
      Serial.print(i); Serial.println(" touched");
      midi.sendNoteOn(channel, i + baseNote, 127);
      //DACOut2 = 13107 * (myoctave + 1) - 32767;
      //DACOut1 = 13107/12 * (i + 30) - 32767;
      // DACOut1 = 1092 * (i + 30) - 32767;
      DACOut1 = 2000 * (i + baseNote) - 32767;
      digitalWrite(GATE_OUT_PIN, HIGH);

    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched1 & _BV(i)) && (lasttouched1 & _BV(i)) ) {
      Serial.print(i); Serial.println(" released");
      midi.sendNoteOff(channel, i + baseNote, 127);
      digitalWrite(GATE_OUT_PIN, LOW);
    }
  }
  for (uint8_t i = 0; i < 12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched2 & _BV(i)) && !(lasttouched2 & _BV(i)) ) {
      Serial.print(i); Serial.println(" touched");
      midi.sendNoteOn(channel, i + baseNote + 12, 127);
      //DACOut1 = 1092 * (i + 42) - 32767;
      DACOut1 = 2000 * (i + baseNote + 12) - 32767;
      digitalWrite(GATE_OUT_PIN, HIGH);
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched2 & _BV(i)) && (lasttouched2 & _BV(i)) ) {
      Serial.print(i); Serial.println(" released");
      midi.sendNoteOff(channel, i + baseNote + 12, 127);
      digitalWrite(GATE_OUT_PIN, LOW);
    }
  }

  //if (isDebugging) extendedDebug();

  // reset our state
  lasttouched1 = currtouched1;
  lasttouched2 = currtouched2;

  // additional debug info on the "mpr12_test" file
}
void extendedDebug() {
  // debugging info, what
  Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t 0x"); Serial.println(cap1.touched(), HEX);
  Serial.print("Filt: ");
  for (uint8_t i = 0; i < 12; i++) {
    Serial.print(cap1.filteredData(i)); Serial.print("\t");
  }
  Serial.println();
  Serial.print("Base: ");
  for (uint8_t i = 0; i < 12; i++) {
    Serial.print(cap1.baselineData(i)); Serial.print("\t");
  }
  Serial.println();

  // put a delay so it isn't overwhelming
  delay(100);
}
void update_sample(void)
{
  //from 32767 to -32767

  // Write the values to the DAC

  gpio_write_bit(GPIOA, 3, LOW); // Replaces digitalWrite(WS, LOW); Select RIGHT Audio channel
  SPI.write(DACOut2);

  gpio_write_bit(GPIOA, 3, HIGH); // Replaces digitalWrite(WS, HIGH); Select LEFT Audio channel
  SPI.write(DACOut1);
}
