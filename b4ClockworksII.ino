/*
b4ClockworksII module firmware for Arduino Nano
Copyright (C) 2025 Malte Steiner

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Pin Definitions
const byte CLOCK_OUTPUTS[3] = { 9, 10, 11 };  // Digital outputs for the 3 clocks
const byte FREQ_POTS[3] = { A0, A2, A4 };     // Analog inputs for frequency control
const byte LENGTH_POTS[3] = { A1, A3, A5 };   // Analog inputs for gate length control
const byte CV_FREQ = A6;                      // CV input for frequency modulation (clock 1)
const byte CV_LENGTH = A7;                    // CV input for gate length modulation (clock 1)
const byte LED_PINS[3] = { 2, 3, 4 };         // LEDs for clocks 1-3
const byte R_PINS[3] = { 5, 6, 7 };           // the switches for the clock mode

// Clock structures
struct Clock {
  unsigned int clockCount = 0;
  unsigned int durCount = 0;
  unsigned int clockInc = 1;
  unsigned int durInc = 1;
  bool randomMode = false;

  unsigned long period;         // Current period in microseconds
  unsigned long onTime;         // Gate on time in microseconds
  unsigned long nextEventTime;  // When the next toggle should happen

  bool state;      // Current output state
  byte outputPin;  // Physical output pin
  byte ledPin;     // Physical led output pin
  byte modePin;    // Physical randommode switch input pin
};

volatile Clock clocks[3];
byte clockCounter = 0;
void setup() {
  cli();  //noInterrupts();

  // Initialize serial for debugging
  //Serial.begin(115200);

  // Set up clock outputs
  for (byte i = 0; i < 3; i++) {
    pinMode(CLOCK_OUTPUTS[i], OUTPUT);
    pinMode(LED_PINS[i], OUTPUT);
    pinMode(R_PINS[i], INPUT_PULLUP);

    clocks[i].outputPin = CLOCK_OUTPUTS[i];
    clocks[i].ledPin = LED_PINS[i];
    clocks[i].modePin = R_PINS[i];

    clocks[i].state = LOW;
    digitalWrite(CLOCK_OUTPUTS[i], LOW);
  }

  // Initialize Timer1 for interrupt

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Set compare match register for 1kHz interrupt rate
  OCR1A = 15999;            // = 16MHz / (prescaler * desired frequency) - 1
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // No prescaling
  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt
  sei();                    //interrupts();
}

byte counter = 0;

// Interrupt routine
ISR(TIMER1_COMPA_vect) {

  ++clockCounter;
  if (clockCounter > 1) {
    clockCounter = 0;

    for (byte currentClock = 0; currentClock < 3; ++currentClock) {
      clocks[currentClock].clockCount += clocks[currentClock].clockInc;
      if (clocks[currentClock].randomMode == true) {
        if (clocks[currentClock].clockCount > 50000) {
          unsigned int randomValue = random(1000);
          if (randomValue > 500) {
            clocks[currentClock].state = true;
            digitalWrite(clocks[currentClock].outputPin, HIGH);
            digitalWrite(clocks[currentClock].ledPin, HIGH);
            clocks[currentClock].durCount = 0;
          }
          clocks[currentClock].clockCount = 0;
        }
      } else {
        if (clocks[currentClock].clockCount > 50000) {
          clocks[currentClock].state = true;
          digitalWrite(clocks[currentClock].outputPin, HIGH);
          digitalWrite(clocks[currentClock].ledPin, HIGH);
          clocks[currentClock].clockCount = 0;
          clocks[currentClock].durCount = 0;
        }
      }

      if (clocks[currentClock].state == true) {
        clocks[currentClock].durCount += clocks[currentClock].durInc;
        if (clocks[currentClock].durCount > 32768) {
          clocks[currentClock].state = false;
          digitalWrite(clocks[currentClock].outputPin, LOW);
          digitalWrite(clocks[currentClock].ledPin, LOW);
        }
      }
    }
  }
}

void loop() {
  counter++;
  if (counter = 10) {
    // Read all potentiometers and update clock parameters

    // Read frequency potentiometer (0-10Hz)
    unsigned int freqRaw = analogRead(FREQ_POTS[0]);

    // Read length potentiometer (0-100% of period)
    unsigned int lengthRaw = analogRead(LENGTH_POTS[0]);

    // For clock 1, apply CV modulation

    // Read CV inputs
    unsigned int freqCV = analogRead(CV_FREQ);
    unsigned int lengthCV = analogRead(CV_LENGTH);

    // Apply modulation (here using 50% modulation depth)
    freqRaw>>=1;
    freqRaw += freqCV;
    lengthRaw = constrain(lengthRaw - lengthCV, 0, 1023);

    // the for-loop which could be here is unrolled for optimization, at least it saves one if

    clocks[0].clockInc = freqRaw + 1;
    clocks[0].durInc = lengthRaw + 1;

    // check for the random mode switch
    clocks[0].randomMode = digitalRead(clocks[0].modePin) == LOW ? true : false;  // switch is inverted, using internal pullup

    // clock 2
    freqRaw = analogRead(FREQ_POTS[1]);
    lengthRaw = analogRead(LENGTH_POTS[1]);
    clocks[1].clockInc = freqRaw + 2;
    clocks[1].durInc = lengthRaw + 1;
    clocks[1].randomMode = digitalRead(clocks[1].modePin) == LOW ? true : false;

    // clock 3
    freqRaw = analogRead(FREQ_POTS[2]);
    lengthRaw = analogRead(LENGTH_POTS[2]);
    clocks[2].clockInc = freqRaw + 2;
    clocks[2].durInc = lengthRaw + 1;
    clocks[2].randomMode = digitalRead(clocks[2].modePin) == LOW ? true : false;
  }
  // Small delay to prevent loop from running too fast
  delayMicroseconds(100);
}