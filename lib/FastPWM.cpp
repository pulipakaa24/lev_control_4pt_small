#include "FastPWM.hpp"

void setupFastPWM() {
  // Timer 1 (Pins 9 & 10) -> 31.25 kHz
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10); 

  // Timer 2 (Pins 3 & 11) -> 31.25 kHz
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20); 
}