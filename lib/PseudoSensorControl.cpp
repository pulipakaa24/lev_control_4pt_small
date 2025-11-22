#include "PseudoSensorControl.hpp"
#include <Arduino.h>

// CONTROLLER CONSTANTS
float MAX_INTEGRAL_TERM = 1e4;

PinPair pinMap[4] = {{dirFL, pwmFL}, {dirFR, pwmFR}, {dirBL, pwmBL}, {dirBR, pwmBR}};

void PseudoSensorController::update() {

  Left.readMM();
  Right.readMM();
  Front.readMM();
  Back.readMM(); // read and update dists/oor for all sensors.

  oor = Left.oor || Right.oor || Front.oor || Back.oor;

  control();

  for (uint8_t i = 0; i < 4; i++) {
    PWMs[i] = slewLimit(PWMs[i], Prevs[i]);
    Prevs[i] = PWMs[i];
  }
}

void PseudoSensorController::zeroPWMs() {
  memset(PWMs, 0, sizeof(PWMs));
}

void PseudoSensorController::sendOutputs() {
  if (!outputOn) zeroPWMs();

  // Using direct register writes to maintain fast PWM mode set by setupFastPWM()
  // FL: Pin 11 -> Timer 2A
  digitalWrite(dirFL, PWMs[0] < 0);
  OCR2A = abs(PWMs[0]);
  
  // FR: Pin 3 -> Timer 2B
  digitalWrite(dirFR, PWMs[1] < 0);
  OCR2B = abs(PWMs[1]);
  
  // BL: Pin 9 -> Timer 1A
  digitalWrite(dirBL, PWMs[2] < 0);
  OCR1A = abs(PWMs[2]);
  
  // BR: Pin 10 -> Timer 1B
  digitalWrite(dirBR, PWMs[3] < 0);
  OCR1B = abs(PWMs[3]);
}

void PseudoSensorController::control() {
  float avg = (Left.mmVal + Right.mmVal + Front.mmVal + Back.mmVal) * 0.25f;
  float pseudos[4] = {Front.mmVal + Left.mmVal - avg, // FL
                      Front.mmVal + Right.mmVal - avg, // FR
                      Back.mmVal + Left.mmVal - avg, // BL
                      Back.mmVal + Right.mmVal - avg}; // BR

  for (uint8_t i = 0; i < 4; i++) {
    float eCurr = Refs[i] - pseudos[i];
    
    errors[i].eDiff = (eCurr - errors[i].e);
    
    // Only integrate when not out of range
    if (!oor) {
      errors[i].eInt += eCurr;
      errors[i].eInt = constrain(errors[i].eInt, -MAX_INTEGRAL_TERM, MAX_INTEGRAL_TERM);
    }
    
    errors[i].e = eCurr;

    PWMs[i] = pwmFunc(Consts, errors[i]);
  }
}

int16_t PseudoSensorController::pwmFunc(K_MAP consts, Errors errs) {
  if (oor) return 0;
  Constants constants = (errs.e < 0) ? consts.attracting : consts.repelling;
  return (int)constrain(constants.kp*errs.e + constants.ki*errs.eInt + constants.kd*errs.eDiff, -(float)CAP,(float)CAP);
}

int16_t PseudoSensorController::slewLimit(int16_t target, int16_t prev) {
  int16_t delta = target - prev;
  if (abs(delta) <= slewRateLimit) return target;
  return prev + (delta > 0 ? slewRateLimit : -slewRateLimit);
}

void PseudoSensorController::report() {
  Serial.print("CONTROL ON - ");
  Serial.print(outputOn);
  Serial.print("\n");

  Serial.print("SENSORS - Left: ");
  Serial.print(Left.mmVal);
  Serial.print("mm, Right: ");
  Serial.print(Right.mmVal);
  Serial.print("mm, Front: ");
  Serial.print(Front.mmVal);
  Serial.print("mm, Back: ");
  Serial.print(Back.mmVal);
  Serial.print("mm,\n");

  Serial.print("OOR - Left: ");
  Serial.print(Left.oor);
  Serial.print(", Right: ");
  Serial.print(Right.oor);
  Serial.print(", Front: ");
  Serial.print(Front.oor);
  Serial.print(", Back: ");
  Serial.print(Back.oor);
  Serial.print(",\n");

  Serial.print("Overall OOR: ");
  Serial.println(oor);

  Serial.print("PWMS - FL_PWM: ");
  Serial.print(PWMs[0]);
  Serial.print(", FR_PWM: ");
  Serial.print(PWMs[1]);
  Serial.print(", BL_PWM: ");
  Serial.print(PWMs[2]);
  Serial.print(", BR_PWM: ");
  Serial.print(PWMs[3]);
  Serial.print("\n");
}