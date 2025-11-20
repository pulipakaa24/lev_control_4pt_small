#ifndef PSEUDOSENSORCONTROLLER_HPP
#define PSEUDOSENSORCONTROLLER_HPP

#include <stdint.h>
#include <Vector.h>
#include "IndSensorMap.hpp"

// PIN MAPPING
#define dirFR 2
#define pwmFR 3
#define dirBR 4
#define pwmBR 5
#define pwmFL 6
#define dirFL 7
#define dirBL 8
#define pwmBL 9

typedef struct PinPair {
  const uint8_t dir;
  const uint8_t pwm;
} PinPair;

extern PinPair pinMap[4];
// FL, FR, BL, BR

#define CAP 200

typedef struct Constants {
  float K;
  float ki;
  float kd;
} Constants;

typedef struct K_MAP {
  Constants repelling;
  Constants attracting;
} K_MAP;

typedef struct Errors {
  float e;
  float eDiff;
  float eInt;
} Errors;

class PseudoSensorController {
  public:
    bool oor;
    bool outputOn;

    PseudoSensorController(IndSensor& l, IndSensor& r, IndSensor& f, IndSensor& b,
      K_MAP consts, float* refs, uint16_t slewRate) : Left(l), Right(r), Front(f),
      Back(b), Refs(refs), errors{}, Consts(consts), oor(false), outputOn(false),
      Prevs{}, slewRateLimit(slewRate) {}

    void update();
    void zeroPWMs();
    void sendOutputs();
    void report();

  private:
    void control();
    int16_t pwmFunc(K_MAP consts, Errors errs);
    int16_t slewLimit(int16_t target, int16_t prev);

    IndSensor& Front;
    IndSensor& Back;
    IndSensor& Right;
    IndSensor& Left;

    K_MAP Consts;

    Errors errors[4]; // FL FR BL BR

    float* Refs; // length 4 FL FR BL BR
    uint16_t slewRateLimit;

    int16_t PWMs[4]; // FL FR BL BR

    int16_t Prevs[4]; // FL FR BL BR
};
#endif // PSEUDOSENSORCONTROLLER_HPP