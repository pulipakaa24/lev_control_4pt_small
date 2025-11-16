#ifndef IND_SENSOR_MAP_HPP
#define IND_SENSOR_MAP_HPP

// Inductive Sensor Mapping Struct
typedef struct IndSensorMap {
  double A;
  double K;
  double B;
  double C;
  double v;
} IndSensorMap;

// Predefined sensor calibrations
extern IndSensorMap ind0Map;
extern IndSensorMap ind1Map;
extern IndSensorMap ind2Map;
extern IndSensorMap ind3Map;

// Convert raw analog reading to millimeters using sensor calibration
float indToMM(IndSensorMap ind, unsigned int raw);

#endif // IND_SENSOR_MAP_HPP
