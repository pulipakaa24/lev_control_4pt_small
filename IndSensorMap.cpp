#include "IndSensorMap.hpp"
#include <math.h>

// Sensor calibration data
IndSensorMap ind0Map = {-8.976076325826309, 913.5463710698101, 0.29767471011439534, 5.6686184386250025, 0.3627635461289861};
IndSensorMap ind1Map = {-4.831976283950702, 885.9877001844566, 0.2793284618109283, 3.8852507844119217, 0.2389935455347361};
IndSensorMap ind2Map = {-9.824360913609562, 871.4744633266955, 0.2909366235093304, 4.3307594408159495, 0.2822807132259202};
IndSensorMap ind3Map = {-13.891292062248292, 990.6819962477331, 0.16376045588859353, -0.074904004740735, 0.17727132893449118};

// Convert raw analog reading to millimeters using sensor calibration
float ind2mm(IndSensorMap ind, unsigned int raw) {
  return ind.C - (1.0 / ind.B) * log(pow((ind.K - ind.A) / ((float)raw - ind.A), ind.v) - 1.0);
}
