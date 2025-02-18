#include "diffdrive_arduino/wheel.h"

#include <cmath>

#define ENCODER_MAX 16383 // 14-bit absolute encoder range

Wheel::Wheel(const std::string &wheel_name, int counts_per_rev)
{
  setup(wheel_name, counts_per_rev);
}


void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
  name = wheel_name;
  rads_per_count = (2*M_PI)/counts_per_rev;
}

double Wheel::calcEncAngle()
{
  // Ensure proper wrap-around handling for absolute encoders
  int wrapped_enc = enc % (ENCODER_MAX + 1);
  return wrapped_enc * rads_per_count;
}