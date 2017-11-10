#include "mobilerobot/sensors.h"

namespace sensors {

GPSSensor::GPSSensor(double noise_x, double noise_v)
    : t(NAN), x(NAN), y(NAN), vx(NAN), vy(NAN) {
  std::random_device rd;
  gen_x_ = std::mt19937(rd());
  gen_v_ = std::mt19937(rd());
  gaussian_x_ = std::normal_distribution<>(0, noise_x);
  gaussian_v_ = std::normal_distribution<>(0, noise_v);
}

void GPSSensor::measure(double t, const world::State& state) {
  this->t = t;
  x = state(0) + NoisePosition();
  y = state(1) + NoisePosition();
  vx = state(2) * cos(state(3)) + NoiseVelocity();
  vy = state(2) * sin(state(3)) + NoiseVelocity();
}

double GPSSensor::NoisePosition() {return gaussian_x_(gen_x_);}
double GPSSensor::NoiseVelocity() {return gaussian_v_(gen_v_);}



} // namespace sensors
