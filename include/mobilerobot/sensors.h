#ifndef MOBILEROBOT_SENSORS_H_
#define MOBILEROBOT_SENSORS_H_

#include <cmath>
#include <random>
#include <eigen3/Eigen/Dense>

#include "mobilerobot/world.h"


namespace sensors {

class GPSSensor {
public:
  GPSSensor(double noise_x, double noise_v);
  void measure(double t, const world::State& state);

  double NoisePosition();
  double NoiseVelocity();

  double t;
  double x;
  double y;
  double vx;
  double vy;

private:
  std::mt19937 gen_x_;
  std::mt19937 gen_v_;
  std::normal_distribution<> gaussian_x_;
  std::normal_distribution<> gaussian_v_;
};

} // namespace sensors

#endif // MOBILEROBOT_SENSORS_H_
