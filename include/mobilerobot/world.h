#ifndef MOBILEROBOT_WORLD_H_
#define MOBILEROBOT_WORLD_H_

#include <cmath>
#include <random>
#include <eigen3/Eigen/Dense>

#include "mobilerobot/control.h"


namespace world {

// (x, y, v, theta)
using State = Eigen::Vector4d;


class World {
public:
  World(double ticktime, int subticks, double gamma_v, double gamma_theta, double noise_v, double noise_theta);

  void derivative(const State& x, const control::State& u, State& dx);
  State derivative(const State& x, const control::State& u);

  State step(State x, const control::State& u);

  double random_dv();
  double random_dtheta();

private:
  // time duration for one tick
  double ticktime_;

  // number of euler steps used to evolve state for one tick
  int subticks_;

  // friction parameters
  double gamma_v_;
  double gamma_theta_;

  // noise parameters
  double noise_v_;
  double noise_theta_;

  std::mt19937 gen_v_;
  std::mt19937 gen_theta_;
  std::normal_distribution<> gaussian_v_;
  std::normal_distribution<> gaussian_theta_;
};

} // namespace world

#endif // MOBILEROBOT_WORLD_H_
