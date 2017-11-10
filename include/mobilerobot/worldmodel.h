#ifndef MOBILEROBOT_WORLDMODEL_H_
#define MOBILEROBOT_WORLDMODEL_H_

#include <random>
#include <eigen3/Eigen/Dense>

#include "mobilerobot/control.h"


namespace worldmodel {

// (x, y, v, theta)
using State = Eigen::Vector4d;

using Matrix = Eigen::Matrix4d;

class World {
public:
  World(double gamma_v, double gamma_theta, double noise_v, double noise_theta);

  worldmodel::State derivative(const worldmodel::State& x, const control::State& u) const;

  worldmodel::State step(double dt, State x, const control::State& u) const;
  worldmodel::State random_step(double dt, State x, const control::State& u);

  double random_dv();
  double random_dtheta();

  Matrix linearize_taylor(double dt, const worldmodel::State& x, const control::State& u) const;
  Matrix noise_covariance() const;

  // friction parameters
  double gamma_v_;
  double gamma_theta_;

  // noise parameters
  double noise_v_;
  double noise_theta_;

private:
  std::mt19937 gen_v_;
  std::mt19937 gen_theta_;
  std::normal_distribution<> gaussian_v_;
  std::normal_distribution<> gaussian_theta_;
};

} // namespace worldmodel

#endif // MOBILEROBOT_WORLDMODEL_H_
