#include <cmath>
#include <random>
#include <eigen3/Eigen/Dense>

#include "mobilerobot/world.h"

namespace world {

World::World(double ticktime, int subticks, double gamma_v, double gamma_theta, double noise_v, double noise_theta)
: ticktime_(ticktime), subticks_(subticks),
gamma_v_(gamma_v), gamma_theta_(gamma_theta),
noise_v_(noise_v), noise_theta_(noise_theta) {
  std::random_device rd;
  gen_v_ = std::mt19937(rd());
  gen_theta_ = std::mt19937(rd());
  gaussian_v_ = std::normal_distribution<>(0, noise_v_);
  gaussian_theta_ = std::normal_distribution<>(0, noise_theta_);
}

void World::derivative(const State& x, const Control& u, State& dx) {
  dx(0) = x(2) * cos(x(3));
  dx(1) = x(2) * sin(x(3));
  dx(2) = u(0) - gamma_v_*x(2);
  dx(3) = u(1)*exp(-gamma_theta_*x(2));
}

State World::derivative(const State& x, const Control& u) {
  State dx;
  derivative(x, u, dx);
  return dx;
}

State World::step(State x, const Control& u) {
  State dx;
  double dt = ticktime_/subticks_;
  for (int i=1; i<=subticks_; i++) {
    derivative(x, u, dx);
    x += dx*dt;
    x(2) += random_dv()*sqrt(dt);
    x(3) += random_dtheta()*sqrt(dt);
  }
  return x;
}

double World::random_dv() {
  return gaussian_v_(gen_v_);
}

double World::random_dtheta() {
  return gaussian_theta_(gen_theta_);
}

} // namespace world
