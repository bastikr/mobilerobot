#include "mobilerobot/worldmodel.h"

namespace worldmodel {

World::World(double gamma_v, double gamma_theta, double noise_v, double noise_theta)
    : gamma_v_(gamma_v), gamma_theta_(gamma_theta),
      noise_v_(noise_v), noise_theta_(noise_theta) {
  std::random_device rd;
  gen_v_ = std::mt19937(rd());
  gen_theta_ = std::mt19937(rd());
  gaussian_v_ = std::normal_distribution<>(0, noise_v_);
  gaussian_theta_ = std::normal_distribution<>(0, noise_theta_);
}

worldmodel::State World::derivative(const worldmodel::State& x, const control::State& u) const {
  State dx;
  dx(0) = x(2) * cos(x(3));
  dx(1) = x(2) * sin(x(3));
  dx(2) = u(0) - gamma_v_*x(2);
  dx(3) = u(1)*exp(-gamma_theta_*x(2));
  return dx;
}

worldmodel::State World::step(double dt, worldmodel::State x, const control::State& u) const {
  worldmodel::State dx = derivative(x, u);
  x += dx*dt;
  return x;
}

worldmodel::State World::random_step(double dt, worldmodel::State x, const control::State& u) {
  x = step(dt, x, u);
  x(2) += random_dv()*sqrt(dt);
  x(3) += random_dtheta()*sqrt(dt);
  return x;
}

Matrix World::linearize_taylor(double dt, const worldmodel::State& x, const control::State& u) const {
  double v = x(2);
  double theta = x(3);
  Matrix G = Matrix::Zero();
  G(0, 0) = 1;
  G(1, 1) = 1;
  G(0, 2) = dt *cos(theta);
  G(1, 2) = dt *sin(theta);
  G(2, 2) = 1 - dt *gamma_v_;
  G(3, 2) = -dt* u(1)*gamma_theta_*exp(-gamma_theta_*v);
  G(0, 3) = -dt *sin(theta);
  G(1, 3) = dt *cos(theta);
  G(3, 3) = 1;
  return G;
}

Matrix World::noise_covariance() const {
  Matrix R = Matrix::Zero();
  R(2, 2) = noise_v_;
  R(3, 3) = noise_theta_;
  return R;
}


double World::random_dv() {
  return gaussian_v_(gen_v_);
}

double World::random_dtheta() {
  return gaussian_theta_(gen_theta_);
}

} // namespace worldmodel
