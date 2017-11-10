#include "mobilerobot/filters.h"

#include <cmath>

namespace filters {

ModelState ExtendedKalmanFilter::g(const ModelState& x0, const world::Control& u) {
  ModelState dx;
  dx(0) = x0(2) * cos(x0(3));
  dx(1) = x0(2) * sin(x0(3));
  dx(2) = u(0) - gamma_v_*x0(2);
  dx(3) = u(1)*exp(-gamma_theta_*x0(2));
  return x0 + dt_*dx;
}

ModelState ExtendedKalmanFilter::h(const ModelState& x0) {
  ModelState z;
  z(0) = x0(0);
  z(1) = x0(1);
  z(2) = x0(2)*cos(x0(3));
  z(3) = x0(2)*sin(x0(3));
  return z;
}

Covariance ExtendedKalmanFilter::GMatrix(const ModelState& x0, const world::Control& u) {
  double v = x0(2);
  double theta = x0(3);
  Covariance G = Covariance::Zero();
  G(0, 0) = 1;
  G(1, 1) = 1;
  G(0, 2) = dt_*cos(theta);
  G(1, 2) = dt_*sin(theta);
  G(2, 2) = 1 - dt_*gamma_v_;
  G(3, 2) = -dt_*u(1)*gamma_theta_*exp(-gamma_theta_*v);
  G(0, 3) = -dt_*sin(theta);
  G(1, 3) = dt_*cos(theta);
  G(3, 3) = 1;
  return G;
}

Covariance ExtendedKalmanFilter::HMatrix(const ModelState& x0, const world::Control& u) {
  double v = x0(2);
  double theta = x0(3);
  Covariance H = Covariance::Zero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(2, 2) = cos(theta);
  H(3, 2) = sin(theta);
  H(2, 3) = -v*sin(theta);
  H(3, 3) = v*cos(theta);
  return H;
}

Covariance ExtendedKalmanFilter::RMatrix(const ModelState& x0, const world::Control& u) {
  Covariance R = Covariance::Zero();
  R(2, 2) = noise_v_;
  R(3, 3) = noise_theta_;
  return R;
}

Covariance ExtendedKalmanFilter::QMatrix(const ModelState& x0, const world::Control& u) {
  Covariance Q = Covariance::Zero();
  Q(0, 0) = noise_position_;
  Q(1, 1) = noise_position_;
  Q(2, 2) = noise_velocity_;
  Q(3, 3) = noise_velocity_;
  return Q;
}

KalmanProbability ExtendedKalmanFilter::filter(const KalmanProbability& p0, const world::Control& u, const sensors::Measurement& z) {
  Covariance G = GMatrix(p0.mu, u);
  Covariance H = HMatrix(p0.mu, u);
  Covariance R = RMatrix(p0.mu, u);
  Covariance Q = QMatrix(p0.mu, u);

  ModelState mu_predict = g(p0.mu, u);
  Covariance sigma_predict = G*p0.sigma*G.transpose() + R;
  Covariance K = sigma_predict*H.transpose()*(H*sigma_predict*H.transpose() + Q).inverse();
  ModelState mu_correct = mu_predict + K*(z - h(mu_predict));
  Covariance sigma_correct = sigma_predict - K*H*sigma_predict;
  KalmanProbability p1;
  p1.mu = mu_correct;
  p1.sigma = sigma_correct;
  return p1;
}

} // namespace filters

