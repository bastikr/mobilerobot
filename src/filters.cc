#include "mobilerobot/filters.h"

#include <cmath>

namespace filters {

KalmanProbability ExtendedKalmanFilter::filter(double dt, const KalmanProbability& p0, const control::State& u, const measurement::State& z) const {
  worldmodel::Matrix G = world.linearize_taylor(dt, p0.mu, u);
  worldmodel::Matrix R = world.noise_covariance();

  sensormodel::Matrix H = sensor.linearize_taylor(p0.mu, u);
  sensormodel::Matrix Q = sensor.noise_covariance();

  worldmodel::State mu_predict = world.step(dt, p0.mu, u);
  worldmodel::Matrix sigma_predict = G*p0.sigma*G.transpose() + R;
  worldmodel::Matrix K = sigma_predict*H.transpose()*(H*sigma_predict*H.transpose() + Q).inverse();
  worldmodel::State mu_correct = mu_predict + K*(z - sensor.measure(mu_predict));
  worldmodel::Matrix sigma_correct = sigma_predict - K*H*sigma_predict;
  KalmanProbability p1;
  p1.mu = mu_correct;
  p1.sigma = sigma_correct;
  return p1;
}

} // namespace filters

