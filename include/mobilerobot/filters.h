#ifndef MOBILEROBOT_FILTERS_H_
#define MOBILEROBOT_FILTERS_H_

#include <eigen3/Eigen/Dense>

#include "mobilerobot/world.h"
#include "mobilerobot/sensors.h"

namespace filters {

// (x, y, v, theta)
using ModelState = Eigen::Vector4d;

using Covariance = Eigen::Matrix4d;

struct KalmanProbability {
  ModelState mu;
  Covariance sigma;
};

class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter(double dt, double gamma_v, double gamma_theta,
                       double noise_v, double noise_theta,
                       double noise_position, double noise_velocity)
  : dt_(dt), gamma_v_(gamma_v), gamma_theta_(gamma_theta),
  noise_v_(noise_v), noise_theta_(noise_theta),
  noise_position_(noise_position), noise_velocity_(noise_velocity) {}

  ModelState g(const ModelState& x0, const world::Control& u);
  ModelState h(const ModelState& x0);
  Covariance GMatrix(const ModelState& x0, const world::Control& u);
  Covariance HMatrix(const ModelState& x0, const world::Control& u);
  Covariance RMatrix(const ModelState& x0, const world::Control& u);
  Covariance QMatrix(const ModelState& x0, const world::Control& u);
  KalmanProbability filter(const KalmanProbability& p0, const world::Control& u, const sensors::Measurement& z);

private:
  double dt_;
  double gamma_v_;
  double gamma_theta_;
  double noise_v_;
  double noise_theta_;
  double noise_position_;
  double noise_velocity_;
};

} // namespace filters

#endif // MOBILEROBOT_FILTERS_H_
