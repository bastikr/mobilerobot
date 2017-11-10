#ifndef MOBILEROBOT_SENSORMODEL_H_
#define MOBILEROBOT_SENSORMODEL_H_

#include <eigen3/Eigen/Dense>

#include "mobilerobot/measurement.h"
#include "mobilerobot/worldmodel.h"

namespace sensormodel {

using Matrix = Eigen::Matrix4d;

struct GPSSensor {
  GPSSensor(double noise_position, double noise_velocity)
    : noise_position(noise_position), noise_velocity(noise_velocity) {}

  measurement::State measure(const worldmodel::State&) const;

  Matrix linearize_taylor(const worldmodel::State& x0, const control::State& u0) const;
  Matrix noise_covariance() const;

  double noise_position;
  double noise_velocity;
};

} // namespace sensormodel

#endif // MOBILEROBOT_SENSORMODEL_H_
