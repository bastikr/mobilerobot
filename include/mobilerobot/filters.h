#ifndef MOBILEROBOT_FILTERS_H_
#define MOBILEROBOT_FILTERS_H_

#include <eigen3/Eigen/Dense>

#include "mobilerobot/control.h"
#include "mobilerobot/measurement.h"
#include "mobilerobot/worldmodel.h"
#include "mobilerobot/sensormodel.h"


namespace filters {


struct KalmanProbability {
  worldmodel::State mu;
  worldmodel::Matrix sigma;
};

class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter(const worldmodel::World& world, const sensormodel::GPSSensor& sensor)
    : world(world), sensor(sensor) {}

  KalmanProbability filter(double dt, const KalmanProbability& p0, const control::State& u, const measurement::State& z) const;

private:
  const worldmodel::World& world;
  const sensormodel::GPSSensor& sensor;
};

} // namespace filters

#endif // MOBILEROBOT_FILTERS_H_
