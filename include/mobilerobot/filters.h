#ifndef MOBILEROBOT_FILTERS_H_
#define MOBILEROBOT_FILTERS_H_

#include <vector>
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

using Particles = std::vector<worldmodel::State>;

class ParticleFilter {
public:
  ParticleFilter(worldmodel::World world, sensormodel::GPSSensor sensor)
      : world_(world), sensor_(sensor) {
    std::random_device rd;
    gen_ = std::mt19937(rd());
    uniform_ = std::uniform_real_distribution<>(0, 1);
  }
  Particles step(double dt, const filters::Particles& particles0, const control::State& u);
  Eigen::VectorXd weights(const Particles& particles, const measurement::State& z);
  Particles resample(const Particles& particles, const Eigen::VectorXd& weights);
  Particles filter(double dt, const filters::Particles& particles,
                    const control::State& u, const measurement::State& z);

  worldmodel::State guess(const filters::Particles&);

private:
  worldmodel::World world_;
  sensormodel::GPSSensor sensor_;

  double rand() {return uniform_(gen_);}
  std::mt19937 gen_;
  std::uniform_real_distribution<> uniform_;
};

} // namespace filters

#endif // MOBILEROBOT_FILTERS_H_
