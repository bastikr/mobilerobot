#include "mobilerobot/filters.h"

#include <iostream>
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

Particles ParticleFilter::step(double dt, const Particles& particles0, const control::State& u) {
  int Nparticles = particles0.size();
  Eigen::VectorXd weights(Nparticles);
  Particles particles1(Nparticles);
  for (int i=0; i<Nparticles; i++) {
    worldmodel::State particle = particles0[i];
    particles1[i] = world_.random_step(dt, particle, u);
  }
  return particles1;
}

Eigen::VectorXd ParticleFilter::weights(const Particles& particles, const measurement::State& z) {
  int Nparticles = particles.size();
  Eigen::VectorXd weights(Nparticles);
  for (int i=0; i<Nparticles; i++) {
    weights[i] = sensor_.probability(z, particles[i]);
  }
  weights /= weights.sum();
  return weights;
}

// Particles ParticleFilter::resample(const Particles& particles, const Eigen::VectorXd& weights) {
//   std::cout << "resample:" << std::endl;
//   int Nparticles = particles.size();
//   Eigen::VectorXd cumweights(Nparticles);
//   cumweights[0] = 0;
//   for (int i=1; i<Nparticles; i++) cumweights[i] = cumweights[i-1] + weights[i];
//   Particles particles_resampled(Nparticles);
//   for (int i=0; i<Nparticles; i++) {
//     double r = rand();
//     int j = 0;
//     while (j<Nparticles && cumweights[j]<r) j++;
//     particles_resampled[i] = particles[j-1];
//     std::cout << "i: " << i << "    j: " << j << std::endl;
//   }
//   std::cout << "======= Resample ==========" << std::endl;
//   std::cout << "previous:" << std::endl;
//   for (auto& particle: particles) std::cout << particle.transpose() << std::endl;
//   std::cout << "resampled:" << std::endl;
//   for (auto& particle: particles_resampled) std::cout << particle.transpose() << std::endl;
//   return particles_resampled;
// }

Particles ParticleFilter::resample(const Particles& particles, const Eigen::VectorXd& weights) {
  int Nparticles = particles.size();
  Particles particles_resampled(Nparticles);
  double r = rand()/Nparticles;
  double c = weights(0);
  int i = 1;
  for (int n=1; n<=Nparticles; n++) {
    double U = r + (n-1)/double(Nparticles);
//     std::cout << "U: " << U << std::endl;
    while (U > c) {
      i += 1;
      c += weights(i-1);
    }
//     if (i>3) {
//       std::cout << "i = " << i << std::endl;
//       throw(1);
//     }
    particles_resampled[n-1] = particles[i-1];
//     std::cout << "i: " << i << "    j: " << j << std::endl;
  }
//   std::cout << "previous:" << std::endl;
//   for (auto& particle: particles) std::cout << particle.transpose() << std::endl;
//   std::cout << "resampled:" << std::endl;
//   for (auto& particle: particles_resampled) std::cout << particle.transpose() << std::endl;
  return particles_resampled;
}

Particles ParticleFilter::filter(double dt, const Particles& particles0,
                                          const control::State& u, const measurement::State& z) {
//   std::cout << "---------- filter ---------- " << std::endl;
  Particles particles1 = step(dt, particles0, u);
  Eigen::VectorXd w = weights(particles1, z);
  Particles particles_resampled = resample(particles1, w);
//   Eigen::VectorXd w2 = weights(particles_resampled, z);
//   std::cout << "weights before: " << w.transpose() << std::endl;
//   std::cout << "weights resampled: " << w2.transpose() << std::endl;
  return particles_resampled;
}

worldmodel::State ParticleFilter::guess(const Particles& particles) {
  worldmodel::State Mean = worldmodel::State::Zero();
  for (auto& particle: particles) Mean += particle;
  Mean /= particles.size();
  return Mean;
}

} // namespace filters

