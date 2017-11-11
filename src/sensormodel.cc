#include "mobilerobot/sensormodel.h"

namespace sensormodel {

measurement::State GPSSensor::measure(const worldmodel::State& x) const {
  measurement::State z;
  z(0) = x(0);
  z(1) = x(1);
  z(2) = x(2)*cos(x(3));
  z(3) = x(2)*sin(x(3));
  return z;
}

Matrix GPSSensor::linearize_taylor(const worldmodel::State& x, const control::State& u) const {
  double v = x(2);
  double theta = x(3);
  Matrix H = Matrix::Zero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(2, 2) = cos(theta);
  H(3, 2) = sin(theta);
  H(2, 3) = -v*sin(theta);
  H(3, 3) = v*cos(theta);
  return H;
}

Matrix GPSSensor::noise_covariance() const {
  Matrix Q = Matrix::Zero();
  Q(0, 0) = noise_position;
  Q(1, 1) = noise_position;
  Q(2, 2) = noise_velocity;
  Q(3, 3) = noise_velocity;
  return Q;
}

double GPSSensor::probability(const measurement::State& z, const worldmodel::State& x) const {
  double pi = 3.1415926535897;
  measurement::State z0 = measure(x);
  measurement::State dz = z - z0;
  Matrix Q = noise_covariance();
  return sqrt(2*pi*Q.determinant())*exp(-0.5*dz.transpose()*Q.inverse()*dz);
}


} // namespace sensormodel
