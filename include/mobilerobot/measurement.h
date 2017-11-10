#ifndef MOBILEROBOT_MEASUREMENT_H_
#define MOBILEROBOT_MEASUREMENT_H_

#include <eigen3/Eigen/Dense>

namespace measurement {

// (x, y, vx, vy)
using State = Eigen::Vector4d;

} // namespace control

#endif // MOBILEROBOT_MEASUREMENT_H_
