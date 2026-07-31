#include "ros2_motion_planning/utility_functions.h"

#include <cmath>

namespace motion_planning {

  // Euclidean distance between (x1, y1) and (x2, y2)
  double euclidean_dist(const double& x1, const double& y1, const double& x2, const double& y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  }

  // Convert (col, row) xy-indices to single OccupancyGrid index
  int xy_to_index(const int& x, const int& y, const int& width) {
    return (y * width) + x;
  }

  // Calculate cost to change heading from angle theta_i to theta_f in (rad)
  // 1(rad) = 1(m) in cost
  double turn_cost(const double& theta_i, const double& theta_f) {
    return std::abs(std::remainder((theta_f - theta_i), 2.0 * M_PI));
  }

  // Calculate terrain cost multiplier based on OccupancyGrid cell value (-128 to 127)
  double terrain_multiplier(const int8_t& cell) {
    // Normalize traversable values [-127, 127] to [0.0, 1.0] (0.0 = easiest terrain)
    double norm = (127.0 - static_cast<double>(cell)) / 255.0;
    norm = (norm < 0.0) ? 0.0 : ((norm > 1.0) ? 1.0 : norm);
    // Map to cost factor in range [1.0, 5.0] (1.0 = easiest terrain, 5.0 = hardest terrain)
    return 1.0 + 4.0 * norm;
  }

}
