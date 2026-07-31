#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include <cstdint>

namespace motion_planning {

  double euclidean_dist(const double&, const double&, const double&, const double&);
  int xy_to_index(const int&, const int&, const int&);
  double turn_cost(const double&, const double&);
  double terrain_multiplier(const int8_t&);

}

#endif /* UTILITY_FUNCTIONS_H */
