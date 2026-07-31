#include "ros2_motion_planning/planner_functions.h"

#include <utility>

namespace motion_planning {

  // Expand the cylindrical robot's C-space and output a new data vector with expanded obstacles
  std::vector<int8_t> expand_cspace(const nav_msgs::msg::OccupancyGrid& map, const double& radius, const int8_t& obstacle) {

    // Extract OccupancyGrid information
    std::vector<int8_t> cspace = map.data;
    int width = map.info.width;
    int height = map.info.height;
    double res = map.info.resolution;

    // Check for the maximum offset (in either x or y) needed to check from each pixel
    int expand_limit = std::ceil(radius / res);

    double radius_sq = std::pow(radius, 2);

    // Store pairs of (dx, dy) offsets to expand from every obstacle pixel
    std::vector<std::pair<int, int>> expansion_kernel;

    // Calculate all the offsets dx, dy for obstacle expansion
    for (int dx = -expand_limit; dx <= expand_limit; dx++) {
      for (int dy = -expand_limit; dy <= expand_limit; dy++) {
        double dist_sq = std::pow(dx * res, 2) + std::pow(dy * res, 2);
        if (dist_sq <= radius_sq) {
          expansion_kernel.push_back({dx, dy});
        }
      }
    }

    int index = 0;
    int temp_index = 0;

    // Iterate through the original data and expand the C-space (inflating obstacles)
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        index = xy_to_index(x, y, width);
        if (map.data[index] == obstacle) {
          for (const auto& offset : expansion_kernel) {
            int temp_x = x + offset.first;
            int temp_y = y + offset.second;

            // Avoid expanding at map boundaries
            if (temp_x >= 0 && temp_x < width && temp_y >= 0 && temp_y < height) {
              temp_index = xy_to_index(temp_x, temp_y, width);
              cspace[temp_index] = obstacle;
            }

          }
        }
      }
    }

    return cspace;

  }

}
