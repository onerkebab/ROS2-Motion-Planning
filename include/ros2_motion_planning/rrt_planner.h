#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <cstdint>
#include <memory>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "ros2_motion_planning/srv/motion_planning_service.hpp"

namespace motion_planning {

  class RRTPlanner {
    public:
      RRTPlanner() = default;
      virtual ~RRTPlanner() = default;

      bool generate_path(
        const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Request>& request,
        const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Response>& response
      );

    private:
      const double robot_radius = 0.1; 
      const int8_t obstacle = -128; 
      
      bool interpolate_collision(
        const double& x1, const double& y1, const double& x2, const double& y2, 
        const nav_msgs::msg::OccupancyGrid& map, const std::vector<int8_t>& cspace
      );
      
  };

}

#endif /* RRT_PLANNER_H */
