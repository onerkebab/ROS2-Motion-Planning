#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <memory>
#include <cstdint>

#include "ros2_motion_planning/srv/motion_planning_service.hpp"

namespace motion_planning {

  class AStarPlanner {
  
    public:
      AStarPlanner() = default;
      virtual ~AStarPlanner() = default;

      bool generate_path(
        const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Request>& request,
        const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Response>& response
      );
      
    private:
      const double robot_radius = 0.25; 
      const int8_t obstacle = -128; 
      
  };

}

#endif /* ASTAR_PLANNER_H */
