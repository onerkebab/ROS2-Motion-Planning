#ifndef PLANNER_FUNCTIONS_H
#define PLANNER_FUNCTIONS_H

#include <vector>
#include <memory>
#include <cmath>
#include <cstdint>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ros2_motion_planning/utility_functions.h"

namespace motion_planning {

  std::vector<int8_t> expand_cspace(const nav_msgs::msg::OccupancyGrid&, const double&, const int8_t&);

  // Template to populate plan with either A* or RRT node shared pointers.
  template <typename node_ptr>
  std::vector<geometry_msgs::msg::PoseStamped> populate_plan(
    const node_ptr& goal_node,
    const double& origin_x = 0.0,
    const double& origin_y = 0.0,
    const double& res = 1.0
  ) {
    
    // Start back-tracking from goal node to populate path from goal-to-start
    std::vector<geometry_msgs::msg::PoseStamped> goal_to_start;
    node_ptr node = goal_node;

    while (node != nullptr) { // Back-track to start node (backpointer = null)

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";

      // For A*: Nodes store position in row/col form. Multiply row/col with map resolution and offset by origin to get continuous (x,y) coordinates in (m).
      // For RRT: Nodes already store position in continuous (x,y) coordinates with respect to origin in (m). Set res = 1 and origin = (0,0).
      pose.pose.position.x = (node->x * res) + origin_x;
      pose.pose.position.y = (node->y * res) + origin_y;
      pose.pose.position.z = 0.0;

      // Quaternion representation for PoseStamped
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(node->theta / 2.0);
      pose.pose.orientation.w = std::cos(node->theta / 2.0);

      // Push pose to goal-to-start path and move to next node
      goal_to_start.push_back(pose);
      node = node->bp;

    }
 
    // Reverse goal-to-start path to get start-to-goal path
    std::vector<geometry_msgs::msg::PoseStamped> start_to_goal(goal_to_start.rbegin(), goal_to_start.rend());
    return start_to_goal;

  }

}

#endif /* PLANNER_FUNCTIONS_H */
