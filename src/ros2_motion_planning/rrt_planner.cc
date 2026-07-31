#include "ros2_motion_planning/rrt_planner.h"

#include <cmath>
#include <random>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "ros2_motion_planning/rrt_node.h"
#include "ros2_motion_planning/utility_functions.h"
#include "ros2_motion_planning/planner_functions.h"

namespace motion_planning {

  bool RRTPlanner::interpolate_collision(
    const double& x1, const double& y1, const double& x2, const double& y2, 
    const nav_msgs::msg::OccupancyGrid& map, const std::vector<int8_t>& cspace
  ) {
  
    const int width = map.info.width;
    const int height = map.info.height;
    const double res = map.info.resolution;
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;

    double dist = motion_planning::euclidean_dist(x1, y1, x2, y2);
    // Sample based on distance between points at half of map resolution
    int steps = std::ceil(dist / (res / 2.0));

    for (int i = 0; i <= steps; i++) {
      double progress = (steps == 0) ? 1.0 : (double)i / steps;
      
      // Linearly interpolated point between point 1 and point 2
      double temp_x = x1 + progress * (x2 - x1);
      double temp_y = y1 + progress * (y2 - y1);

      int temp_x_idx = std::floor((temp_x - origin_x) / res);
      int temp_y_idx = std::floor((temp_y - origin_y) / res);
      int temp_idx = motion_planning::xy_to_index(temp_x_idx, temp_y_idx, width);

      // Out-of-bounds or obstacle check
      if (temp_x_idx < 0 || temp_x_idx >= width || temp_y_idx < 0 || temp_y_idx >= height || cspace[temp_idx] == obstacle) {
        return false;
      }
    }
    
    return true;
    
  }

  bool RRTPlanner::generate_path(
    const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Request>& request,
    const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Response>& response
  ) {
    
    // Extract map information and expand C-space
    const std::vector<int8_t> cspace = motion_planning::expand_cspace(request->map, robot_radius, obstacle);
    const double res = request->map.info.resolution;
    const int width = request->map.info.width;
    const int height = request->map.info.height;
    const double origin_x = request->map.info.origin.position.x;
    const double origin_y = request->map.info.origin.position.y;

    // Coordinates to map corner opposite to origin.
    const double corner_x = origin_x + (width * res);
    const double corner_y = origin_y + (height * res);

    // RNG
    std::random_device rand;
    std::minstd_rand gen(rand());
    // Set random distribution ranges for x and y within map bounds.
    std::uniform_real_distribution<double> x_range(origin_x, corner_x);
    std::uniform_real_distribution<double> y_range(origin_y, corner_y);

    // Initialize RRT Tree
    std::vector<std::shared_ptr<motion_planning::RRTNode>> tree;
    tree.push_back(std::make_shared<motion_planning::RRTNode>(request->start.x, request->start.y, request->start.theta, 0.0, nullptr));

    std::shared_ptr<motion_planning::RRTNode> goal_node = nullptr;

    // RRT parameters
    int max_iter = 100000;
    double step_size = 0.5; // (m)

    // RRT algorithm
    auto start_time = std::chrono::steady_clock::now(); // Starts keeping track of time
    for (int i = 0; i < max_iter; i++) {
     
      // Check time performance requirement
      auto current_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_time = current_time - start_time;
      if (elapsed_time.count() >= 10.0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RRT Planner exceeded 10 seconds requirement, ending algorithm...");
        return false;
      }

      // Sample random point on map
      double rand_x = x_range(gen);
      double rand_y = y_range(gen);

      // Find nearest node on tree
      std::shared_ptr<motion_planning::RRTNode> q_near = tree.front();
      double min_dist = motion_planning::euclidean_dist(q_near->x, q_near->y, rand_x, rand_y);
      for (const auto& node : tree) {
        double dist = motion_planning::euclidean_dist(node->x, node->y, rand_x, rand_y);
        if (dist < min_dist) {
          min_dist = dist;
          q_near = node;
        }
      }

      // Create new node branched out from nearest node
      double new_theta = std::atan2(rand_y - q_near->y, rand_x - q_near->x);
      double step = std::min(step_size, min_dist);
      double new_x = q_near->x + step * std::cos(new_theta);
      double new_y = q_near->y + step * std::sin(new_theta);

      // Check for collisions
      if (interpolate_collision(q_near->x, q_near->y, new_x, new_y, request->map, cspace)) {
        auto q_new = std::make_shared<motion_planning::RRTNode>(new_x, new_y, new_theta, 0.0, q_near);
        tree.push_back(q_new);

        // Check if within step_size of goal and clear of obstacles
        double goal_dist = motion_planning::euclidean_dist(new_x, new_y, request->goal.x, request->goal.y);
        if (goal_dist <= step_size && interpolate_collision(new_x, new_y, request->goal.x, request->goal.y, request->map, cspace)) {
          
          // Draw direct path to goal
          goal_node = std::make_shared<motion_planning::RRTNode>(request->goal.x, request->goal.y, request->goal.theta, 0.0, q_new);
          
          // Log algorithm runtime
          current_time = std::chrono::steady_clock::now();
          elapsed_time = current_time - start_time;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Solution found in %f seconds!", elapsed_time.count());
          break;

        }
      }
    }

    // Backtrack from goal node (if found) to populate plan
    if (goal_node != nullptr) {

      // Send response
      response->plan.poses = motion_planning::populate_plan(goal_node);
      return true; // Solution found and plan generated successfully
    
    }

    return false; // Solution not found/plan failed to generate

  }

}
