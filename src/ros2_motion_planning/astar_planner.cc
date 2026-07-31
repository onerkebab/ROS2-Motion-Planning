#include "ros2_motion_planning/astar_planner.h"

#include <vector>
#include <queue>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "ros2_motion_planning/astar_node.h"
#include "ros2_motion_planning/utility_functions.h"
#include "ros2_motion_planning/planner_functions.h"

namespace motion_planning {

  bool AStarPlanner::generate_path(
    const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Request>& request,
    const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Response>& response
  ) {
  
    // Extract map information and expand C-space for obstacles
    const std::vector<int8_t> cspace = motion_planning::expand_cspace(request->map, robot_radius, obstacle);
    const double res = request->map.info.resolution;
    const int width = request->map.info.width;
    const int height = request->map.info.height;
    const double origin_x = request->map.info.origin.position.x;
    const double origin_y = request->map.info.origin.position.y;
    const int start_x = std::floor((request->start.x - origin_x) / res);
    const int start_y = std::floor((request->start.y - origin_y) / res);
    const int goal_x = std::floor((request->goal.x - origin_x) / res);
    const int goal_y = std::floor((request->goal.y - origin_y) / res);

    // Priority queue lambda function
    auto lambda = [](const std::shared_ptr<motion_planning::AStarNode>& a, const std::shared_ptr<motion_planning::AStarNode>& b) { return ((a->f) > (b->f)); };
    
    // Open list (priority queue)
    std::priority_queue<std::shared_ptr<motion_planning::AStarNode>, std::vector<std::shared_ptr<motion_planning::AStarNode>>, decltype(lambda)> open_list(lambda);
    
    // Closed list (initialized vector equal to map size, initially null)
    std::vector<std::shared_ptr<motion_planning::AStarNode>> closed_list(width * height, nullptr);

    // 8-neighborhood connected grid possible movements, paired by index
    const int dir_x[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dir_y[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    // Initialize start and goal nodes
    double start_h = motion_planning::euclidean_dist(start_x, start_y, goal_x, goal_y) * res;
    std::shared_ptr<motion_planning::AStarNode> start_node = std::make_shared<motion_planning::AStarNode>(start_x, start_y, request->start.theta, start_h, 0.0, start_h, nullptr);
    open_list.push(start_node);

    std::shared_ptr<motion_planning::AStarNode> goal_node = nullptr;

    // A* search loop
    const auto start_time = std::chrono::steady_clock::now(); // Starts keeping track of time
    while (!open_list.empty()) {

      // Check time performance requirement
      auto current_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_time = current_time - start_time;
      if (elapsed_time.count() >= 10.0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "A* Planner exceeded 10 seconds requirement, ending algorithm...");
        return false;
      }
      
      // Pop lowest-cost node from top of open list
      std::shared_ptr<motion_planning::AStarNode> current_node = open_list.top();
      open_list.pop();

      // Find closed list vector index for current node
      int current_idx = motion_planning::xy_to_index(current_node->x, current_node->y, width);
      
      // Skip if node at same position (x,y) was expanded before
      if (closed_list[current_idx] != nullptr) continue;
      else closed_list[current_idx] = current_node;
      
      // Arrived at goal (x,y)
      if (current_node->x == goal_x && current_node->y == goal_y) {
        
        // Perform turn in-place to satisfy goal theta
        double goal_turn_cost = motion_planning::turn_cost(current_node->theta, request->goal.theta);
        double final_g = current_node->g + goal_turn_cost;
        goal_node = std::make_shared<motion_planning::AStarNode>(current_node->x, current_node->y, request->goal.theta, final_g, final_g, 0.0, current_node);
        
        // Log algorithm runtime
        current_time = std::chrono::steady_clock::now();
        elapsed_time = current_time - start_time;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Solution found in %f seconds!", elapsed_time.count());
        break; // Exit loop

      }

      // 8-neighborhood connected grid node expansion
      for (int i = 0; i < 8; i++) {

        int new_x = current_node->x + dir_x[i];
        int new_y = current_node->y + dir_y[i];
        
        if (new_x >= 0 && new_x < width && new_y >= 0 && new_y < height) { // Check if moving out of bounds

          int temp_idx = motion_planning::xy_to_index(new_x, new_y, width);
          
          if (cspace[temp_idx] != obstacle && closed_list[temp_idx] == nullptr) { // Check if moving into obstacle

            // Calculate base distance cost
            double base_dist = (dir_x[i] == 0 || dir_y[i] == 0) ? res : (res * std::sqrt(2.0));
            // Factor in terrain cost multiplier for target cell
            double terrain_mult = motion_planning::terrain_multiplier(cspace[temp_idx]);
            double move_cost = base_dist * terrain_mult;

            double new_theta = std::atan2(dir_y[i], dir_x[i]); // Calculate new heading using angle of unit vector in direction of movement
            double turn_cost = motion_planning::turn_cost(current_node->theta, new_theta);
            
            // Update cost and heuristics
            double new_g = current_node->g + move_cost + turn_cost;
            double new_h = motion_planning::euclidean_dist(new_x, new_y, goal_x, goal_y) * res; // Heuristics in (m)
            double new_f = new_g + new_h;

            // Push new node to open list
            std::shared_ptr<motion_planning::AStarNode> new_node = std::make_shared<motion_planning::AStarNode>(new_x, new_y, new_theta, new_f, new_g, new_h, current_node);
            open_list.push(new_node);

          }

        }

      }

    }

    // Backtrack from goal node (if found) to populate plan
    if (goal_node != nullptr) {

      // Send response
      response->plan.poses = motion_planning::populate_plan(goal_node, origin_x, origin_y, res);
      return true; // Solution found and plan generated successfully

    }

    return false; // Solution not found/plan failed to generate

  }

}
