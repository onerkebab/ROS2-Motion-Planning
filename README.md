# ROS 2 Motion Planning

![MIT](https://img.shields.io/badge/License-MIT-%23750014) [![ROS](https://img.shields.io/badge/ROS-22314E?logo=ROS&logoColor=white)](#) [![C++](https://img.shields.io/badge/C++-%2300599C.svg?logo=c%2B%2B&logoColor=white)](#)

A ROS 2 package implementing **A\*** and **RRT** (Rapidly-exploring Random Tree) motion planning algorithms for a mobile robot navigating 2D terrain maps, using a client-service architecture. The planner accepts start/goal poses and a map file with grayscale elevation values representing "terrain," or difficulty to traverse. It inflates configuration space obstacles for a cylindrical robot, and computes a minimum-cost collision-free path visualizable in RViz.

## Problem Setup
Plan a collision-free path for a mobile robot from a some starting pose $(x_0, y_0, \theta_0)$ to a goal pose $(x_f, y_f, \theta_f)$ within a 2D terrain map. 

* Robot geometry:
  * Cylindrical differential-drive robot
    * Can perform turns-in-place and straight drives
  * Radius: 0.1 m
  * Maximum speed: 0.5 m/s

* Terrain map:
  * Grayscale `.pgm` file with values ranging from 0 to 255
    * 0 is most traversable (flat land), 255 is an obstacle
    * Higher values indicate more difficult terrain
  * Three map files are provided with this repo under `maps/`:
    * `terrain_open.pgm`: Open terrain with smooth elevation gradients and radial terrain patches.
    * `terrain_maze.pgm`: Maze-like environment with cascaded walls and terrain gradients.
    * `terrain_mixed.pgm`: Map featuring circular obstacles and high-cost terrain patches.

  * NOTE: Terrain cost is evaluated at the examined poses, NOT based on the difference or slope between adjacent cells. For example, cost algorithms will consider going from a $245$-valued cell to a $0$-valued cell as "cheaper" than to a $244$-valued cell. It might be more intuitive to think of the cell values as mud and speedbumps rather than elevation.


## Algorithms

**Motion Planning Algorithms:**

- **A\* Search**: 
  * Deterministic breadth-first search algorithm incorporating an Euclidean heuristic, a 8-connected grid, and terrain cost weighting.
  * Cost function is defined as $f(n) = g(n) + h(n)$ where $g(n)$ is the total accumulated cost up to node $n$, and $h(n)$ is the Euclidean distance from node $n$ to the goal (heuristic). 
  * Guaranteed to find optimal path (lowest cost) with admissible heuristic (Euclidean in this case).

- **RRT**: 
  * Probabilistic path planning algorithm that grows a tree of random samples, with line-segment interpolation for collision awareness.
  * Points are randomly sampled from the map as $q_{rand}$. A line segment of fixed size $\Delta_q$ is extended from the closest node $q_{near}$ in the tree towards $q_{rand}$ (unless $q_{rand}$ is within the step size, in which case $q_{rand}$ is added directly to the tree). If the segment is collision-free, $q_{new}$ is added to the tree, branching from $q_{near}$. 
  * This process is repeated until any $q_{new}$ is within $\Delta_q$ of the goal, provided a collision-free line segment exists between them.
  * Does not guarantee optimality.

Once at the goal node, both algorithms backtrack using backpointers to reconstruct the path from goal to start, which is then published as a sequence of 2D poses.

For both algorithms, the input map's obstacle cells are inflated by the robot's physical radius to generate a C-space for collision-free planning.

## ROS Architecture

The project uses a client-service architecture with the client executable configured to take CLI arguments for map file, algorithm, and start and goal poses, and the server executable to run the planning algorithms.

**Service Definition:**
`srv/MotionPlanningService.srv`
```srv
geometry_msgs/Pose2D start
geometry_msgs/Pose2D goal
nav_msgs/OccupancyGrid map
string algorithm
---
nav_msgs/Path plan
```
`motion_planning_client` loads the `.pgm` map file, populates a `nav_msgs/OccupancyGrid`, and calls the `motion_planning_server` using the `planning_query` service along with a specified planning algorithm and the start and goal poses as `geometry_msgs/Pose2D`. The client then publishes topics for RViz visualization:

- `/occupancy_grid`: loaded terrain map
- `/visualization_marker_array`: start and goal markers
- `/plan`: server-computed path



## Dependencies

- **ROS 2** (Jazzy)
- **C++ 17**
- Boost (`program_options`)

## Build Instructions

Ubuntu Linux 24.04:

```bash
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash # or setup.zsh for ZSH

# Navigate to workspace and build
cd ~/ros2_ws
colcon build --packages-select ros2_motion_planning
source install/setup.bash
```

## Usage

### 1. Run Server executable

In the first terminal:
```bash
ros2 run ros2_motion_planning motion_planning_server
```

### 2. Run Client executable with CLI arguments

In a second terminal, execute `motion_planning_client` by passing the map file and specifying the desired algorithm (`astar` or `rrt`):

**Using A\* algorithm:**
```bash
ros2 run ros2_motion_planning motion_planning_client \
  --map src/ros2_motion_planning/maps/terrain_open.pgm \
  --algorithm astar \
  --start-x -20.0 --start-y 0.0 --start-theta 0.0 \
  --goal-x 20.0 --goal-y 0.0 --goal-theta 0.0
```

**Using RRT algorithm:**
```bash
ros2 run ros2_motion_planning motion_planning_client \
  --map src/ros2_motion_planning/maps/terrain_maze.pgm \
  --algorithm rrt \
  --start-x -20.0 --start-y 0.0 --start-theta 0.0 \
  --goal-x 20.0 --goal-y 0.0 --goal-theta 0.0
```

### 3. Visualize in RViz

In another terminal, run `rviz2` and add the following displays:
- **Map**: topic `/occupancy_grid`
- **MarkerArray**: topic `/visualization_marker_array`
- **Path**: topic `/plan`
