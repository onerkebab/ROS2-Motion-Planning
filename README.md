# ROS 2 Motion Planning

![MIT](https://img.shields.io/badge/License-MIT-%23750014) [![ROS](https://img.shields.io/badge/ROS-22314E?logo=ROS&logoColor=white)](#) [![C++](https://img.shields.io/badge/C++-%2300599C.svg?logo=c%2B%2B&logoColor=white)](#) [![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=fff)](#)

A ROS 2 package implementing **A\* Search** and **RRT** (Rapidly-exploring Random Tree) motion planning algorithms for a mobile robot navigating 2D terrain maps, using a client-service architecture. The planner accepts start/goal poses and a map file with grayscale elevation values representing "terrain," or difficulty to traverse. It inflates configuration space obstacles for a cylindrical robot, and computes a minimum-cost collision-free path visualizable in RViz. 

In the demo GIFs below, red pixels are obstacles, and terrain difficulty increases in the following order: green (no penalty) > white > black > yellow > orange (highest penalty). A visualization of the search process is also shown, with closed-list nodes for A* Search and the tree for RRT.

<img src="assets/Astaropen.gif" width="49%" /> <img src="assets/RRTopen.gif" width="49%" />

> Read full problem setup in [ProblemSetup.md](ProblemSetup.md).

## Algorithms

- **A\* Search**: 
  * Deterministic breadth-first search algorithm incorporating an Euclidean heuristic, a 8-connected grid, and terrain cost weighting.
  * Cost function is defined as $f(n) = g(n) + h(n)$ where $g(n)$ is the total accumulated cost up to node $n$, and $h(n)$ is the Euclidean distance from node $n$ to the goal (heuristic). 
  * Guaranteed to find optimal path (lowest cost) with admissible heuristic (Euclidean in this case).

<img src="assets/Astarmaze.gif" width="49%" /> <img src="assets/Astarmixed.gif" width="49%" />

> A* yields the most optimal solution every time, following easiest terrain path.

- **RRT**: 
  * Probabilistic path planning algorithm that grows a tree of random samples, with line-segment interpolation for collision awareness.
  * Nodes $q_{rand}$ are randomly sampled from the map to build out new nodes $q_{new}$ connected to the nearest node in the tree $q_{near}$ via collision-free, fixed-length line segments until the goal is reached.
  * Does not guarantee optimality and does not care about cost of solution.

<img src="assets/RRTmazefast.gif" width="32%" /> <img src="assets/RRTmazeslow.gif" width="32%" /> <img src="assets/RRTmazearound.gif" width="32%" />

> RRT can find the same-ish path very fast, or very slow, or a completely unoptimized path through terrain.

Once at the goal node, both algorithms backtrack using backpointers to reconstruct the path from goal to start, which is then published as a sequence of 2D poses.

For both algorithms, the input map's obstacle cells are inflated by the robot's physical radius to generate a C-space for collision-free planning.

## Maps

Three terrain map files are provided under `maps/`:

- `terrain_open.pgm`: Open terrain with smooth gradient patches.
- `terrain_maze.pgm`: Maze-like environment with cascaded walls and terrain gradients.
- `terrain_mixed.pgm`: Map featuring circular obstacles and high-cost terrain patches.

Terrain maps should only affect the solution to A* Search (cost-aware), not RRT (cost-agnostic).

## ROS Architecture

The project uses a client-service architecture with the client executable configured to take CLI arguments for map file, algorithm, and start and goal poses, and the server executable to run the planning algorithms.

**Service Definition:**
`srv/MotionPlanningService.srv`
```srv
geometry_msgs/Pose2D start
geometry_msgs/Pose2D goal
nav_msgs/OccupancyGrid map
string algorithm
bool animate
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

## Build

Ubuntu Linux 24.04

```bash
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash # or setup.zsh for ZSH

# Clone repo to ROS workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/onerkebab/ROS2-Motion-Planning

# Build package
cd ~/ros2_ws
colcon build --packages-select ros2_motion_planning
source install/setup.bash
```

## Usage

### Launch file (Recommended)

To run the entire motion planning pipeline (server, client, and RViz 2 visualization), use the provided launch file. By default, `animate` is set to false and search visualization is disabled. Setting `animate` to true enables visualization of closed list for A* and exploration tree for RRT.

**Using A\* algorithm with search visualization on mixed terrain map:**
```bash
ros2 launch ros2_motion_planning motion_planning.launch.py \
  algorithm:=astar \ 
  map:=terrain_mixed.pgm \
  animate:=true \
  start_x:=-20.0 start_y:=0.0 start_theta:=0.0 \
  goal_x:=20.0 goal_y:=0.0 goal_theta:=0.0
```

**Using RRT algorithm with search visualization on maze terrain map:**
```bash
ros2 launch ros2_motion_planning motion_planning.launch.py \
  algorithm:=rrt \
  map:=terrain_maze.pgm \
  animate:=true \
  start_x:=-20.0 start_y:=0.0 start_theta:=0.0 \
  goal_x:=20.0 goal_y:=0.0 goal_theta:=0.0
```
> The terrain map will occasionally fail to load initially when running with `animate:=true`, but will appear at the end of the search.

> To run without launching RViz, add `use_rviz:=false` to the launch command.

## License
This project is licensed under the [MIT License](LICENSE).
