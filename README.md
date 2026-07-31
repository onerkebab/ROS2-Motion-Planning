# ROS 2 Motion Planning

![MIT](https://img.shields.io/badge/License-MIT-%23750014) [![ROS](https://img.shields.io/badge/ROS-22314E?logo=ROS&logoColor=white)](#) [![C++](https://img.shields.io/badge/C++-%2300599C.svg?logo=c%2B%2B&logoColor=white)](#)

A ROS 2 package implementing **A\*** and **RRT** (Rapidly-exploring Random Tree) motion planning algorithms for a mobile robot navigating 2D terrain maps, using a client-service architecture. The planner accepts start/goal poses and a map file with grayscale elevation values representing "terrain," or difficulty to traverse. It inflates configuration space obstacles for a cylindrical robot, and computes a minimum-cost collision-free path visualizable in RViz.

## Problem Setup
Plan a collision-free path for a mobile robot from a some starting pose $(x_0, y_0, \theta_0)$ to a goal pose $(x_f, y_f, \theta_f)$ within a 2D terrain map. 

### Robot geometry
* Cylindrical differential-drive robot
  * Can perform turns-in-place and straight drives
* Radius: 0.1 m
* Maximum speed: 0.5 m/s

### 2D terrain map

#### Map format
  * Grayscale `.pgm` file with pixel values ranging from 0 to 255:
    * 255 is most traversable (flat land), 0 is an obstacle cell.
    * Lower pixel values indicate more difficult terrain (e.g. mud, speedbumps).
  * Three map files are provided under `maps/`:
    * `terrain_open.pgm`: Open terrain with smooth elevation gradients and radial terrain patches.
    * `terrain_maze.pgm`: Maze-like environment with cascaded walls and terrain gradients.
    * `terrain_mixed.pgm`: Map featuring circular obstacles and high-cost terrain patches.

#### Terrain cost model (for A* only)

The edge traversal cost in A* scales with terrain difficulty. For an `OccupancyGrid` cell value $c \in [-128, 127]$ (where $c = -128$ is an obstacle), the normalized difficulty $c_{norm} \in [0.0, 1.0]$ and terrain multiplier $M(c)$ are defined as:

$$c_{norm} = \operatorname{clamp}\left(\frac{127 - c}{255}, \, 0.0, \, 1.0\right)$$

$$M(c) = 1.0 + k \cdot c_{norm}$$

With terrain multiplier $k = 4.0$:
* Flat land ($c = +127$, PGM 255): $c_{norm} = 0.0 \implies M(c) = 1.0$ (base distance cost)
* Heavy mud ($c = -127$, PGM 1): $c_{norm} = 1.0 \implies M(c) = 5.0$ ($5\times$ traversal penalty)

For a grid step of distance $d_{\text{step}}$, the movement cost added to $g(n)$ is:

$$\text{move\_cost} = d_{\text{step}} \cdot M(c) + \text{turn\_cost}(\theta_i, \theta_f)$$

> **Note:** Tuning $k$ controls the planner's sensitivity to terrain. Increasing $k$ heavily penalizes high-cost cells, forcing A\* to hug low-cost corridors rather than taking shortcuts through high-cost patches.


## Algorithms

**Motion Planning Algorithms:**

- **A\* Search**: 
  * Deterministic breadth-first search algorithm incorporating an Euclidean heuristic, a 8-connected grid, and terrain cost weighting.
  * Cost function is defined as $f(n) = g(n) + h(n)$ where $g(n)$ is the total accumulated cost up to node $n$, and $h(n)$ is the Euclidean distance from node $n$ to the goal (heuristic). 
  * Guaranteed to find optimal path (lowest cost) with admissible heuristic (Euclidean in this case).

- **RRT**: 
  * Probabilistic path planning algorithm that grows a tree of random samples, with line-segment interpolation for collision awareness.
  * Points are randomly sampled from the map to build out a tree of nodes connected by collision-free, fixed-length line segments until the goal is reached.
  * Does not guarantee optimality and does not care about cost of solution.

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

### Quick Start (Recommended Launch File)

To run the entire motion planning pipeline (server, client, and RViz 2 visualization) seamlessly with a single command, use the provided launch file:

**Using A\* algorithm (default):**
```bash
ros2 launch ros2_motion_planning motion_planning.launch.py
```

**Using RRT algorithm with a different map:**
```bash
ros2 launch ros2_motion_planning motion_planning.launch.py \
  algorithm:=rrt \
  map_name:=terrain_maze.pgm \
  start_x:=-20.0 start_y:=0.0 start_theta:=0.0 \
  goal_x:=20.0 goal_y:=0.0 goal_theta:=0.0
```

> **Note:** The launch file starts the server and RViz 2 (pre-configured with the Map, MarkerArray, and Path displays), then launches the planning client automatically after a brief delay. To run without launching RViz, append `use_rviz:=false`.


### Manual Execution (Separate Terminals)

#### 1. Run Server executable

In the first terminal:
```bash
ros2 run ros2_motion_planning motion_planning_server
```

#### 2. Run Client executable with CLI arguments

In a second terminal, execute `motion_planning_client` by passing the map file and specifying the desired algorithm (`astar` or `rrt`):

**Using A\* algorithm:**
```bash
ros2 run ros2_motion_planning motion_planning_client \
  --map src/ROS2-Motion-Planning/maps/terrain_open.pgm \
  --algorithm astar \
  --start-x -20.0 --start-y 0.0 --start-theta 0.0 \
  --goal-x 20.0 --goal-y 0.0 --goal-theta 0.0
```

**Using RRT algorithm:**
```bash
ros2 run ros2_motion_planning motion_planning_client \
  --map src/ROS2-Motion-Planning/maps/terrain_maze.pgm \
  --algorithm rrt \
  --start-x -20.0 --start-y 0.0 --start-theta 0.0 \
  --goal-x 20.0 --goal-y 0.0 --goal-theta 0.0
```

#### 3. Visualize in RViz

In another terminal, run `rviz2` and add the following displays:
- **Map**: topic `/occupancy_grid`
- **MarkerArray**: topic `/visualization_marker_array`
- **Path**: topic `/plan`
