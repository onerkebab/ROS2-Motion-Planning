# Problem Setup
Plan a collision-free path for a mobile robot from a some starting pose $(x_0, y_0, \theta_0)$ to a goal pose $(x_f, y_f, \theta_f)$ within a 2D terrain map. 

## Robot geometry
* Cylindrical differential-drive robot
  * Can perform turns-in-place and straight drives
* Radius: 0.1 m
* Maximum speed: 0.5 m/s

## 2D terrain map

### Map format
  * Grayscale `.pgm` file with pixel values ranging from 0 to 255:
    * 255 is most traversable (flat land), 0 is an obstacle cell.
    * Lower pixel values indicate more difficult terrain (e.g. mud, speedbumps).
  * Three map files are provided under `maps/`:
    * `terrain_open.pgm`: Open terrain with smooth elevation gradients and radial terrain patches.
    * `terrain_maze.pgm`: Maze-like environment with cascaded walls and terrain gradients.
    * `terrain_mixed.pgm`: Map featuring circular obstacles and high-cost terrain patches.

### Terrain cost model (for A* only)

The edge traversal cost in A* scales with terrain difficulty. For an `OccupancyGrid` cell value $c \in [-128, 127]$ (where $c = -128$ is an obstacle), the normalized difficulty $c_{norm} \in [0.0, 1.0]$ and terrain multiplier $M(c)$ are defined as:

$$c_{norm} = \text{clamp}\left(\frac{127 - c}{255}, \, 0.0, \, 1.0\right)$$

$$M(c) = 1.0 + k \cdot c_{norm}$$

With terrain multiplier $k = 4.0$:
* Flat land ($c = +127$, PGM 255): $c_{norm} = 0.0 \implies M(c) = 1.0$ (base distance cost)
* Heavy mud ($c = -127$, PGM 1): $c_{norm} = 1.0 \implies M(c) = 5.0$ ($5\times$ traversal penalty)

For a grid step of distance $d_{\text{step}}$, the movement cost added to $g(n)$ is:

$$\text{movecost} = d_{\text{step}} \cdot M(c) + \text{turncost}(\theta_i, \theta_f)$$

> **Note:** Tuning $k$ controls the planner's sensitivity to terrain. Increasing $k$ heavily penalizes high-cost cells, forcing A\* to hug low-cost corridors rather than taking shortcuts through high-cost patches.