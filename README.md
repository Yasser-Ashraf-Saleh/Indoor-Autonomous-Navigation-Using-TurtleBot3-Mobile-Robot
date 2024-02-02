# Indoor-Autonomous-Navigation-Using-TurtleBot3-Mobile-Robot 

## Introduction
This project explores advanced techniques for indoor mapping using the TurtleBot3 Mobile Robot, focusing on image enhancement, occupancy grid creation, path planning, and trajectory tracking. Our goal is to improve autonomous navigation in complex indoor environments.

## Project Objectives
* Develop a robust method for real-time indoor mapping.
* Enhance image processing for clearer environmental representation.
* Implement efficient path planning and trajectory tracking algorithms.

## Methodology
The proposed approach for operating the turtle bot as a path planner consists of four main steps: image enhancement, creating an occupancy grid, path planning, and trajectory tracking.

### Image Enhancement:

* Utilizes median filtering to remove noise and enhance the accuracy of map data.
* Helps in eliminating discontinuities and undesired obstacles from the map.
* Median filtering preserves significant edges while removing most of the noise.
* Crop the map to define the region of interest and reduce the complexity of planning algorithms.
* Thresholding is applied to increase contrast, which is necessary for creating an occupancy map.
### Occupancy Grid:

*  Occupancy grid mapping is used to convert the map into a binary grid (occupied or unoccupied).
* Pixels' intensity is considered, and a distinct threshold is set to represent occupied and unoccupied areas.
* The algorithm doesn't consider the robot's size initially, so the robot's radius is added using the inflate function in MATLAB.
* The map is prepared for the application of the path planning algorithm.
### Path Planning:

* The Probabilistic Roadmap (PRM) algorithm is employed for finding a path between two points on the map.
* PRM involves sampling nodes from random locations in the free space of the map and connecting them to form random paths.
* The desired path is determined by connecting points on the set of lines acquired through the sampling process.
* Two crucial parameters for the efficiency of the algorithm are the number of sampled nodes and the connection distance.
### Trajectory Tracking:

* Utilizes a PID controller algorithm for tracking reference trajectories smoothly and efficiently.
* The trajectory is generated using the PRM algorithm for path planning.
* The PID controller algorithm is implemented to make the robot follow the generated trajectory in a smooth and fast manner.


In summary, the approach combines image enhancement techniques, occupancy grid mapping, PRM path planning, and PID-based trajectory tracking to enable the turtlebot3 to navigate from a specified start point to an endpoint in a given environment. The integration of these components aims to provide accurate mapping, efficient path planning, and effective trajectory tracking for the robot.
### Image Enhancement
We applied various filters to improve image quality, essential for accurate mapping. This step ensures that the occupancy grid and path planning algorithms have the best possible data quality.

### Occupancy Grid Creation
Using the enhanced images, we generated binary occupancy grids that represent the accessible and obstructed areas within the environment.

### Path Planning and Trajectory Tracking
We designed algorithms to plan optimal paths based on the occupancy grid and developed a trajectory tracking system to ensure the TurtleBot3 could navigate these paths accurately.

## Experiments & Results
Results:

### Path Interpolation:

- Linear interpolation was initially attempted for the RPM-generated path, but it proved challenging for the robot to handle sudden changes.
- Cubic spline interpolation was then employed to smooth the path, showing improved performance in robot trajectory following.
### PID Tuning:

- The PID parameters were tuned for the robot to follow both trajectories efficiently.
- Delay was found to be effective in handling percentage overshoot, contributing to better trajectory tracking.
### Path Characteristics (PRM Output):

- Figures 10 and 11 display the first and second paths, revealing noise in the image treated as obstacles.
- The paths exhibit non-linearity and fluctuation due to the randomness in generated nodes, impacting the algorithm's nature.
- Algorithm suffers from randomness and lacks smoothness in path generation.
### Efficiency and Adaptability:

The results demonstrate efficiency in tracking complex trajectories, and PID parameters remain effective across different paths.
Changing the delay influences fluctuation percentage, providing adaptability to varying conditions.

## Conclusion
- The implementation of a delivery robot using the turtlebot3 involved static mapping, planning, and dynamic navigation with a PID controller and odometer sensor for environment perception.
- Challenges included accurate environment scaling, time constraints, path smoothing, and dealing with partially observable environments.
- Despite the PRM planner generating non-smooth paths with fluctuations, the results indicate overall smoothness and effectiveness in reaching desired trajectories.
