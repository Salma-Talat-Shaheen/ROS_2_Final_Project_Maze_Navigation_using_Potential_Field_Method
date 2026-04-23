
## **ROS 2 Final Project: Maze Navigation using Potential Field Method - TurtleBot 4** 
<p align="center">
  <img src="https://github.com/Salma-Talat-Shaheen/ROS_2_Final_Project_Maze_Navigation_using_Potential_Field_Method/blob/main/assets/turtlebot4.png?raw=true" width="500" />
</p>

---
This project focuses on developing an advanced autonomous navigation algorithm using the  **Potential Field Methods (PFM)** . The robot autonomously navigates by balancing attractive forces toward a goal and repulsive forces to avoid obstacles in real-time. This approach ensures smooth, safe, and efficient movement, maintaining high Real-time Performance within complex environments.

### Test Environments
The algorithm was rigorously tested across two distinct environments to evaluate its navigation efficiency:

1. Simple Maze
An introductory testing environment featuring open paths and distinct obstacles to evaluate the stability and baseline performance of the algorithm.
<p align="center">
  <img src="https://github.com/Salma-Talat-Shaheen/ROS_2_Final_Project_Maze_Navigation_using_Potential_Field_Method/blob/main/assets/simple_maze.png?raw=true" width="400" />
</p>

2. Complex Maze
An advanced, high-density environment featuring narrow corridors and interlocking obstacles. This environment tests the algorithm's capability to handle Local Minima and determine optimal paths under challenging conditions.
<p align="center">
  <img src="https://github.com/Salma-Talat-Shaheen/ROS_2_Final_Project_Maze_Navigation_using_Potential_Field_Method/blob/main/assets/complex_maze.png?raw=true" width="400" />
</p>

###  System Configuration (ros_gz_bridge)
The `ros_gz_bridge` acts as the vital communication layer, translating messages between the Gazebo transport layer and the ROS 2 DDS middleware.

| Topic | Gazebo Type | ROS 2 Type | Direction |
| :--- | :--- | :--- | :--- |
| `/scan` | `gz.msgs.LaserScan` | `sensor_msgs/LaserScan` | GZ → ROS 2 |
| `/odom` | `gz.msgs.Odometry` | `nav_msgs/Odometry` | GZ → ROS 2 |
| `/cmd_vel` | `gz.msgs.Twist` | `geometry_msgs/Twist` | ROS 2 → GZ |

---

###  Potential Field Navigation Algorithm

#### 1. The Mathematical Model
The navigation logic is based on an  Potential Field Methods (PFM), where the robot moves according to the resultant vector of attractive and repulsive forces.

* **Attractive Force ($F_{att}$):** Generates a pull toward the goal coordinates.
    $$F_{att} = k_{att} \times (P_{goal} - P_{robot})$$
* **Repulsive Force ($F_{rep}$):** Generates a push away from obstacles detected by LiDAR.
    $$F_{rep} = k_{rep} \times \left(\frac{1}{d} - \frac{1}{d_{obs}}\right) \times \frac{1}{d^2}$$

#### 2. Handling LiDAR Data
To ensure numerical stability and prevent system crashes, a robust filtering pipeline was implemented:
* **Zeros & NaNs:** Automatically discarded to avoid division-by-zero errors in the force calculations.
* **Infs:** Treated as the maximum sensor range (5.0m), representing clear paths for navigation.

#### 3. Escape Strategy (Local Minima)

In the **Complex Maze**, standard APF often gets stuck in U-shaped traps. We implemented **Virtual Charges**:
* The robot maintains a `path_history` of its previous coordinates.
* When a stall is detected, these coordinates act as temporary "repulsive charges," pushing the robot out of dead-ends and forcing it to explore new paths toward the goal.

---

### Parameter Tuning
After multiple iterations in the Gazebo environment, we settled on the following gains to balance speed and safety:

| Parameter | Symbol | Simple Maze | Complex Maze | Role |
| :--- | :--- | :--- | :--- | :--- |
| Attractive Gain | $k_{att}$ | 1.2 | 0.4 | Controls the approach velocity to the goal |
| Repulsive Gain | $k_{rep}$ | 1.0 | 4.5 | Determines the strength of obstacle avoidance |
| Influence Dist. | $d_{obs}$ | 1.5 m | 1.5 m | Sets the safety buffer radius around walls |
| Memory Gain | $k_{mem}$ | 0.0 | 3.0 | Strength of spatial memory to avoid traps |
| Goal Tolerance | $d_{goal}$ | 0.2 m | 0.2 m | Defines the arrival radius for a successful stop |

---
#### Setup Reference
```bash
# Clone the necessary repositories
cd ~/ros2_project_ws/src
git clone -b jazzy [https://github.com/turtlebot/turtlebot4.git](https://github.com/turtlebot/turtlebot4.git)
git clone -b jazzy [https://github.com/turtlebot/turtlebot4_simulator.git](https://github.com/turtlebot/turtlebot4_simulator.git)
git clone -b jazzy [https://github.com/turtlebot/turtlebot4_desktop.git](https://github.com/turtlebot/turtlebot4_desktop.git)

# Resolve dependencies
cd ~/ros2_project_ws
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
```

### Execution Commands
Follow these commands to build the workspace and launch the navigation simulation.

#### Step 1: Source

```bash
source /opt/ros/jazzy/setup.bash
```

#### Step 2: Build Workspace
```bash
cd ~/ros2_project_ws
colcon build --symlink-install
source install/setup.bash
```

#### Step 3.1: Launch Simple Maze
```bash
# Terminal 1: Launch Sim
source /opt/ros/jazzy/setup.bash
ros2 launch maze_navigation_finalProject_1 maze_sim.launch.py world:=simple_maze.world

# Terminal 2: Run Planner
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run maze_navigation_finalProject_1 hybrid_planner --ros-args -p goal_x:=9.0 -p goal_y:=9.0
```

#### Step 3.2: Launch Complex Maze
```bash
# Terminal 1: Launch Sim
source /opt/ros/jazzy/setup.bash
ros2 launch maze_navigation_finalProject_1 maze_sim.launch.py world:=complex_maze.world

# Terminal 2: Run Planner
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run maze_navigation_finalProject_1 hybrid_planner --ros-args -p goal_x:=11.4 -p goal_y:=11.4
```

---

Future work will focus on integrating **SLAM (Simultaneous Localization and Mapping)** for dynamic map generation and extending the navigation capabilities to include multi-robot coordination. Furthermore, we aim to deploy the developed algorithm onto physical TurtleBot 4 hardware to validate its real-world navigation performance and robustness. 

---
