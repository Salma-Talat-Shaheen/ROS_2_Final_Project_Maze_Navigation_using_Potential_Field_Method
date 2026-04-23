
# ROS 2 Final Project: Autonomous Maze Navigation
## Robot: TurtleBot 4 Standard | Team: Shahd & Salma

---

### 4. System Configuration (ros_gz_bridge)
The `ros_gz_bridge` acts as the vital communication layer, translating messages between the Gazebo transport layer and the ROS 2 DDS middleware.

| Topic | Gazebo Type | ROS 2 Type | Direction |
| :--- | :--- | :--- | :--- |
| `/scan` | `gz.msgs.LaserScan` | `sensor_msgs/LaserScan` | GZ → ROS 2 |
| `/odom` | `gz.msgs.Odometry` | `nav_msgs/Odometry` | GZ → ROS 2 |
| `/cmd_vel` | `gz.msgs.Twist` | `geometry_msgs/Twist` | ROS 2 → GZ |

**Issues Resolved:**
1. **Sensor Activation:** Modified `simple_maze.world` to include the `gz::sim::systems::Sensors` plugin, which was missing in the skeleton, ensuring LiDAR data generation.
2. **Namespace Mapping:** Implemented remapping logic to bridge the robot's hardware interface with our custom planner node effectively.

---

### 5. Potential Field Navigation Algorithm
#### 5.1 The Mathematical Model
The navigation logic is based on an Artificial Potential Field (APF), where the robot moves according to the resultant vector of attractive and repulsive forces.

* **Attractive Force ($F_{att}$):** Generates a pull toward the goal coordinates.
    $$F_{att} = k_{att} \times (P_{goal} - P_{robot})$$
* **Repulsive Force ($F_{rep}$):** Generates a push away from obstacles detected by LiDAR.
    $$F_{rep} = k_{rep} \times \left(\frac{1}{d} - \frac{1}{d_{obs}}\right) \times \frac{1}{d^2}$$
    *(Where $d$ is the current distance to an obstacle and $d_{obs}$ is the influence threshold).*

#### 5.2 Handling LiDAR Data
To ensure numerical stability and prevent system crashes, a robust filtering pipeline was implemented:
* **Zeros & NaNs:** Automatically discarded to avoid division-by-zero errors in the force calculations.
* **Infs:** Treated as the maximum sensor range (5.0m), representing clear paths for navigation.

#### 5.3 Escape Strategy (Local Minima)
In the **Complex Maze**, standard APF often gets stuck in U-shaped traps. We implemented **Virtual Charges**:
* The robot maintains a `path_history` of its previous coordinates.
* When a stall is detected, these coordinates act as temporary "repulsive charges," pushing the robot out of dead-ends and forcing it to explore new paths toward the goal.

---

### 6. Parameter Tuning
After multiple iterations in the Gazebo environment, we settled on the following gains to balance speed and safety:

| Parameter | Symbol | Value | Role |
| :--- | :--- | :--- | :--- |
| Attractive Gain | $k_{att}$ | 0.5 | Controls the approach velocity to the goal |
| Repulsive Gain | $k_{rep}$ | 1.0 | Determines the strength of obstacle avoidance |
| Influence Dist. | $d_{obs}$ | 1.0 m | Sets the safety buffer radius around walls |
| Goal Tolerance | $d_{goal}$ | 0.2 m | Defines the arrival radius for a successful stop |

---

### 7. Execution Commands
Follow these commands to build the workspace and launch the navigation simulation.

#### Step 1: Build Workspace
```bash
cd ~/ros2_project_ws
colcon build --symlink-install
source install/setup.bash
```

#### Step 2: Launch Simple Maze
```bash
# Terminal 1: Launch Sim
source /opt/ros/jazzy/setup.bash
ros2 launch maze_navigation_finalProject_1 maze_sim.launch.py world:=simple_maze.world
# Terminal 2: Run Planner
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run maze_navigation_finalProject_1 hybrid_planner --ros-args -p goal_x:=9.0 -p goal_y:=9.0
```

#### Step 3: Launch Complex Maze (Bonus)
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

### 8. Individual Contribution Statement

**Student A: SHAHD EYAD YOUSEF ETHALATHINI**
* **Responsibilities:** Robot selection and integration, Gazebo environment configuration, plugin debugging (`Sensors` system), `ros_gz_bridge` setup, and orchestration of ROS 2 launch files.

**Student B: SALMA TALAT SHAHEEN**
* **Responsibilities:** Core APF algorithm development, LiDAR/Odometry data sanitization, implementation of the "Virtual Charges" escape strategy, and fine-tuning control parameters.
