# Two-Wheeled Differential Drive Cleaning Robot with SLAM and Navigation

  

This ROS-based project simulates a two-wheeled differential drive cleaning robot capable of performing **SLAM using GMapping** and **localization using AMCL** within a Gazebo environment. The robot is designed for autonomous indoor navigation and mapping.


![3](https://github.com/user-attachments/assets/d063f21e-ae7c-4352-8090-3cb5ff9c3960)


---
## Features

- Simulated in Gazebo with LIDAR and differential drive
- Teleoperation control (including lowering/rising of Scrubber)
- SLAM with GMapping
- Localization with AMCL
- RViz visualization
- Navigation to user-defined goals

---
## Package Structure

- `mb_control/`: Model Control Files including custom teleop
- `mb_description/`: URDF and robot configuration
- `mb_gazebo/`: Gazebo world and launch files
- `mb_navigation/`: Navigation stack, mapping, AMCL, and SLAM

---
## Modes of Operation

### 🔹 GMapping (SLAM) Mode

(In separate terminals)

1. Start Gazebo:

 ```bash
 $ roslaunch mb_gazebo mb_gazebo.launch
 ```
   
2. Launch SLAM:

 ```bash
 $ roslaunch mb_navigation gmapping_demo.launch
 ```
   
3. Start Teleoperation:

 ```bash
 $ rosrun mb_navigation mb_teleop.py
 ```
   
4. Launch RViz:

 ```bash
 $ roslaunch mb_description bot_rviz_gmapping.launch
 ```
   
  
**RViz Settings:**

- Map: `/map`
- LaserScan: `/MB/laser_scan`
- Robot Model

📷 Example:

![1](https://github.com/user-attachments/assets/0e0959ce-adaa-40a3-90c0-18366ab52bcc)

![Screenshot 2025-05-05 202436](https://github.com/user-attachments/assets/91ca5c11-e35b-4b4f-a84c-f31c1c152f66)


---
### 🔹 AMCL (Localization) Mode

(In separate terminals)

1. Start Gazebo:

 ```bash
 $ roslaunch mb_gazebo mb_gazebo.launch
 ```

2. Launch AMCL:

 ```bash
 $ roslaunch mb_navigation amcl_demo.launch
 ```
   
3. Launch RViz:

 ```bash
 $ roslaunch mb_description bot_rviz_amcl.launch
 ```
  

**RViz Settings:**

- Map: 
  - `/map`
  - `/move_base/global_costmap/costmap`
  - `/move_base/local_costmap/costmap`
- LaserScan: `/MB/laser_scan`
- Global Path: `/move_base/DWAPlannerROS/global_plan`
- Robot Model

*Use "2D Nav Goal" in RViz to command navigation.*


📷 Example:


![Screenshot 2025-05-05 203458](https://github.com/user-attachments/assets/cc50b753-55dd-4187-8552-91b0cfe86acf)

  
---

## Dependencies

- ROS (tested with Noetic)
- `gmapping`
- `amcl`
- `move_base`
- `teleop_twist_keyboard` or custom `mb_teleop.py`
- `gazebo_ros`
- `rviz`

---

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/ONESHO1/MB_differential_drive_SLAM.git
cd ..
catkin_make
source devel/setup.bash
```

