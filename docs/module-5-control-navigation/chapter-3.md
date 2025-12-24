# Chapter 3: SLAM and Navigation in Unknown Environments

## Introduction

Autonomous navigation in unknown environments is a cornerstone of mobile robotics. This chapter covers Simultaneous Localization and Mapping (SLAM) techniques and the practical application of ROS 2 Navigation2 for enabling robots to perceive, map, and navigate their surroundings.

## Simultaneous Localization and Mapping (SLAM)

SLAM is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of a robot's location within it.

### Core Concepts

-   **Mapping**: Building a representation of the environment (e.g., occupancy grid, feature map).
-   **Localization**: Estimating the robot's pose (position and orientation) within the map.
-   **Data Association**: Matching sensor measurements to map features or grid cells.
*   **Loop Closure**: Recognizing previously visited locations to correct accumulated errors in the map and pose estimate.

### SLAM Algorithms

Various algorithms exist to solve the SLAM problem, each with its strengths and weaknesses:

-   **Kalman Filters (KF)**:
    *   **Extended Kalman Filter (EKF)**: A common variant for SLAM, assuming Gaussian noise and linearizing non-linear system models. Can suffer from linearization errors and computational complexity.
    *   **Unscented Kalman Filter (UKF)**: Better handles non-linearities than EKF but can be more computationally intensive.
*   **Particle Filters**:
    *   **Monte Carlo Localization (MCL)** / **Particle Filter SLAM**: Uses a set of weighted particles (hypotheses) to represent the robot's pose distribution and map. More robust to non-linearities and multi-modal distributions than KFs. Examples include `gmapping` (for 2D occupancy grids) and `slam_toolbox` (more flexible, supports 2D/3D).
*   **Graph-based SLAM**: Represents landmarks and robot poses as nodes in a graph, optimizing the graph to find a globally consistent map and trajectory.

## Autonomous Navigation with ROS 2 Navigation2

ROS 2 Navigation2 is a next-generation navigation stack that provides a comprehensive framework for robot path planning, obstacle avoidance, and localization.

### Key Components of Navigation2

-   **Global Planner**: Plans a path from the robot's current location to the goal over the entire map.
    *   Algorithms: `nav2_navfn_planner` (global planner based on Dijkstra/A*), `nav2_planner_workbench` (allows plugin loading of various global planners).
-   **Local Planner**: Plans short-term velocity commands to follow the global path while avoiding immediate obstacles.
    *   Algorithms: `nav2_smoother` (smooths paths), `nav2_controller` (e.g., DWA, TEB).
-   **Costmaps**: Grid-based representations of the environment, marking obstacles and traversable areas.
    *   **Global Costmap**: Stores the map for long-term planning.
    *   **Local Costmap**: Stores information about the immediate surroundings for obstacle avoidance.
*   **Localization**: Integrates with external localization sources (e.g., AMCL for particle filter localization in known maps).
*   **Behavior Trees**: A powerful tool for orchestrating complex navigation behaviors and decision-making.

### ROS 2 Integration and Examples

ROS 2 provides the necessary tools and infrastructure:
*   **Nodes**: Individual components like planners, controllers, and localizers run as ROS 2 nodes.
*   **Topics**: Data like sensor readings, map updates, and planned paths are communicated via ROS 2 topics.
*   **Actions**: Navigation goals are typically managed via ROS 2 actions, allowing for feedback and cancellation.
*   **Launch System**: ROS 2 launch files (`.launch.py`) are used to start and configure the entire navigation stack.

**Conceptual ROS 2 Launch File Snippet (`nav2_basic_launch.launch.py`):**
```python
# Conceptual ROS 2 launch file for basic Navigation2 setup
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_source import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    # Parameter files
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation time or real time'),
        DeclareLaunchArgument('autostart', default_value='True', description='Automatically start configured nodes'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params_to_override.yaml'), description='Full path to the ROS 2 parameters file to use'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch'), '/bringup_launch.py']),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file
            }.items()
        ),
    ])

```

## Conclusion

Mastering SLAM and autonomous navigation is vital for robots operating in dynamic and unknown environments. ROS 2 Navigation2 provides a robust and flexible framework for implementing these capabilities, leveraging a modular architecture and extensive tooling.
