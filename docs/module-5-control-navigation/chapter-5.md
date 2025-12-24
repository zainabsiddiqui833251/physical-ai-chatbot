# Chapter 5: Grasp Planning and In-Hand Manipulation

## Introduction

Effective interaction with the physical world hinges on a robot's ability to grasp objects securely and manipulate them precisely. This chapter explores the principles of grasp planning and the sophisticated techniques involved in in-hand manipulation, with a focus on practical application using frameworks like MoveIt in ROS 2.

## Grasp Planning

Grasp planning involves determining how a robot's end-effector (gripper) should interact with an object to achieve a stable hold. This requires understanding object geometry, grasp affordances, and the robot's own capabilities.

### Key Concepts

-   **Grasp Types**:
    *   **Power Grasp**: Enveloping the object with fingers and palm to exert high forces. Suitable for holding tools or irregular objects.
    *   **Pinch Grasp**: Using fingertips to hold small objects, typically involving precision forces. Examples include tip pinch, lateral pinch, and three-finger pinch.
    *   **Force Closure**: A grasp where the object cannot be moved arbitrarily by external forces without requiring a change in gripper forces.
-   **Grasp Affordances**: Properties of an object that suggest how it can be grasped (e.g., handle for grasping, flat surface for suction).
-   **Grasp Quality Metrics**: Evaluating grasps based on stability, force resistance, and dexterity.

### Grasp Planning Algorithms

Algorithms aim to find optimal grasps based on sensory input (e.g., from cameras, depth sensors) and object models.
*   **Geometric Methods**: Analyzing object geometry to find stable grasp points.
*   **Data-Driven Methods**: Using machine learning models trained on large datasets of successful grasps.
*   **Sampling-based Methods**: Exploring the space of possible grasps to find suitable ones.

## In-Hand Manipulation

In-hand manipulation refers to the ability of a robot hand to reorient or adjust an object while it is being held, without dropping it. This is crucial for tasks requiring fine adjustments or repositioning.

### Strategies for In-Hand Manipulation

-   **Object Reorientation**: Techniques to change an object's pose using finger movements and sliding.
-   **Finger Gaiting**: Complex coordination of finger articulations to slide and rotate objects.
-   **Dexterous Grasp Maintenance**: Maintaining a stable grasp while performing manipulation.

## MoveIt Examples for Grasping and Manipulation

MoveIt is a powerful motion planning framework for ROS that supports grasping and manipulation tasks.

### Conceptual MoveIt Integration

*   **Robot Model (URDF/XACRO)**: Must accurately describe the robot's arm, hand, and gripper, including their kinematics and collision geometry.
*   **Planning Scene**: The environment representation including the robot, objects to be grasped, and obstacles.
*   **Grasp Planning Pipeline**:
    1.  **Perception**: Detect objects and estimate their poses.
    2.  **Grasp Candidate Generation**: Propose potential grasps based on object geometry and gripper capabilities.
    3.  **Grasp Filtering/Scoring**: Evaluate candidate grasps for stability and feasibility.
    4.  **Motion Planning**: Generate collision-free trajectories for the arm and gripper to execute the grasp.
    5.  **Execution**: Send the planned trajectory to the robot's controllers.

**Conceptual Python Example Snippet (`src/manipulation/moveit_grasp_examples.py`):**
```python
# Conceptual Python snippet for MoveIt grasp planning example
import rospy # Assuming ROS 1 for conceptual example, but MoveIt2 in ROS 2
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import Grasp, GripperTranslation, RobotTrajectory
from geometry_msgs.msg import PoseStamped
import sys

# Placeholder for conceptual example - actual implementation would be extensive
class MoveItGraspPlanner:
    def __init__(self):
        # Initialize MoveIt Commander and Planning Scene Interface
        self.move_group = MoveGroupCommander("manipulator") # Name of the move group for the arm
        self.scene = PlanningSceneInterface()
        # ... other initializations ...

    def plan_and_execute_grasp(self, object_pose, grasp_pose):
        # 1. Add object to planning scene
        self.scene.add_mesh("object_mesh", object_pose, "object.stl") # Conceptual mesh

        # 2. Define grasp approach and retreat (conceptual)
        approach_translation = self.define_approach_translation()
        retreat_translation = self.define_retreat_translation()

        # 3. Create a Grasp object
        grasp = Grasp()
        grasp.pre_grasp_approach.direction.header.frame_id = self.move_group.get_planning_frame()
        grasp.pre_grasp_approach.direction.vector.x = 1.0 # Example approach direction
        grasp.pre_grasp_approach.min_distance = 0.05
        grasp.pre_grasp_approach.desired_distance = 0.1
        grasp.post_grasp_retreat.direction.header.frame_id = self.move_group.get_planning_frame()
        grasp.post_grasp_retreat.direction.vector.x = -1.0 # Example retreat direction
        grasp.post_grasp_retreat.min_distance = 0.01
        grasp.post_grasp_retreat.desired_distance = 0.03

        grasp.grasp_pose = grasp_pose # Target pose for the gripper
        grasp.pre_grasp_posture.joint_positions = [1.57, 0.1, 0.1] # Example pre-grasp joint configuration
        grasp.post_grasp_posture.joint_positions = [1.57, 0.1, 0.1] # Example post-grasp joint configuration
        grasp.max_contact_force = 10.0 # Example force limit
        grasp.allowed_touch_objects = ["object_mesh"] # Allow touching the object

        # 4. Plan motion
        grasp_plan_success = self.move_group.plan_grasp(grasp) # Conceptual MoveIt call

        if grasp_plan_success:
            # 5. Execute motion
            self.move_group.execute_grasp(grasp)
            rospy.loginfo("Grasp execution successful.")
        else:
            rospy.logerr("Grasp planning failed.")

    def define_approach_translation(self):
        # Define approach translation (e.g., down along Z axis)
        trans = GripperTranslation()
        trans.direction.header.frame_id = self.move_group.get_planning_frame()
        trans.direction.vector.z = 1.0
        trans.min_distance = 0.05
        trans.desired_distance = 0.1
        return trans

    def define_retreat_translation(self):
        # Define retreat translation (e.g., up along Z axis)
        trans = GripperTranslation()
        trans.direction.header.frame_id = self.move_group.get_planning_frame()
        trans.direction.vector.z = -1.0
        trans.min_distance = 0.01
        trans.desired_distance = 0.03
        return trans

if __name__ == '__main__':
    rospy.init_node('moveit_grasp_example', anonymous=True)
    planner = MoveItGraspPlanner()
    # Example: Define object pose and grasp pose (would typically come from perception)
    object_pose = PoseStamped()
    object_pose.header.frame_id = "base_link"
    object_pose.pose.position.x = 0.5
    object_pose.pose.position.y = 0.0
    object_pose.pose.position.z = 0.1
    # object_pose.pose.orientation = ...

    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "base_link"
    grasp_pose.pose.position.x = 0.5
    grasp_pose.pose.position.y = 0.0
    grasp_pose.pose.position.z = 0.15 # Slightly above object for pre-grasp
    # grasp_pose.pose.orientation = ...

    planner.plan_and_execute_grasp(object_pose, grasp_pose)
```

## Conclusion

Mastering grasp planning and in-hand manipulation is vital for robots to interact effectively with the physical world. Frameworks like MoveIt in ROS 2 provide powerful tools to implement these complex capabilities, enabling robots to perform sophisticated tasks from simple picking to dexterous object reorientation.
