---
id: 'chapter-2-kinematics'
title: Forward and Inverse Kinematics
sidebar_label: "Chapter 2: Kinematics"
week: 'Weeks 3–5'
tags: [robotics, kinematics, forward kinematics, inverse kinematics]
---

# Forward and Inverse Kinematics in Robotics

## Introduction

This chapter delves into the fundamental concepts of Forward Kinematics (FK) and Inverse Kinematics (IK) for robotic manipulators. Understanding these principles is essential for predicting the end-effector's pose given joint states (FK) and for determining the joint configurations required to achieve a desired end-effector pose (IK).

## 1. Forward Kinematics (FK)

Forward Kinematics is the process of calculating the end-effector's position and orientation in Cartesian space, given the joint variables (angles for revolute joints, displacements for prismatic joints).

### 1.1. FK for a Single Joint

*   For a single revolute joint, FK determines the end-effector's orientation around the joint axis.
*   For a single prismatic joint, FK determines the end-effector's linear displacement along the joint axis.

### 1.2. FK for a Multi-DOF Manipulator

*   The total FK of a manipulator is found by composing the transformations of individual joints.
*   This involves multiplying transformation matrices from the base frame to the end-effector frame.

## 2. Inverse Kinematics (IK)

Inverse Kinematics is the process of calculating the required joint variables to achieve a desired end-effector pose (position and orientation) in Cartesian space. This is generally a more complex problem than FK.

### 2.1. The Challenge of IK

*   IK problems can have multiple solutions, a single solution, or no solution (if the target pose is outside the workspace).
*   Computational methods vary in complexity and accuracy.

### 2.2. Analytical vs. Numerical IK

*   **Analytical IK**: Solvable for simpler robots with closed-form solutions (e.g., some 6-DOF arms). Faster and exact.
*   **Numerical IK**: Uses iterative methods (e.g., Jacobian-based methods, optimization) to find solutions. Applicable to more complex robots but can be slower and may find local minima.

## 3. Kinematic Chains and Representations

Representing the robot's structure is key to solving kinematic problems.

### 3.1. Homogeneous Transformation Matrices

*   Used to represent the position and orientation of one coordinate frame with respect to another.
*   A 4x4 matrix combining rotation and translation.

### 3.2. Denavit-Hartenberg (DH) Convention

*   A standard method for assigning coordinate frames to the links of a serial-link manipulator and describing transformations between them.
*   Uses four parameters per link.

## 4. Visualizing Kinematics

*(Placeholder for Mermaid diagram showing a kinematic chain and transformations)*

```mermaid
graph TD
    A[Base Frame] --> B(Link 1);
    B -- FK --> C(End Effector Pose);
    D[Desired Pose] -- IK --> E(Joint Variables);
    E --> B;
```

## 5. Python Code Examples

Example of calculating FK for a simple 2-DOF arm:

```python
import numpy as np

def forward_kinematics_2d_planar(joint_angles, link_lengths):
    """
    Calculates the end-effector position for a 2D planar arm.
    Assumes revolute joints.
    """
    l1, l2 = link_lengths
    theta1, theta2 = joint_angles

    # Transformation for link 1 (simplified)
    T1 = np.array([
        [np.cos(theta1), -np.sin(theta1), 0, l1 * np.cos(theta1)],
        [np.sin(theta1), np.cos(theta1), 0, l1 * np.sin(theta1)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # Transformation for link 2 (simplified)
    T2 = np.array([
        [np.cos(theta2), -np.sin(theta2), 0, l2 * np.cos(theta2)],
        [np.sin(theta2), np.cos(theta2), 0, l2 * np.sin(theta2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # Total transformation T_0_2 = T1 @ T2
    T_0_2 = T1 @ T2

    # End effector position is the translation part of the final matrix
    end_effector_pos = T_0_2[:2, 3]
    return end_effector_pos

# Example usage
# joint_angles = [np.pi/4, np.pi/4] # 45 degrees for both joints
# link_lengths = [1.0, 1.0]
# position = forward_kinematics_2d_planar(joint_angles, link_lengths)
# print(f"End effector position: {position}")