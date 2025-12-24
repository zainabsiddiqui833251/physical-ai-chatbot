---
id: chapter-4
title: Robotics Basics – Coordinate Frames & DH Parameters
sidebar_label: 'Chapter 4: Frames & DH'
week: 'Weeks 3-5'
tags: [robotics, kinematics, frames, DH parameters]
---

## Introduction
Coordinate frames and the Denavit–Hartenberg (DH) convention form the foundation of robot kinematics. They provide a systematic way to describe the position and orientation of every link in a robot arm or leg. Mastering these concepts is essential for anyone designing, programming, or simulating humanoid robots.

This chapter explains everything in clear, detailed steps using only plain text and tables — no complex symbols or formats that could cause issues.

## Why We Need Coordinate Frames
A robot manipulator consists of rigid links connected by joints. Each link can move relative to the previous one. To predict where the hand (end-effector) will be when the joints move, we need a reliable reference system.

Coordinate frames solve this problem by attaching a set of three perpendicular axes (X, Y, Z) with an origin to specific points on the robot:
- One frame at the base (fixed to the ground or torso)
- One frame at each joint or link
- One final frame at the gripper or tool tip

With frames defined, we can describe any point on the robot using simple coordinates relative to any frame we choose.

## Homogeneous Transformation Matrices
To go from one frame to the next, robotics uses a single 4 by 4 matrix called a homogeneous transformation matrix.

This matrix packs two things together:
- Rotation information (how the axes are oriented differently)
- Translation information (how far apart the origins are)

The general layout of this matrix always looks the same:
- Top-left 3 by 3 part: rotation
- Top-right 3 by 1 column: position
- Bottom row: always 0 0 0 1

Being able to multiply these matrices together is what makes forward kinematics possible — we just chain all the individual link transformations from base to tip.

## The Denavit–Hartenberg (DH) Convention
The DH convention is the worldwide standard for assigning coordinate frames to robot links in a consistent way. It removes ambiguity and makes models easy to share between researchers, companies, and software tools.

The convention defines exactly four parameters that completely describe the geometric relationship between two consecutive links/joints.

### Detailed Explanation of the Four DH Parameters

| Parameter | Full Name      | Detailed Description                                                                                           | Direction of Measurement                          |
|-----------|----------------|----------------------------------------------------------------------------------------------------------------|---------------------------------------------------|
| a         | Link length    | The distance between the two joint axes measured along their common perpendicular (shortest distance).         | Along the X-axis of the previous frame             |
| α         | Link twist     | The angle between the two joint axes when viewed along the common perpendicular.                               | Rotation about the X-axis of the previous frame    |
| d         | Link offset    | The signed distance along the previous joint axis from the common perpendicular to the current frame origin.   | Along the Z-axis of the previous frame            |
| θ         | Joint angle    | The angle needed around the previous joint axis to align the previous X-axis with the current X-axis.          | Rotation about the Z-axis of the previous frame    |

Key points:
- For revolute (rotary) joints — very common in humanoids — the joint variable is θ (it changes as the joint rotates).
- For prismatic (linear sliding) joints — less common in humanoids — the joint variable is d.
- The other parameters (a and α) are fixed by the mechanical design of the robot.

### The Standard Transformation Matrix Between Consecutive Frames
The DH convention produces one transformation matrix for each link. This matrix takes points from the current frame back to the previous frame.

The matrix is always structured the same way. Here is the complete content written row by row in full detail:

**Row 1**  
cosine of θ, negative sine of θ times cosine of α, sine of θ times sine of α, a times cosine of θ

**Row 2**  
sine of θ, cosine of θ times cosine of α, negative cosine of θ times sine of α, a times sine of θ

**Row 3**  
zero, sine of α, cosine of α, d

**Row 4**  
zero, zero, zero, one

When you multiply all these matrices together (starting from the base), you get the final pose of the end-effector relative to the robot base — this is called forward kinematics.

## Step-by-Step Rules for Assigning Frames (Classic DH Convention)
Follow these exact steps every time you model a robot:

1. Align the Z-axis of frame i with the axis of motion of joint i+1 (direction the joint rotates or slides).
2. Place the origin of frame i where the common perpendicular between joint i and joint i+1 meets joint i+1 axis.
3. Set the X-axis of frame i along that common perpendicular, pointing from joint i axis toward joint i+1 axis.
4. Set the Y-axis to complete a right-handed coordinate system.
5. Repeat for every link.

These rules guarantee that only four simple parameters are needed between any two frames.

## Real-World Applications
Understanding DH parameters is required for:
- Creating accurate URDF or SDF model files in ROS 2
- Configuring motion planning packages like MoveIt
- Reading technical documentation from robot manufacturers (Fanuc, KUKA, Boston Dynamics, Tesla Optimus, etc.)
- Debugging simulation vs real robot mismatches
- Designing custom humanoid arms or legs from scratch

## Practice Questions
1. List the four DH parameters in order and explain which one changes when a typical humanoid elbow joint moves.
2. Describe in your own words why the bottom row of every homogeneous transformation matrix is always 0 0 0 1.
3. For a robot arm with perpendicular joint axes (like a shoulder), what will the twist parameter α usually be?
4. Sketch on paper a simple two-link robot lying flat on a table. Label the base frame, both joint axes, and identify a, α, d, and θ for the second link.
5. Mini-project idea: Look up the public URDF file of any open-source humanoid (like iCub or Valkyrie) and find the DH parameters section — try to match them to the physical robot photos.

