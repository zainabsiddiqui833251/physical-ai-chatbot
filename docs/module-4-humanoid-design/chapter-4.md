# Chapter 4: Materials, Actuators, and Power Systems

## Introduction

Selecting the right materials, actuators, and power systems is fundamental to building functional and cost-effective humanoid robots. This chapter explores common choices and considerations for these critical components, emphasizing real-world buildability.

## Materials for Robot Construction

Choosing appropriate materials impacts a robot's strength, weight, cost, and manufacturability.

### Common Material Choices

-   **3D-Printable Plastics**:
    *   **PLA (Polylactic Acid)**: Easy to print, rigid, biodegradable. Good for prototypes and non-structural parts. Relatively brittle.
    *   **PETG (Polyethylene Terephthalate Glycol)**: More durable and flexible than PLA, better impact resistance. Good for functional parts.
    *   **ABS (Acrylonitrile Butadiene Styrene)**: Strong, impact-resistant, higher temperature resistance. Can be harder to print due to warping.
    *   **Nylon**: Very strong, durable, and flexible. Excellent for gears and high-stress parts. Requires higher printing temperatures.
-   **Aluminum Extrusions**:
    *   **V-Slot/T-Slot**: Modular framing systems (e.g., 2020 profiles) are widely used for robot chassis, offering strength, rigidity, and ease of assembly. Cost-effective and versatile.
-   **Composites**:
    *   **Carbon Fiber**: High strength-to-weight ratio, but significantly more expensive and harder to fabricate. Used in high-performance applications.

## Actuators for Robotic Movement

Actuators provide the force and motion for a robot's joints and effectors. The choice depends heavily on the required torque, speed, precision, and power constraints.

### Servo Motors

*   **Description**: Electric motors with integrated position feedback (potentiometer or encoder) and control circuitry, allowing for precise angular positioning.
*   **Specifications**: Key parameters include torque (Nm or oz-in), speed (RPM or deg/sec), voltage, gear ratio, and control signal type (PWM).
*   **Applications**: Widely used in robotic arms, hands, steering mechanisms, and pan-tilt units due to their ease of use and moderate precision.
*   **Selection Criteria**: Choose servos based on the required torque for the load, desired speed, precision needs, and operating voltage. Hobby servos are cost-effective for smaller robots, while industrial servos offer higher performance and durability.

### Hydraulic Actuators

*   **Description**: Use pressurized fluid (typically oil) to generate force and motion. Known for their high power density.
*   **Specifications**: Force (N or lbs), pressure ratings, flow rate, speed.
*   **Applications**: Best suited for heavy-duty tasks requiring significant force, such as large industrial robots or heavy lifting components in powerful humanoids.
*   **Selection Criteria**: Required for applications demanding extreme force, high speed, or precise force control where electric actuators are insufficient or too bulky.

### Comparison Table: Servo vs. Hydraulic Actuators

| Feature           | Servo Motors                                   | Hydraulic Actuators                             |
| :---------------- | :--------------------------------------------- | :---------------------------------------------- |
| **Actuation Type**| Electric, precise positional control           | Fluid pressure-based, high force                |
| **Force/Torque**  | Moderate to high, depends on size/gear ratio   | Very high                                       |
| **Precision**     | High precision, good for delicate tasks        | Can be precise but more complex to control      |
| **Speed**         | Moderate to high                               | High                                            |
| **Complexity**    | Relatively simple to control                   | Complex hydraulic systems (pump, valves, fluid) |
| **Cost**          | Generally lower for comparable force output    | Higher, especially for complex systems          |
| **Power Req.**    | Lower, electrical                               | Higher, requires hydraulic pump                 |
| **Typical Use**   | Robotic arms, hands, camera pan/tilt         | Heavy machinery, industrial robots, large limbs |
| **Applications**  | Dexterous manipulation, smaller robots         | Heavy lifting, high-force tasks                 |

## Power Systems for Robots

Providing adequate and reliable power is critical for robot operation.

### Battery Technologies

-   **LiPo (Lithium Polymer)**: High energy density, lightweight, good power-to-weight ratio. Requires careful handling and charging due to safety considerations. Common in hobby robotics and drones.
-   **NiMH (Nickel-Metal Hydride)**: Safer than LiPo, more robust, but lower energy density and can suffer from memory effect. A good balance for educational projects.
-   **Lead-Acid**: High capacity, relatively inexpensive, but heavy and bulky. Used in larger robots or stationary applications.

### Power Distribution

-   **Voltage Regulation**: Ensuring consistent voltage levels for different components (microcontrollers, sensors, motors).
-   **Current Capacity**: Providing enough current to meet peak demands, especially during motor startup or high-load operations.
*   **Wiring and Connectors**: Using appropriate gauge wires and reliable connectors to handle current and ensure safety.

## Design Data: CAD Screenshots and Cost Tables

### CAD Screenshots

(Conceptual Textual Representation of CAD Screenshots)

-   **Robot Chassis Frame**: Images showing assembled aluminum extrusion frame with brackets and joints.
-   **Actuator Integration**: Close-ups of servo motor mounting on a robotic arm joint.
-   **End-Effector Design**: Visuals of different gripper types (e.g., two-finger parallel gripper, multi-finger adaptive gripper).
-   **Power System Layout**: Diagram showing battery placement, voltage regulators, and power distribution board.

### Weight and Cost Tables

*   **Example Component Table**:

| Component             | Material        | Weight (kg) | Estimated Cost (USD) | Notes                                    |
| :-------------------- | :-------------- | :---------- | :------------------- | :--------------------------------------- |
| Aluminum Extrusion    | 2020 Profile    | 0.5 / meter | $10 / meter          | For chassis framing                      |
| Servo Motor (Torque)  | Plastic/Metal   | 0.06        | $25                  | Standard hobby servo, 15 kg-cm           |
| 3D Printed Part       | PLA             | 0.1         | $5                   | Custom bracket, requires 3D printer      |
| LiPo Battery (Voltage)| Lithium Polymer | 0.3         | $50                  | 3S (11.1V), 5000mAh                      |
| IMU Sensor            | PCB, Components | 0.01        | $15                  | MPU6050 module                           |

*Note: Costs are estimates and subject to change based on suppliers and quantity.*

## Conclusion

Building a real-world humanoid robot involves careful selection of materials, actuators, and power systems, balancing performance requirements with budget and manufacturability. This chapter provides a foundation for making informed decisions in these critical areas.
