
# Water-Cleaning Robot – Turtlesim Simulation

## Introduction

This document outlines the simulation of an autonomous floating robot built to detect and remove debris from water surfaces. The virtual environment is developed using the `turtlesim` package within the Robot Operating System (ROS) and mimics the expected behavior of a future physical prototype.

The robot operates in a 2D space and identifies virtual waste based on color. Upon detection, it navigates toward the object and simulates a cleaning task by marking the area. This logic imitates a real-world implementation where a camera module and color filtering (with OpenCV) enable object recognition, and an Arduino microcontroller handles motor control.

This project also aligns with the United Nations Sustainable Development Goals:

- **Goal 6: Clean Water and Sanitation** – Encouraging proactive waste removal from aquatic environments.
- **Goal 14: Life Below Water** – Supporting pollution reduction efforts in marine ecosystems.

## System Overview

- The robot is implemented using the `turtlesim` package in ROS and programmed in Python.
- A single virtual object (waste) is generated at a random position and assigned a fixed color.
- The robot scans a 120° field of view to detect the object.
- Upon detection, the robot moves toward the waste and simulates a cleaning operation.
- After completion, the robot is reset, and the process can be repeated with a newly generated object.

## Theoretical Framework

### 1. Visual Object Recognition

The simulation represents objects using distinct color codes, allowing the robot to identify them within a certain vision cone. This approach is modeled after basic image segmentation methods used in real robotics.

### 2. ROS Architecture

ROS is a modular framework that simplifies communication and data handling between software components. In this simulation, services are used to spawn and remove turtles, while publishers and subscribers manage real-time motion and pose data.

### 3. Turtlesim

Turtlesim provides a simple 2D simulation space for testing ROS-based robots. Although basic, it supports key ROS principles, making it ideal for early-stage development of robotic systems.

### 4. Proportional Control

To minimize energy usage and improve accuracy, the robot implements a proportional control loop. It calculates the required angular adjustment to align with the object and advances only when the direction is sufficiently corrected.

### 5. Energy Consumption Metric

Rather than speed or timing, this simulation uses **energy consumption** as its primary performance metric. This is computed using the formula:

\[
\text{Energy} = \int (F \cdot v) \, dt
\]

Assuming constant force and approximating based on duration and velocity, energy consumption can be expressed as proportional to:

\[
E = k \cdot (v^2 + \omega^2) \cdot t
\]

Where:

- \( v \) = linear velocity  
- \( \omega \) = angular velocity  
- \( t \) = total movement time  
- \( k \) = arbitrary proportional constant  

This allows us to assess how efficiently the robot completes its task.

## Code Implementation and Explanation

Below is the main structure of the simulation using ROS and Python:

```python
# (Código Python incluido aquí — omitido por brevedad)
```

## Performance Metric Evaluation

In this version, performance is measured by approximated energy usage during the cleaning action. Instead of tracking how long it takes to reach the object, we consider how much energy is spent by the robot during movement.

The energy is calculated using the squared sum of linear and angular velocities, accumulated over each time step, and scaled to account for total time. This gives an abstract but useful estimation of energy efficiency.

### *1st Attempt*

- Energy consumption: **16.21 units**

### *2nd Attempt*

- Energy consumption: **13.94 units**

### *3rd Attempt*

- Energy consumption: **17.48 units**

**Average Energy Consumption: 15.88 units**

## Justification

To better understand these values, consider the factors influencing energy consumption:

- **Distance to target**: Longer paths naturally consume more energy.
- **Required rotation**: Sharp turns or misalignment increase angular velocity, contributing significantly to energy usage.
- **Control policy**: The robot uses a proportional controller that stops forward motion unless alignment is achieved, avoiding unnecessary trajectory errors but increasing rotation time.

These trade-offs explain fluctuations between attempts. For example, in the third attempt, the target required significant initial rotation, increasing angular energy consumption. In contrast, in the second attempt, the target was nearly aligned with the robot's initial heading, leading to lower overall energy usage.

## Conclusions

This simulation helped reinforce key robotic principles through the lens of energy optimization. Using `turtlesim` and ROS, we programmed a robot capable of locating and approaching visual targets with minimal energy consumption.

The energy-based metric provided a valuable alternative to speed-focused evaluations, offering a more sustainability-aware perspective. It highlighted the importance of path efficiency and control precision in autonomous robots.

If further expanded, this project could include:

- Real-time power sensors (in a physical prototype),
- Obstacle avoidance,
- Or dynamic re-planning strategies to further minimize energy expenditure.

## References

- Arduino. (2024). Arduino Language Reference. https://www.arduino.cc/reference/en/  
- OpenCV Team. (2023). Color Spaces and Object Detection. https://docs.opencv.org/4.x/  
- Open Robotics. (2023). ROS 2 Documentation. https://docs.ros.org/en/humble/index.html  
- Python Software Foundation. (2023). Python 3 Standard Library: time. https://docs.python.org/3/library/time.html  
- United Nations. (2015). Sustainable Development Goals (SDGs). https://sdgs.un.org/goals  
- Willow Garage. (2010). Turtlesim Tutorials. http://wiki.ros.org/turtlesim  
