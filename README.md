# Inverted Pendulum Self-Balancing Cart Using PID Controller

## Overview

The **Inverted Pendulum Self-Balancing Cart** is a classic control theory problem where a pendulum is mounted on a cart that moves along a track. The objective is to keep the pendulum balanced in an upright position using a control algorithm. This project demonstrates the implementation of a Proportional-Integral-Derivative (PID) controller to achieve stabilization. The project involves using hardware components, mathematical modeling, and MATLAB Simulink for simulation.

## Table of Contents

- [Introduction](#introduction)
- [Hardware Components](#hardware-components)
- [3D Printed Parts](#3d-printed-parts)
- [Mathematical Modeling](#mathematical-modeling)
- [Control Algorithm](#control-algorithm)
- [Simulation](#simulation)
- [Results](#results)
- [Conclusion](#conclusion)
- [Future Work](#future-work)
- [References](#references)

## Introduction

The inverted pendulum is an inherently unstable system that requires a robust control system to balance. The system is significant for studying control algorithms because it exemplifies concepts of dynamics and feedback in control systems. This project utilizes an Arduino-based system with a stepper motor and a PID controller to maintain the pendulum in its upright position.
![WhatsApp Image 2024-04-24 at 16 26 32_a8c7c9b4](https://github.com/user-attachments/assets/fedca9e3-1f3d-4a17-b8dd-164024cf3153)


## Hardware Components

| Component                    | Specification                 | Quantity |
|------------------------------|--------------------------------|----------|
| Arduino UNO board            | -                              | 1        |
| Nema17 stepper motor         | 7.5kg/cm Torque               | 1        |
| A4988 stepper motor driver   | -                              | 1        |
| MPU-6050 Six-Axis IMU Sensor | -                              | 1        |
| 12V 10A power supply         | -                              | 1        |
| Bearings                     | 16 x 4mm                       | 16       |
| GT2 pulleys and belt         | -                              | 2        |
| PVC Pipe                     | 1.5-inch diameter, 2.4 meters  | 1        |
| Miscellaneous                | Nuts, bolts, wires, etc.       | -        |

## 3D Printed Parts

| Part Name                        | Quantity |
|----------------------------------|----------|
| Gantry Roller                    | 2        |
| End Caps                         | 4        |
| Stepper Bracket                  | 1        |
| Idle Pulley Bearing Holder       | 2        |
| Pendulum Holder                  | 1        |
| Belt Attachment                  | 2        |
| Pendulum Bearing Holder          | 2        |
| Pulley Hole Spacer               | 1        |
| Bearing Hole Spacer              | 4        |
| Gantry Plate                     | 1        |
| Stepper Holder Plate             | 1        |
| Idle Pulley Holder Plate         | 1        |
| Pendulum                         | 2        |

## Mathematical Modeling

The system's mathematical model is derived using the equations of motion for the inverted pendulum system. The kinematics and dynamics are modeled considering the forces acting on the cart and the pendulum. The PID controller is then designed to minimize the error between the desired and actual angles of the pendulum. The system's stability is determined using the eigenvalues of the state matrix.

## Control Algorithm

### PID Controller

The PID controller uses three terms:
- **Proportional (P):** Reacts to the current error.
- **Integral (I):** Reacts to the accumulation of past errors.
- **Derivative (D):** Predicts future errors based on the rate of change.

This combination provides a robust solution to balance the inverted pendulum and maintain stability. The controller was tuned to optimize performance, reducing oscillations and improving response time.

## Simulation

MATLAB Simulink was used to model and simulate the inverted pendulum system with a PID controller. The simulation includes a 3D VRML model to visualize the pendulum's movement and stability over time.
![image](https://github.com/user-attachments/assets/c14617e8-2ed5-4b2c-bacc-58c3a80ace85)


## Results

The simulation demonstrated that the PID controller effectively balances the inverted pendulum on the cart. The system successfully countered disturbances and maintained stability, with the IMU sensor readings remaining near the desired setpoint throughout the experiment. The performance was superior compared to a simple proportional control system.
![image](https://github.com/user-attachments/assets/468659ee-e748-4658-b61b-43207922a3ad)

## Conclusion

The Inverted Pendulum Self-Balancing Cart project successfully demonstrates the effectiveness of the PID control algorithm in stabilizing an unstable system. It provides insights into control system design and the practical challenges of implementing real-world applications in robotics and automation.

## Future Work

Future enhancements could involve:
- Implementing more advanced control algorithms like Linear Quadratic Regulator (LQR) or Model Predictive Control (MPC).
- Adding encoders for more precise feedback on both linear and angular positions.
- Exploring applications in transportation systems, such as Segways.

## References

1. Darshan Chotaliya, Chaitanya Yenge, Vedang Banpurkar. "RCS KDR Project Report" [PDF].
2. Roberge, J.K., "The Mechanical Seal," M.I.T. Bachelor's Thesis, 1960.
3. Higdon, Cannon, "Stabilization of Inverted Pendulum Systems," American Journal of Physics, 1963.

---

Feel free to modify or add more details based on specific content or additional information you'd like to highlight.
