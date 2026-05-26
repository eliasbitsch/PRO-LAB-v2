# PRO Lab: Probabilistic Robotics with ROS2 and TurtleBot4

In this project, you have to implement and evaluate three fundamental probabilistic state estimation methods in mobile robotics:

- Kalman Filter (KF)
- Extended Kalman Filter (EKF)
- Particle Filter (PF)

All methods are implemented as ROS2 nodes and applied to a TurtleBot4 simulation to estimate the robot state.

## Learning Objectives

To complete the assignment, you have to:

- Implement KF, EKF and PF in a robotics context
- Understand the role of motion and measurement models
- Analyze the impact of uncertainty and noise
- Evaluate estimation methods using quantitative metrics
- Understand limitations of different filtering approaches
- Design and analyze experiments in robotic systems
- Solve the specific task you find in the given table

## Core Tasks

- **Implement Filters:** three separate ROS2 nodes
- Subscribe to your sensors
- Publish an estimated pose

### Use Common System Setup

- Same input data for all filters
- Same coordinate frame
- Same test trajectories
- Same evaluation conditions

### Perform Mandatory Experiments

- **Process Noise (Q) Variation**
  - Test different Q values
  - Analyze model confidence
- **Measurement Noise (R) Variation**
  - Test different R values
  - Analyze sensor trust
- **Runtime / Performance**
  - Compare computational cost
  - Discuss real-time capability
- **Ground Truth Evaluation**
  - Compute error metrics (e.g. RMSE)
  - Compare trajectories
- **Landmark detection**
  - Define a landmark by yourself

### Evaluation

- Compare KF, EKF and PF
- Visualize results (plots and RViz2)
- Discuss strengths and weaknesses

## Grading

| Component       | Weight |
| --------------- | ------ |
| Code Submission | 40%    |
| Presentation    | 20%    |
| Documentation   | 40%    |

## Submission

- One GitHub repo (with a `README.md`)
- PowerPoint presentation (on the last two sessions you have to show your results)
- Documentation (Paper Style)

## Task Assignments

2510331021 | **Wrong Initialization**                                                  | Initialize the filter with an incorrect starting pose and/or uncertainty.         |

## Ideas / Notes

- **PRO Terminal GUI**: Build a terminal launcher / status banner for PRO similar to FastMCP's startup screen
  (boxed ASCII banner showing project name, version, server/node status, useful URLs).
  Example reference (FastMCP 3.2.4):

  ```
  ╭──────────────────────────────────────────────────────────────────────────────╮
  │                         ▄▀▀ ▄▀█ █▀▀ ▀█▀ █▀▄▀█ █▀▀ █▀█                        │
  │                         █▀  █▀█ ▄▄█  █  █ ▀ █ █▄▄ █▀▀                        │
  │                                FastMCP 3.2.4                                 │
  │                            https://gofastmcp.com                             │
  │                 🖥  Server:      mcp-for-unity-server, 3.2.4                  │
  ╰──────────────────────────────────────────────────────────────────────────────╯
  ```

  For PRO this could show: active filter node (KF/EKF/PF), ROS2 domain ID, TurtleBot4 connection state,
  RViz/topic links, and current experiment config.

