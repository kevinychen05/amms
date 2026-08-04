# Autonomous Mobile Manipulation System (AMMS)

An end-to-end ROS 2 robotic system, in simulation, that autonomously maps an environment, localizes itself, navigates to a specified object, detects and estimates the object's pose, plans a collision-free grasp, manipulates the object, transports it to a goal location, and reports mission state through visualization.

Full scope and phase-by-phase plan: [`Autonomous_Mobile_Manipulation_Project_Proposal.pdf`](./Autonomous_Mobile_Manipulation_Project_Proposal.pdf).

## Stack

| Area | Choice |
|---|---|
| Middleware | ROS 2 Jazzy Jalisco (Ubuntu 24.04) |
| Simulation | Gazebo Harmonic via `ros_gz` |
| Visualization | RViz2 |
| Navigation | Nav2, slam_toolbox, AMCL |
| Perception | OpenCV, YOLO, `message_filters` |
| Manipulation | MoveIt 2 |

ROS 2 Jazzy was chosen over Humble because it pairs natively with Gazebo Harmonic (this project's simulator) and has a longer LTS window.

## Repository structure

| Directory | Contents |
|---|---|
| `/robot_description` | URDF/Xacro, meshes, robot configuration files |
| `/simulation` | Gazebo worlds, simulation plugins, spawn/launch configs |
| `/navigation` | Nav2 parameter files, SLAM configuration, map files |
| `/perception` | Camera drivers, OpenCV utilities, detector node(s) |
| `/manipulation` | MoveIt 2 configuration, grasp planner |
| `/mission_manager` | Behavior tree / state machine implementation |
| `/interfaces` | Custom ROS 2 messages, services, and actions |
| `/docs` | Architecture diagrams, design notes, technical report |
| `/tests` | Integration scenarios, fault-injection test suite |

## Branching strategy

- `main` is treated as protected by convention: no direct pushes, all changes land via pull request. (GitHub's server-side branch protection requires a public repo or paid plan for private repos, so this is currently self-enforced rather than platform-enforced.)
- One feature branch per unit of work, named `phase-N/short-description` (e.g. `phase-0/architecture`).
- PRs are used even as a solo developer, to build the habit and keep history reviewable.

## Development environment

ROS 2, Gazebo, and MoveIt 2 don't run natively on macOS, so development happens inside a Ubuntu 24.04 VM (UTM) with this repository mounted read-write via a shared folder — code is authored on the host, built and run inside the VM.
