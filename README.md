# Cognitive-Robotics-Project
Cognitive Robotics Project – Color Based Object Sorting in Webots
Technion – Cognitive Robotics (097244), Spring 2024

Authors: Sivan Maspin (207807967), Yuval Rainis (318884921)

Overview - 
This project implements a cognitive robotic system capable of recognizing, collecting, and sorting colored cubes into corresponding bins inside a simulated environment.
The robot — a KUKA youBot in Webots — integrates perception, planning, motion control, and decision making in a cohesive Task and Motion Planning (TAMP) pipeline.

System Architecture - 
Main Components
File	Description:
my_supervisor.py : Oversees the simulation randomizes object placement, manages mission state (HUNT → DELIVER → DONE), and communicates with the robot via a text file.
youbot_cogrob.py : Main robot controller. Handles sensing, planning, motion execution, and high level state transitions.
motion.py : Low level motion controller using Mecanum wheel kinematics, obstacle avoidance, and rotation control.
path_planner.py :	A* grid based planner with obstacle inflation, smoothing, and safety margins.
common.py :	Utility functions for distance and angle computations (if included).
target.txt : Communication file between Supervisor and YouBot controller (shared goal and state).

How It Works
Environment Setu - 
Four colored bins in each room corner (red, green, yellow, blue).
Four matching colored cubes scattered randomly at startup.
The youBot starts at a random pose.

Perception - 
Uses Webots’ camera recognition or pixel based color detection.
Identifies cube colors and updates internal state.

Planning - 
The Supervisor selects the nearest available cube.
The YouBot plans a path using A* while avoiding bins and remaining cubes (inflated obstacles).

Motion Control - 
Velocity based Mecanum control for omnidirectional motion.
Smooth navigation, turn in place logic, and world boundary safety.

Execution - 
The robot "picks up" the cube visually (attachment simulation).
Carries it to the corresponding bin and drops it using an animated motion.
Repeats until all cubes are sorted.

Features -
Task and Motion Planning (TAMP) integration, Randomized environments for each run, Path visualization using Matplotlib (path_plan_X.png)
Collision safe navigation with inflated obstacle zones, Supervisor Robot communication through shared file protocol, Scalable evaluation metrics (completion time, path efficiency, accuracy).

Running the Simulation - 
Prerequisites :
Webots R2023+ installed
Python controller support enabled
matplotlib installed for path visualization : pip install matplotlib

Steps - 
Open the project world in Webots, Set the Supervisor controller to my_supervisor.py.
Set the robot (YouBot) controller to youbot_cogrob.py.
Run the simulation.
Observe console logs, cube sorting behavior, and saved path visualizations.

Demonstration
A short video demonstration of the robot sorting all cubes can be found on YouTube (link to be added).
