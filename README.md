# Cognitive Robotics Project – Color Based Object Sorting

Course: Technion – Cognitive Robotics (097244), Spring 2025  
Authors: Sivan Maspin (207807967), Yuval Rainis (318884921)

---

## Overview
This project implements a cognitive robotic system capable of recognizing, collecting and sorting colored cubes into their corresponding bins within a simulated environment.
The system integrates perception, task planning, motion planning and control into a cohesive Task and Motion Planning (TAMP) pipeline.  
The simulation is implemented in Webots, using the KUKA youBot platform.

---

## System Architecture

| Component | File | Description |
|------------|------|-------------|
| **Supervisor** | `my_supervisor.py` | Oversees the simulation, randomizes object placement, manages mission states (HUNT → DELIVER → DONE) and communicates with the robot via a shared text file. |
| **Robot Controller** | `youbot_cogrob.py` | Handles perception, planning, motion execution and high level control. |
| **Motion Controller** | `motion.py` | Low level velocity and obstacle control using Mecanum wheel kinematics. |
| **Path Planner** | `path_planner.py` | A* grid based planner with obstacle inflation, smoothing and fallback logic. |
| **Utilities** | `common.py` | Helper functions for distance and angle computations. |
| **Communication File** | `target.txt` | Shared file for synchronization between the Supervisor and Robot controller. |

---

## How It Works

### **Environment Setup**
- A 4×4 m checkered floor with four colored bins (red, green, yellow, blue) at the corners.  
- Four colored cubes (one per bin color) are randomly placed at startup.  
- The KUKA youBot starts at a random position.

### **Perception**
- Uses Webots’ recognition camera and color classification to detect cube colors.

### **Planning**
- The Supervisor selects the next cube to collect.  
- The robot plans its path using A\*, avoiding both bins (static obstacles) and remaining cubes (dynamic obstacles).

### **Motion Control**
- Velocity based control using Mecanum wheels.  
- Includes turning in place logic and slowdown near targets for smooth navigation.

### **Execution**
- The robot picks up cubes visually, carries them to the correct bin and releases them.  
- The process repeats until all cubes are sorted.

---

## Features and Evaluation
- Full Task and Motion Planning (TAMP) integration.  
- Randomized environments for each run.  
- Path visualization using Matplotlib (`path_plan_X.png`).  
- Collision free navigation via inflated obstacle zones.  
- Supervisor Robot communication through a shared file protocol.  
- Real time metric logging to `performance_log.csv` (task time, path length, sorting accuracy).

---

## Running the Simulation

### **Prerequisites**
- **Webots R2023+** installed  
- Python controller support enabled  
- `matplotlib` installed for path visualization:
  ```bash
  pip install matplotlib
  ```

### **Steps to Run**
1. Open the project world file (`color_sorting.wbt`) in Webots.  
2. Ensure the following controller assignments:  
   - Supervisor: `my_supervisor.py`  
   - Robot (YouBot): `youbot_cogrob.py`  
3. Run the simulation.  
4. Observe the console output, cube sorting behavior, and saved path images.  
5. Results are logged automatically to `performance_log.csv`.  

---

## Demonstration
A short video demonstration of the robot sorting all cubes can be found on YouTube:  
*(link to be added once uploaded)*

---

## Authors and Contributions
- Sivan Maspin – Supervisor, randomization logic, mission state machine, visualization.  
- Yuval Rainis – YouBot controller, path planner, motion controller, perception pipeline.  
- Both collaborated closely on environment design, debugging and parameter tuning to ensure stable system performance.

---

## Summary
This project demonstrates a complete cognitive robotics pipeline, integrating perception, reasoning, and motion control under uncertainty.  It successfully implements Task and Motion Planning (TAMP) principles in a fully autonomous robotic system.
