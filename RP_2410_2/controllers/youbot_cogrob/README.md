# KUKA youBot Cognitive Robotics System

A complete autonomous cube sorting system for the KUKA youBot robot in Webots simulation environment.

## Overview

This project implements a cognitive robotics system that enables the KUKA youBot to autonomously:
- Navigate and explore the environment
- Detect colored cubes using computer vision
- Plan optimal paths using A* algorithm
- Pick up cubes with the 5-DOF manipulator
- Sort cubes into corresponding colored bins
- Track performance metrics and completion status

## System Architecture

### Core Modules

1. **main.py** - Main controller with finite state machine
   - Coordinates all subsystems
   - Implements FSM: EXPLORE → PLAN_TO_OBJECT → NAV_TO_OBJECT → PICK → PLAN_TO_BIN → NAV_TO_BIN → PLACE
   - Handles pose estimation using GPS and IMU
   - Tracks performance metrics

2. **perception.py** - Computer vision and object detection
   - HSV-based color segmentation for red, green, blue, yellow cubes
   - Pixel-to-world coordinate transformation
   - Camera calibration and image processing
   - Object confidence scoring

3. **arm.py** - 5-DOF manipulator control
   - Predefined poses for pick and place operations
   - Gripper control with position feedback
   - Complete pick/place sequence automation
   - Joint position monitoring

4. **motion.py** - Mecanum wheel base control
   - Inverse kinematics for omnidirectional movement
   - Waypoint navigation with proportional control
   - Pure pursuit path following option
   - Velocity limiting and safety features

5. **mapper.py** - Occupancy grid mapping and path planning
   - Real-time occupancy grid construction
   - A* pathfinding with obstacle inflation
   - Dynamic obstacle detection and avoidance
   - Map visualization and export

6. **task_planner.py** - High-level decision making
   - Intelligent object selection based on distance and confidence
   - Bin assignment and sorting strategy
   - Task history tracking and learning
   - Performance optimization

7. **common.py** - Shared utility functions
   - Mathematical helpers (angle wrapping, distance calculation)
   - Coordinate transformations
   - Validation functions

## Hardware Requirements

### Robot Configuration
- KUKA youBot with mecanum wheels
- 5-DOF manipulator arm
- 2-finger parallel gripper
- Forward/downward facing camera
- GPS sensor for localization
- IMU for orientation

### World Setup
- 4 colored cubes: red, green, blue, yellow
- 4 corresponding bins at room corners
- Adequate lighting for color detection
- Obstacle-free navigation space

## Installation and Setup

### Prerequisites
- Webots R2023b or later
- Python 3.8+
- Required packages: numpy, opencv-python

### File Structure
```
controllers/youbot_cogrob/
├── main.py              # Main controller
├── arm.py               # Arm control
├── motion.py            # Base movement
├── perception.py        # Vision system
├── mapper.py            # Mapping and planning
├── task_planner.py      # Decision making
├── common.py            # Utilities
└── README.md            # This file
```

### Configuration
1. Adjust bin positions in `main.py` BIN_POSITIONS dictionary
2. Tune camera parameters in `perception.py` 
3. Calibrate arm poses in `arm.py` if needed
4. Modify HSV color ranges for your lighting conditions

## Usage

### Running the System
1. Open Webots and load your world file
2. Set the controller to `youbot_cogrob`
3. Run the simulation
4. The robot will automatically start the sorting mission

### Expected Behavior
1. **Exploration Phase**: Robot moves around to discover cubes
2. **Detection Phase**: Camera identifies and localizes colored objects
3. **Planning Phase**: System selects optimal cube and plans path
4. **Navigation Phase**: Robot moves to target cube location
5. **Manipulation Phase**: Arm picks up the cube
6. **Transport Phase**: Robot navigates to corresponding bin
7. **Placement Phase**: Arm places cube in bin
8. **Repeat**: Process continues until all cubes are sorted

### Performance Metrics
The system tracks and reports:
- Total mission time
- Sorting accuracy (cubes sorted / total cubes)
- Total distance traveled
- Success rate of pick/place operations
- Results saved to `youbot_metrics.csv`

## Tuning Parameters

### Vision System
```python
# In perception.py
HSV_RANGES = {
    "red1":   ((0, 120, 70), (10, 255, 255)),
    "green":  ((35, 60, 60), (85, 255, 255)),
    # Adjust ranges for your lighting
}
```

### Navigation Control
```python
# In motion.py
self.kp_linear = 1.5      # Linear velocity gain
self.kp_angular = 3.0     # Angular velocity gain
self.goal_tolerance = 0.05 # Distance tolerance [m]
```

### Arm Poses
```python
# In arm.py - Adjust joint angles for your setup
ARM_POSES = {
    "home": [0.0, 1.0, -1.57, 0.0, 0.0],
    "grasp": [0.0, 1.1, -1.6, 0.5, 0.0],
    # Tune for cube height and gripper alignment
}
```

### Task Planning
```python
# In task_planner.py
self.confidence_threshold = 0.3  # Minimum detection confidence
self.distance_weight = 1.0       # Distance importance
self.confidence_weight = 0.5     # Confidence importance
```

## Troubleshooting

### Common Issues

**Robot doesn't move**
- Check wheel motor initialization
- Verify velocity limits aren't too restrictive
- Ensure GPS/IMU sensors are enabled

**Poor object detection**
- Adjust HSV color ranges for your lighting
- Check camera mounting and field of view
- Increase minimum detection area threshold

**Arm doesn't reach cubes**
- Calibrate arm poses for cube height
- Adjust gripper opening/closing positions
- Check joint velocity limits

**Navigation issues**
- Tune PID controller gains
- Verify coordinate system alignment
- Check obstacle inflation parameters

### Debug Features
- Set debug flags in modules for verbose output
- Use `perception.visualize_detections()` for vision debugging
- Call `mapper.save_map()` to export occupancy grid
- Check `youbot_metrics.csv` for performance analysis

## Performance Optimization

### Speed Improvements
- Increase navigation gains for faster movement
- Reduce arm movement delays
- Optimize exploration waypoints

### Accuracy Improvements
- Lower confidence threshold for more detections
- Increase camera resolution if supported
- Add sensor fusion for better localization

### Robustness Improvements
- Add retry logic for failed operations
- Implement collision detection
- Add emergency stop capabilities

## Extension Ideas

1. **Multi-robot coordination** - Multiple youBots working together
2. **Dynamic obstacles** - Moving obstacle avoidance
3. **Learning system** - Improve performance over time
4. **Voice commands** - Human-robot interaction
5. **3D perception** - Depth camera integration
6. **Advanced planning** - RRT* or other algorithms

## License

This project is provided as educational material for robotics and AI research.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review parameter tuning guidelines
3. Examine debug output and metrics
4. Test individual modules in isolation

---

**Author**: AI Cognitive Robotics System  
**Version**: 1.0  
**Last Updated**: 2024
