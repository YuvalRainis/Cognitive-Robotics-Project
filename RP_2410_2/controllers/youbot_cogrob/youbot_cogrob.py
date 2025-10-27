from controller import Robot
import time
import os
from motion import MotionController
from common import distance_2d
from path_planner import astar, is_free, inflate_obstacles
import math
import matplotlib.pyplot as plt  
import csv
from datetime import datetime
from path_planner import X_MIN, X_MAX, Y_MIN, Y_MAX # to get world boundaries
import glob
from PIL import Image

TIME_STEP = 32
# Bin positions 
BIN_POSITIONS = {
    "yellow": (-1.87, -1.87),
    "red": (-1.87, 1.87),
    "green": (1.87, 1.87),
    "blue": (1.87, -1.87)
}

# NEW PLOTTING FUNCTION 
def plot_path(start, goal, path, obstacles, filename="path_plan.png"):
    """
    Generates and saves a Matplotlib plot of the planned path and obstacles.
    """
    print("Plotting path")
    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot obstacles - bins and other cubes
    if obstacles:
        # Unzip the list of (x, y) tuples into two lists
        obs_x, obs_y = zip(*obstacles)
        # Plot as gray circles
        ax.plot(obs_x, obs_y, 'o', color='gray', markersize=10, label='Obstacles')

    # Plot the planned path
    if path:
        path_x, path_y = zip(*path)
        # Plot path as a blue line with markers
        ax.plot(path_x, path_y, 'b-o', linewidth=2, markersize=5, label='Planned Path')

    # Plot Start and Goal points to make them stand out
    ax.plot(start[0], start[1], 'g*', markersize=20, label='Start')
    ax.plot(goal[0], goal[1], 'rX', markersize=20, label='Goal')

    # Format the plot
    ax.set_title("A* Path Visualization")
    ax.set_xlabel("World X coordinate (m)")
    ax.set_ylabel("World Y coordinate (m)")
    ax.set_xlim(X_MIN - 0.2, X_MAX + 0.2)
    ax.set_ylim(Y_MIN - 0.2, Y_MAX + 0.2)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.legend()

    # Save the plot to a file and close it to free memory
    plt.savefig(filename)
    plt.close(fig)
    print(f"Path visualization saved to {filename}")

# Main cognitive controller: state machine + sensing + planning 
class CognitiveRobotController:
    def __init__(self):
        # Webots devices and timing
        self.robot = Robot()
        self.process_start_time = time.time()
        self.time_step = TIME_STEP
        self.motion_controller = MotionController(self.robot)
        # Sensors
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.time_step)
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        
        # Camera
        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(self.time_step)
            # Enable recognition if available
            if self.camera.hasRecognition():
                self.camera.recognitionEnable(self.time_step)
                print("Camera with recognition enabled")
            else:
                print("Camera enabled (no recognition)")
        else:
            print("No camera found")

        # State variables
        self.state = "WAIT_FOR_TARGET"    # States are WAIT_FOR_TARGET, FOLLOW_PATH, NAV_TO_OBJECT, NAV_TO_BIN, MISSION_COMPLETE, DONE
        self.current_target = None        # Parsed command from supervisor color + position/obstacles
        self.carrying_color = None        # Color of cube currently carried after CARRY command
        self.path = None                  # Current A* waypoint list
        self.path_index = 0               # Index of next waypoint to follow

        # Vision system
        self.detected_objects = []  # Store detected cubes
        # Metrics
        self.start_time = time.time()
        self.total_distance = 0.0
        self.last_position = None
        self.plan_count = 0  # <--- ADD THIS LINE
        print("YouBot cognitive controller initialized.")

    # sensing & metrics
    def get_robot_pose(self):
        """Return (x, y, yaw) in world frame using GPS."""
        pos = self.gps.getValues()
        x, y = pos[0], pos[1]
        theta = self.imu.getRollPitchYaw()[2]
        return (x, y, theta)
    
    def update_metrics(self, pose):
        """Integrate traveled distance along the trajectory for performance logging."""
        if self.last_position is not None:
            self.total_distance += distance_2d(pose[:2], self.last_position)
        self.last_position = pose[:2]

    # vision system 
    def detect_cubes_in_camera(self):
        """
        Detect colored cubes using camera recognition.
        Returns list of detected cubes with colors.
        """
        if not self.camera:
            return []
        
        try:
            # Use recognition API if available
            if self.camera.hasRecognition():
                objects = self.camera.getRecognitionObjects()
                detected = []
                
                for obj in objects:
                    # Get object model name (e.g., "RED_BOX", "BLUE_BOX")
                    model = obj.getModel()
                    
                    # Extract color from model name
                    if "RED" in model.upper():
                        detected.append(("red", 0.9))
                        print(f"Camera detected: RED cube")
                    elif "GREEN" in model.upper():
                        detected.append(("green", 0.9))
                        print(f"Camera detected: GREEN cube")
                    elif "BLUE" in model.upper():
                        detected.append(("blue", 0.9))
                        print(f"Camera detected: BLUE cube")
                    elif "YELLOW" in model.upper():
                        detected.append(("yellow", 0.9))
                        print(f"Camera detected: YELLOW cube")
                
                return detected
        
            # Fallback - Simple pixel based detection if recognition not available
            image = self.camera.getImage()
            if not image:
                return []
            
            width = self.camera.getWidth()
            height = self.camera.getHeight()
            
            # Sample center of image
            detected = []
            color_counts = {'red': 0, 'green': 0, 'blue': 0, 'yellow': 0}
            
            center_x = width // 2
            center_y = height // 2
            sample_size = 30
            
            for dy in range(-sample_size, sample_size, 5):
                for dx in range(-sample_size, sample_size, 5):
                    x = center_x + dx
                    y = center_y + dy
                    if 0 <= x < width and 0 <= y < height:
                        r = self.camera.imageGetRed(image, width, x, y)
                        g = self.camera.imageGetGreen(image, width, x, y)
                        b = self.camera.imageGetBlue(image, width, x, y)
                        
                        # Simple color classification
                        if r > 150 and g < 100 and b < 100:
                            color_counts['red'] += 1
                        elif g > 150 and r < 100 and b < 100:
                            color_counts['green'] += 1
                        elif b > 150 and r < 100 and g < 100:
                            color_counts['blue'] += 1
                        elif r > 150 and g > 150 and b < 100:
                            color_counts['yellow'] += 1
            
            max_count = max(color_counts.values())
            if max_count > 15:
                for color, count in color_counts.items():
                    if count == max_count:
                        confidence = min(count / 50.0, 1.0)
                        detected.append((color, confidence))
                        print(f"ðŸ‘ï¸ Camera sees: {color} (confidence: {confidence:.2f})")
                        break
            
            return detected
            
        except Exception as e:
            return []
        
    # I/O with supervisor 
    def read_target_from_file(self):
        """Read the latest target or carry command from target.txt."""
        try:
            path = os.path.join(os.path.dirname(__file__), "target.txt")
            if not os.path.exists(path):
                return None
            try:
                mtime = os.path.getmtime(path)
                if mtime < getattr(self, "process_start_time", 0):
                    return None
            except Exception:
                pass
            with open(path, "r") as f:
                line = f.readline().strip()
                if not line:
                    return None
                if line == "STOP":
                    return "STOP"
                if line.startswith("CARRY"):
                    parts = line.split()
                    if len(parts) >= 2:
                        color = parts[1].lower()

                        # Parse obstacles from the CARRY line as well
                        obstacles = []
                        for i, part in enumerate(parts):
                            if part.endswith("_obs"):
                                if i + 2 < len(parts):
                                    try:
                                        obs_x = float(parts[i + 1])
                                        obs_y = float(parts[i + 2])
                                        obstacles.append((obs_x, obs_y))
                                    except (ValueError, IndexError):
                                        print(f"Warning: Could not parse obstacle coordinates after '{part}'")

                        return {"type": "CARRY", "color": color, "obstacles": obstacles}
                
                # Parse target and obstacle positions
                parts = line.replace(",", " ").split()
                if len(parts) < 4:
                    return None
                
                color = parts[0].lower()
                x, y, z = map(float, parts[1:4])
                
                obstacles = []
                # We explicitly look for '_obs' tags in the parts list
                for i, part in enumerate(parts):
                    if part.endswith("_obs"):
                        # Check if there are enough elements after the tag for coordinates
                        if i + 2 < len(parts):
                            try:
                                obs_x = float(parts[i + 1])
                                obs_y = float(parts[i + 2])
                                obstacles.append((obs_x, obs_y))
                            except (ValueError, IndexError):
                                # This handles cases where the number isn't valid preventing a crash
                                print(f"Warning: Could not parse obstacle coordinates after '{part}'")

                return {
                    "type": "TARGET",
                    "color": color,
                    "position": (x, y),
                    "obstacles": obstacles
                }
        except Exception as e:
            print(f"Error reading target file: {e}")
            return None
        
    # safety / escape helpers 
    def perform_escape_maneuver(self, pose):
        """
        If robot is too close to wall - move toward center.
        More aggressive escape for corners
        """
        x, y, theta = pose
        
        #  Stricter threshold start escaping earlier
        if abs(x) > 1.55 or abs(y) > 1.55:
            print(f"Too close to edge at ({x:.2f},{y:.2f}), escaping...")
            
            # Direction toward center
            escape_dir = math.atan2(-y, -x)
            
            # Longer escape - move 50cm toward center
            for _ in range(50):  # Increased from 30
                vx = 0.40 * math.cos(escape_dir)  
                vy = 0.40 * math.sin(escape_dir)
                self.motion_controller.set_base_velocity(vx, vy, 0.0)
                self.robot.step(self.time_step)
            
            self.motion_controller.stop()
            
            # Verify we actually moved inward
            new_pose = self.get_robot_pose()
            new_x, new_y = new_pose[0], new_pose[1]
            print(f"Escaped: ({x:.2f},{y:.2f}) â†’ ({new_x:.2f},{new_y:.2f})")
            return True
        
        return False
    
    # planning helpers 
    def plan_path(self, start, goal, additional_obstacles=None):
        """
        Compute a path using A* avoiding bins AND cubes as obstacles.
        Uses inflated cube obstacles for physical safety margin.
        """
        # Start with bin positions as static obstacles
        obstacles = list(BIN_POSITIONS.values())

        # Inflate dynamic obstacles (cubes) to create larger safety buffers
        if additional_obstacles:
            print(f"Raw cube obstacles: {len(additional_obstacles)}")
            inflated = inflate_obstacles(additional_obstacles, inflate_radius=0.1, step=0.05)
            obstacles.extend(inflated)
            print(f"Added inflated obstacles: {len(inflated)}")
        
        # Clamp goal away from borders to avoid unreachable targets
        gx, gy = goal
        clamp_margin = 0.35
        gx = max(-1.8 + clamp_margin, min(1.8 - clamp_margin, gx))
        gy = max(-1.8 + clamp_margin, min(1.8 - clamp_margin, gy))
        goal = (gx, gy)

        # If goal is very far outward, consider a via-point at (0,0) for safer routing
        dist_to_center = math.hypot(gx, gy)
        if dist_to_center > 2.5:
            print("Goal near edge, routing via center (0,0)")
            center = (0.0, 0.0)
            path1 = astar(start, center, obstacles)
            path2 = astar(center, goal, obstacles)
            if path1 and path2:
                print("Two part path planned (via center).")
                return path1[:-1] + path2
        path = astar(start, goal, obstacles)

        if path:
            print(f"Planned path with {len(path)} waypoints total.")

            # Increment the plan counter and generate a unique filename
            self.plan_count += 1
            filename = f"path_plan_{self.plan_count}.png"

            # Call our new plotting function with all the necessary data
            plot_path(start, goal, path, obstacles, filename)

            return path
        else:
            print("No valid path found — fallback engaged.")
            return None
        
    def get_safe_bin_position(self, bin_pos):
        """Calculate a safe approach position offset from the bin corner."""
        offset = 0.50
        safe_x = bin_pos[0] + (offset if bin_pos[0] < 0 else -offset)
        safe_y = bin_pos[1] + (offset if bin_pos[1] < 0 else -offset)
        return (safe_x, safe_y)
    
    # main loop 
    def run(self):
        print("Starting robot control loop...")
        path_file = os.path.join(os.path.dirname(__file__), "target.txt")
        print("Waiting for target.txt to appear...")

        # Block until supervisor creates target.txt
        while not os.path.exists(path_file):
            time.sleep(0.5)
        print("target.txt found! Starting mission...")
        step_count = 0
        # State machine loop
        while self.robot.step(self.time_step) != -1:
            pose = self.get_robot_pose()
            self.update_metrics(pose)
            step_count += 1
            if step_count % 50 == 0:
                print(f"Step {step_count}, state={self.state}, pose=({pose[0]:.2f},{pose[1]:.2f})")

            # WAIT_FOR_TARGET: read and plan toward the next cube 
            if self.state == "WAIT_FOR_TARGET":
                x, y = pose[0], pose[1]
                # Preemptive escape if we arrive too close to borders
                if abs(x) > 1.7 or abs(y) > 1.7:
                    print(f"Near edge at ({x:.2f},{y:.2f}), escaping...")
                    self.perform_escape_maneuver(pose)
                    continue
                target = self.read_target_from_file()

                # STOP means supervisor reports completion
                if target == "STOP":
                    print("Received STOP signal - all cubes collected!")
                    self.motion_controller.stop()
                    self.state = "MISSION_COMPLETE"
                    continue
                elif target and isinstance(target, dict) and target.get("type") == "CARRY":
                    self.carrying_color = target["color"]
                    print(f"Received CARRY {self.carrying_color} — switching directly to NAV_TO_BIN.")
                    self.path = None
                    self.path_index = 0
                    self.state = "NAV_TO_BIN"
                    continue
                
                # New TARGET: plan a path to that cube, including obstacles
                elif target and target["type"] == "TARGET":
                    self.current_target = target
                    print(f"New target: {target['color']} at {target['position']}")
                    print(f"Current: ({pose[0]:.2f}, {pose[1]:.2f}) â†’ Target: {target['position']}")
                    
                    # Pass cube obstacles to path planner
                    obstacles = target.get("obstacles", [])
                    current_pos = (pose[0], pose[1])
                    print("planned path from wait for target")
                    self.path = self.plan_path(current_pos, target["position"], obstacles)
                    self.path_index = 0
                    
                    if self.path:
                        print(f"Path ready with {len(self.path)} waypoints")
                        self.state = "FOLLOW_PATH"
                    else:
                        print(f"No path, using direct navigation")
                        self.state = "NAV_TO_OBJECT"

             # FOLLOW_PATH: track A* waypoints toward the selected cube
            elif self.state == "FOLLOW_PATH":
                x, y = pose[0], pose[1]
                
                # Abort path if we drift near edges, re enter WAIT_FOR_TARGET for recovery
                if abs(x) > 1.85 or abs(y) > 1.85:
                    print(f"STUCK near edge at ({x:.2f},{y:.2f}) while following path!")
                    print("Aborting current path, returning to WAIT_FOR_TARGET")
                    self.motion_controller.stop()
                    self.path = None
                    self.path_index = 0
                    self.current_target = None
                    self.state = "WAIT_FOR_TARGET"
                    
                    self.perform_escape_maneuver(self.get_robot_pose())
                    continue
                
                # Opportunistic replan if supervisor updates target mid navigation
                if step_count % 10 == 0:
                    new_target = self.read_target_from_file()
                    if new_target and new_target != "STOP" and isinstance(new_target, dict):
                        if new_target.get("type") == "TARGET":
                            if (not self.current_target or 
                                new_target["color"] != self.current_target.get("color") or
                                new_target["position"] != self.current_target.get("position")):
                                
                                print(f"New target detected while following path!")
                                print(f"Old: {self.current_target.get('color', 'None')} at {self.current_target.get('position', 'None')}")
                                print(f"New: {new_target['color']} at {new_target['position']}")
                                print("Re planning path...")
                                
                                self.current_target = new_target
                                current_pos = (pose[0], pose[1])
                                obstacles = new_target.get("obstacles", [])
                                print("planned path from follow path")
                                self.path = self.plan_path(current_pos, new_target["position"], obstacles)
                                self.path_index = 0
                                
                                if not self.path:
                                    print("No path found, switching to direct navigation")
                                    self.state = "NAV_TO_OBJECT"
                                    continue
                
                #  Try to detect cubes with camera while navigating
                if step_count % 30 == 0 and self.camera:
                    try:
                        detected = self.detect_cubes_in_camera()
                        if detected:
                            print(f"Camera detection: {detected}")
                    except Exception as e:
                        pass
                
                # Advance along waypoints, or switch to final approach
                if self.path and self.path_index < len(self.path):
                    dist, self.path_index = self.motion_controller.follow_path(
                        pose, self.path, self.path_index
                    )
                    if step_count % 30 == 0:
                        print(f"Following path: waypoint {self.path_index}/{len(self.path)}")
                else:
                    print("Path complete â€“ final approach to object.")
                    self.state = "NAV_TO_OBJECT"

            # NAV_TO_OBJECT: direct approach (no path) for the last short segment
            elif self.state == "NAV_TO_OBJECT":
                # Use camera to verify we're approaching the right cube
                detected = self.detect_cubes_in_camera()
                # Supervisor signals pickup, switch to bin delivery phase
                target = self.read_target_from_file()
                if target and target["type"] == "CARRY":
                    self.carrying_color = target["color"]
                    print(f"Carrying {self.carrying_color} cube! Planning path to bin...")
                    self.path = None
                    self.path_index = 0
                    self.state = "NAV_TO_BIN"
                    continue
                if target and target["type"] == "TARGET":
                    pos = target["position"]
                    color = target["color"]
                    
                    # Verify color with camera when close
                    dist = self.motion_controller.goto_waypoint(pose, pos)
                    
                    # Simple confirmation - if close and vision agrees, log it
                    if dist < 1.0 and detected:
                        for detected_color, confidence in detected:
                            if detected_color == color:
                                print(f"Camera confirmed: {color} cube (confidence: {confidence:.2f})")
                            elif confidence > 0.5:
                                print(f"Camera sees {detected_color} but expected {color}!")
                    
                    if step_count % 20 == 0:
                        print(f"Moving to {color} cube â†’ dist={dist:.2f} m")

            # NAV_TO_BIN: plan and follow a safe path to the appropriate bin 
            elif self.state == "NAV_TO_BIN":
                if self.carrying_color in BIN_POSITIONS:
                    bin_pos = BIN_POSITIONS[self.carrying_color]
                    safe_bin_pos = self.get_safe_bin_position(bin_pos)
                    #  ALWAYS plan path to bin using A*
                    if not self.path:
                        print(f"Planning path to {self.carrying_color} bin at {safe_bin_pos}")
                        current_pos = (pose[0], pose[1])
                        
                        #  Get current obstacles from file
                        target = self.read_target_from_file()
                        obstacles = target.get("obstacles", []) if target and isinstance(target, dict) else []
                        print("planned path from nav to bin")
                        self.path = self.plan_path(current_pos, safe_bin_pos, obstacles)
                        self.path_index = 0
                        # If A* still fails, try via center explicitly
                        if not self.path:
                            print("Direct path failed, routing via (0,0)")
                            center = (0.0, 0.0)
                            path1 = astar(current_pos, center, obstacles)
                            path2 = astar(center, safe_bin_pos, obstacles)
                            if path1 and path2:
                                self.path = path1[:-1] + path2
                                self.path_index = 0
                    # Follow the planned path
                    if self.path and self.path_index < len(self.path):
                        dist, self.path_index = self.motion_controller.follow_path(
                            pose, self.path, self.path_index
                        )
                        if step_count % 30 == 0:
                            print(f"Delivering to bin: waypoint {self.path_index}/{len(self.path)}")
                    else:
                        # Final approach to bin
                        dist = self.motion_controller.goto_waypoint(pose, safe_bin_pos)
                        if step_count % 20 == 0:
                            print(f"Final approach to {self.carrying_color} bin â†’ dist={dist:.2f} m")
                        if dist < 0.40:
                            print(f"Delivered {self.carrying_color} cube successfully!")
                            self.motion_controller.stop()
                            # Escape from bin corner after delivery
                            self.perform_escape_maneuver(self.get_robot_pose())
                            # Reset state for next cube
                            self.carrying_color = None
                            self.path = None
                            self.path_index = 0
                            self.current_target = None
                            self.state = "WAIT_FOR_TARGET"
                            print("Ready for next cube!")
                else:
                    print(f"Unknown color: {self.carrying_color}")
                    self.state = "MISSION_COMPLETE"

            # MISSION_COMPLETE: log results and generate artifacts, then exit
            elif self.state == "MISSION_COMPLETE":
                self.motion_controller.stop()
                elapsed = time.time() - self.start_time
                success_ratio = len([c for c in BIN_POSITIONS.keys()]) / 4.0
                avg_time_per_cube = elapsed / 4.0

                print("=" * 40)
                print("Mission complete!")
                print(f"Time: {elapsed:.1f}s | Distance: {self.total_distance:.2f}m | Avg time per cube: {avg_time_per_cube:.1f}s")
                print("=" * 40)

                csv_path = os.path.join(os.path.dirname(__file__), "performance_log.csv")
                write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0

                try:
                    # Append a single CSV row; line buffered with flush for safety
                    with open(csv_path, "a", newline="", buffering=1) as f:
                        writer = csv.writer(f)
                        if write_header:
                            writer.writerow(["timestamp", "total_distance_m", "avg_time_per_cube_s", "success_ratio"])
                        writer.writerow([
                            datetime.now().isoformat(timespec="seconds"),
                            round(self.total_distance, 2),
                            round(avg_time_per_cube, 2),
                            round(success_ratio, 2)
                        ])
                        f.flush()  
                    print(f"Results appended to {csv_path}")
                except Exception as e:
                    print(f"Failed to write performance log: {e}")

                
                try:
                    # Collect all per-plan PNGs and stitch them into an animated GIF summary
                    paths = sorted(glob.glob(os.path.join(os.path.dirname(__file__), "path_plan_*.png")))
                    if paths:
                        imgs = [Image.open(p) for p in paths]
                        gif_path = os.path.join(os.path.dirname(__file__), "path_summary.gif")
                        imgs[0].save(gif_path, save_all=True, append_images=imgs[1:], duration=700, loop=0)
                        print(f"Created animated path summary: {gif_path}")
                except Exception as e:
                    print(f"Failed to generate path summary: {e}")
                # Clean exit: transition to DONE and break loop
                self.state = "DONE"
                break

if __name__ == "__main__":
    controller = CognitiveRobotController()
    controller.run()
