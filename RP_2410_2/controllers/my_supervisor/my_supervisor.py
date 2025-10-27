from controller import Supervisor
import math
import os
import random
import time

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Define the path to the shared target file for communication with the robot controller
project_dir = os.path.dirname(os.path.abspath(__file__))
target_file_path = os.path.normpath(os.path.join(project_dir, "..", "youbot_cogrob", "target.txt"))
try:
    with open(target_file_path, "w", encoding="utf-8") as f:
        f.write("")     # Clear any existing target at startup
    print("Supervisor: cleared target.txt at startup.")
except Exception as e:
    print(f"Supervisor: failed clearing target.txt: {e}")
print(f"Target file path: {target_file_path}")

# Fixed bin locations in the environment - x, y coordinates
BIN_POSITIONS = {
    "yellow": (-1.87, -1.87),
    "blue": (1.87, -1.87),
    "red": (-1.87, 1.87),
    "green": (1.87, 1.87)
}
# Retrieve references to the robot and cube objects from the Webots world
robot = supervisor.getFromDef("YOUBOT")
boxes = {
    "red": supervisor.getFromDef("RED_BOX"),
    "green": supervisor.getFromDef("GREEN_BOX"),
    "yellow": supervisor.getFromDef("YELLOW_BOX"),
    "blue": supervisor.getFromDef("BLUE_BOX"),
}

# RANDOMIZE ROBOT AND CUBE POSITIONS AT START ===

print("Randomizing initial robot and cube positions...")

# Define safe spawning parameters 
SPAWN_AREA_MIN = -0.9
SPAWN_AREA_MAX = 0.9
CUBE_HEIGHT_ON_FLOOR = 0.03
ROBOT_START_HEIGHT = 0.0 # Robot base is at ground level

# Define minimum separation distances 
MIN_CUBE_TO_CUBE_DISTANCE = 0.8   # Cubes shouldn't spawn on top of each other
MIN_ROBOT_TO_CUBE_DISTANCE = 1.2  # Ensure robot doesn't start too close to a cube

# Randomize the Robot's Position First 
random_robot_x = random.uniform(SPAWN_AREA_MIN, SPAWN_AREA_MAX)
random_robot_y = random.uniform(SPAWN_AREA_MIN, SPAWN_AREA_MAX)
random_robot_yaw = random.uniform(0, 2 * math.pi)

# Get the robot's position and rotation fields and set their new random values
robot_translation_field = robot.getField("translation")
robot_translation_field.setSFVec3f([random_robot_x, random_robot_y, ROBOT_START_HEIGHT])

robot_rotation_field = robot.getField("rotation")
robot_rotation_field.setSFRotation([0, 0, 1, random_robot_yaw]) # Z axis for yaw

# We need to give the simulation a tiny moment to apply the change
supervisor.step(timestep)
# Now get the final robot position for distance checks
robot_pos = robot.getField("translation").getSFVec3f()
print(f"   -> Moved Robot to ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}) with yaw {math.degrees(random_robot_yaw):.1f}°")
# try to write the initial target to the closest cube
try:
    # choosre closest cube
    closest_color, closest_dist = None, 999
    for color, box in boxes.items():
        if not box:
            continue
        pos = box.getField("translation").getSFVec3f()
        dist = math.hypot(pos[0] - robot_pos[0], pos[1] - robot_pos[1])
        if dist < closest_dist:
            closest_color, closest_pos, closest_dist = color, pos, dist

    if closest_color:
        with open(target_file_path, "w", encoding="utf-8") as f:
            f.write(f"{closest_color} {closest_pos[0]} {closest_pos[1]} 0")
            f.flush()
            os.fsync(f.fileno())
        print(f"Initial target written: {closest_color} at ({closest_pos[0]:.2f}, {closest_pos[1]:.2f})")
    else:
        print("No cubes found to write initial target.")
except Exception as e:
    print(f"Failed to write initial target: {e}")


# Randomize Cube Positions, avoiding the robot and other cubes 
placed_cube_positions = []  # Store positions of already placed cubes

for color, box_node in boxes.items():
    if box_node:
        # Loop until we find a valid position for the current cube
        while True:
            # Generate a candidate random position
            candidate_x = random.uniform(SPAWN_AREA_MIN, SPAWN_AREA_MAX)
            candidate_y = random.uniform(SPAWN_AREA_MIN, SPAWN_AREA_MAX)

            # Check distance from the robot's starting position
            is_too_close_to_robot = (
                math.hypot(candidate_x - robot_pos[0], candidate_y - robot_pos[1]) < MIN_ROBOT_TO_CUBE_DISTANCE
            )

            # Check distance from all previously placed cubes
            is_too_close_to_other_cubes = any(
                math.hypot(candidate_x - px, candidate_y - py) < MIN_CUBE_TO_CUBE_DISTANCE
                for px, py in placed_cube_positions
            )

            # If the position is valid we can use it
            if not is_too_close_to_robot and not is_too_close_to_other_cubes:
                placed_cube_positions.append((candidate_x, candidate_y))
                break # Exit the while True loop and place the cube

        # Get the field that controls the box's position and move it
        translation_field = box_node.getField("translation")
        translation_field.setSFVec3f([candidate_x, candidate_y, CUBE_HEIGHT_ON_FLOOR])

        # randomize the cube's rotation as well
        rotation_field = box_node.getField("rotation")
        random_yaw = random.uniform(0, 2 * math.pi)
        rotation_field.setSFRotation([0, 0, 1, random_yaw])

        print(f"   -> Moved {color} cube to ({candidate_x:.2f}, {candidate_y:.2f})")


print("Supervisor started! Tracking all boxes...")
attached_color = None
state = "HUNT"
collected_boxes = []

def hide_box(box):
    """Attach the selected cube to the robot and disable its physics to prevent interference."""
    try:
        robot_pos = robot.getField("translation").getSFVec3f()
        carry_height = 0.6
        box.getField("translation").setSFVec3f([
            robot_pos[0],
            robot_pos[1],
            carry_height
        ])
        # rotation to align with robot
        box.getField("rotation").setSFRotation([0, 1, 0, 0])
        # cut physics to avoid interference
        physics_field = box.getField("physics")
        if physics_field:
            physics_field.setSFNode(None)
            print("Physics disabled for the carried box.")
        else:
            print("No explicit physics field found — visual only (still fine).")
        print(f"Box placed on top of the robot at {carry_height} m (physics off).")
    except Exception as e:
        print(f"Failed to carry box: {e}")


def show_box(color, position):
    """
    Place a box near its bin with a simple drop animation (no physics).
    """
    try:
        box = boxes[color]
        if box:
            # Nudge slightly toward room center so it doesn't sit exactly on the bin center
            offset = 0.2
            # x, y = position
            # x = x - offset if x > 0 else x + offset
            # y = y - offset if y > 0 else y + offset
            x, y = position

            z_start = 0.35
            z_end = 0.1
            steps = 10

            # Small spin during the fall to make it look alive
            for i in range(steps):
                z = z_start - (z_start - z_end) * (i + 1) / steps
                angle = 0.2 * i
                box.getField("translation").setSFVec3f([x, y, z])
                box.getField("rotation").setSFRotation([0, 1, 0, angle])
                supervisor.step(int(timestep / 2))

            # Final crisp placement
            box.getField("translation").setSFVec3f([x, y, z_end])
            box.getField("rotation").setSFRotation([0, 1, 0, 0])

            print(f"{color} box placed near its bin at ({x:.2f}, {y:.2f}) with drop animation.")
    except Exception as e:
        print(f"Failed to show box: {e}")


# simulation main loop
while supervisor.step(timestep) != -1:
    if not robot:
        continue
    robot_pos = robot.getField("translation").getSFVec3f()
    if state == "DELIVER" and attached_color:
        carried_box = boxes.get(attached_color)
        if carried_box:
            robot_pos = robot.getField("translation").getSFVec3f()
            carried_box.getField("translation").setSFVec3f([
                robot_pos[0],
                robot_pos[1],
                0.25  # driving height constant
            ])
        bin_pos = BIN_POSITIONS[attached_color]
        dx = robot_pos[0] - bin_pos[0]
        dy = robot_pos[1] - bin_pos[1]
        dist_to_bin = math.sqrt(dx ** 2 + dy ** 2)
        if timestep % 500 == 0:
            print(f"Delivering {attached_color} → distance to bin: {dist_to_bin:.3f} m")
        if dist_to_bin < 1:
            print(f"Arrived near {attached_color} bin (dist={dist_to_bin:.2f}) — dropping cube!")
            try:
                with open(target_file_path, "w") as f:
                    f.write("")
                print("Cleared target.txt (ready for next cube).")
            except Exception as e:
                print(f"Failed to clear target file: {e}")
            # release the box with animation
            show_box(attached_color, [bin_pos[0], bin_pos[1]])
            collected_boxes.append(attached_color)
            boxes[attached_color] = None
            # check if mission continues or ends
            if len(collected_boxes) < 4:  # there are more cubes to collect
                print(f"Mission continues! Collected: {len(collected_boxes)}/4")
                print(f"Still need: {[c for c in boxes.keys() if c not in collected_boxes]}")
                attached_color = None
                state = "HUNT"
            else:
                print("=" * 60)
                print(f"MISSION COMPLETE! Collected all 4 boxes!")
                print(f"Order: {' → '.join(collected_boxes)}")
                print("=" * 60)
                state = "DONE"
                try:
                    with open(target_file_path, "w") as f:
                        f.write("STOP")
                except Exception as e:
                    print(f"Error writing STOP: {e}")
        else:
            continue
    # Done state
    elif state == "DONE":
        try:
            with open(target_file_path, "w") as f:
                f.write("STOP")
        except:
            pass
        continue
    # HUNT state
    elif state == "HUNT":
        candidates = []
        
        print(f"HUNT mode - scanning for boxes...")
        print(f"   Boxes status: {[(c, 'collected' if not b else 'active') for c, b in boxes.items()]}")
        print(f"   Collected so far: {collected_boxes}")
        
        for color, box in boxes.items():
            # skip if box already collected
            if not box:
                continue
            
            # skip if box already collected
            if color in collected_boxes:
                continue
            
            pos = box.getField("translation").getSFVec3f()
            
            # chack if box is hidden (out of bounds)
            if abs(pos[0]) > 50 or abs(pos[1]) > 50:  
                print(f"{color}: hidden at ({pos[0]:.1f}, {pos[1]:.1f})")
                continue
            
            dx, dy = robot_pos[0] - pos[0], robot_pos[1] - pos[1]
            dist = math.sqrt(dx ** 2 + dy ** 2)
            
            print(f"{color}: active at ({pos[0]:.2f}, {pos[1]:.2f}), distance: {dist:.2f}m")
            candidates.append((color, pos, dist))
        
        print(f"Candidates found: {len(candidates)}")
        
        if not candidates:
            if timestep % 1000 == 0:
                print("No candidates - checking if mission complete...")
                # check if mission complete and there is really nothing left
                if len(collected_boxes) >= 4:
                    print("All boxes collected! Sending STOP signal.")
                    state = "DONE"
                else:
                    print("No visible boxes, continuing search...")
            continue
        
        # Sort by distance - always pick closest
        candidates.sort(key=lambda x: x[2])
        closest_color, closest_pos, actual_dist = candidates[0]
        
        print(f"Available boxes: {[(c, f'{d:.2f}m') for c, _, d in candidates]}")
        print(f"Selected: {closest_color.upper()} at {actual_dist:.2f}m")
        try:
            with open(target_file_path, "w") as f:
                # Use actual distance for pickup check
                if closest_color and actual_dist < 0.5: 
                    print(f"Robot reached {closest_color} box — collecting it!")
                    attached_color = closest_color
                    hide_box(boxes[closest_color])
                    state = "DELIVER"
                    line = f"CARRY {closest_color}"

                    # Add positions of other cubes as obstacles
                    for color, box in boxes.items():
                        if color != closest_color and box and color not in collected_boxes:
                            pos = box.getField("translation").getSFVec3f()
                            if abs(pos[0]) < 50 and abs(pos[1]) < 50:  # Box is visible
                                line += f" {color}_obs {pos[0]} {pos[1]}"

                    f.write(line)
                elif closest_color and closest_pos:
                    # Write target AND all other cube positions for obstacle avoidance
                    line = f"{closest_color} {closest_pos[0]} {closest_pos[1]} 0"
                    
                    # Add positions of other cubes as obstacles
                    for color, box in boxes.items():
                        if color != closest_color and box and color not in collected_boxes:
                            pos = box.getField("translation").getSFVec3f()
                            if abs(pos[0]) < 50 and abs(pos[1]) < 50:  # Box is visible
                                line += f" {color}_obs {pos[0]} {pos[1]}"
                    
                    f.write(line)
                    if timestep % 200 == 0:  # Print less frequently
                        print(f"Closest box: {closest_color.upper()} ({actual_dist:.3f} m)")
                else:
                    if timestep % 1000 == 0:
                        print("No visible box right now, continuing search...")
                    continue
        except Exception as e:
            print(f"Error writing target file: {e}")
    if timestep % 1000 == 0:
        print("-" * 50)
