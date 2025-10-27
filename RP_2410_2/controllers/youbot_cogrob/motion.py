import math
from common import angle_wrap, distance_2d

# youBot physical parameters
WHEEL_RADIUS = 0.047
WHEEL_BASE_X = 0.228
WHEEL_BASE_Y = 0.158
LX_PLUS_LY = (WHEEL_BASE_X + WHEEL_BASE_Y) / 2

# Motion limits
MAX_LINEAR_VELOCITY = 1.0
MAX_ANGULAR_VELOCITY = 2.0
MAX_WHEEL_VELOCITY = 10.0

# MotionController handles local navigation, rotation, and path tracking
class MotionController:
    def __init__(self, robot):
        # Initialize Webots devices for wheel control
        self.robot = robot
        wheel_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
        self.wheels = []
        for name in wheel_names:
            wheel = robot.getDevice(name)
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0.0)
            self.wheels.append(wheel)

        # Navigation parameters
        self.kp_linear = 1.5
        self.kp_angular = 3.0
        self.goal_tolerance = 0.05
        self.angle_tolerance = 0.1
        self.turn_in_place_threshold = math.radians(20)

        # Obstacle avoidance
        self.obstacle_threshold = 0.6
        self.avoidance_gain = 0.5
        print("Motion controller initialized")
        print(f"   Wheel base: {WHEEL_BASE_X:.3f}m x {WHEEL_BASE_Y:.3f}m")
        print(f"   Max velocities: {MAX_LINEAR_VELOCITY:.1f}m/s, {MAX_ANGULAR_VELOCITY:.1f}rad/s")

    def set_base_velocity(self, vx, vy, omega):
        """
        Convert desired robot-frame velocities (vx, vy, ω) to individual wheel speeds.
        Follows standard mecanum kinematics.
        """
        vx = max(min(vx, MAX_LINEAR_VELOCITY), -MAX_LINEAR_VELOCITY)
        vy = max(min(vy, MAX_LINEAR_VELOCITY), -MAX_LINEAR_VELOCITY)
        omega = max(min(omega, MAX_ANGULAR_VELOCITY), -MAX_ANGULAR_VELOCITY)
        # youBot mecanum kinematics
        w1 = (vx - vy - LX_PLUS_LY * omega) / WHEEL_RADIUS
        w2 = (vx + vy + LX_PLUS_LY * omega) / WHEEL_RADIUS
        w3 = (vx + vy - LX_PLUS_LY * omega) / WHEEL_RADIUS
        w4 = (vx - vy + LX_PLUS_LY * omega) / WHEEL_RADIUS
        for wheel, vel in zip(self.wheels, [w1, w2, w3, w4]):
            wheel.setVelocity(max(min(vel, MAX_WHEEL_VELOCITY), -MAX_WHEEL_VELOCITY))
    
    def stop(self):
        """Stop all wheel motion immediately."""
        self.set_base_velocity(0.0, 0.0, 0.0)

    def goto_waypoint(self, current_pose, target_pos, use_pure_pursuit=False):
        """
        Navigate to a target waypoint (X-Y plane, ENU).
        Fixes:
          • correct heading error sign
          • when goal is behind: rotate-in-place (or gentle reverse) instead of pushing forward
          • world-bounds safety
        """
        x, y, theta = current_pose
        target_x, target_y = target_pos
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)
        if distance < self.goal_tolerance:
            self.stop()
            return distance
        # Simple repulsion from bin pillars
        try:
            from youbot_cogrob import BIN_POSITIONS
            ax, ay = 0.0, 0.0
            for bx, by in BIN_POSITIONS.values():
                d = math.hypot(x - bx, y - by)
                if d < self.obstacle_threshold:
                    k = self.obstacle_threshold - d
                    ax += k * (x - bx)
                    ay += k * (y - by)
            dx += self.avoidance_gain * ax
            dy += self.avoidance_gain * ay
        except Exception:
            pass
        desired_heading = math.atan2(dy, dx)

        # correct sign (desired - current), not the other way around
        heading_error = angle_wrap(theta - desired_heading)
        if abs(heading_error) > self.turn_in_place_threshold:
            # STATE 1: ROTATE 
            # The heading error is large so we stop and turn in place.
            vx_cmd = 0.0
            omega_cmd = self.kp_angular * heading_error
        else:
            # STATE 2: DRIVE 
            # The robot is pointing mostly in the right direction, so we drive forward.
            # We still apply a small, dampened turning correction to stay on course.
            vx_cmd = self.kp_linear * distance
            omega_cmd = self.kp_angular * heading_error * 0.5  # Dampen turning while driving
        # Clamp angular speed
        omega_cmd = max(min(omega_cmd, MAX_ANGULAR_VELOCITY), -MAX_ANGULAR_VELOCITY)
        # Slow down near target
        if distance < 0.2:
            vx_cmd *= 0.5
        print(f"Driving | dist={distance:.2f}m, err={heading_error:.2f} rad, vx={vx_cmd:.2f}, ω={omega_cmd:.2f}")

        # Safety to keep inside world
        if not (-1.95 <= x <= 1.95 and -1.95 <= y <= 1.95):
            print("Near world edge, turning to center to prevent falling off!")
            center_heading = math.atan2(-y, -x)
            heading_error = -angle_wrap(center_heading - theta)
            vx_cmd = 0
            omega_cmd = self.kp_angular * heading_error
        self.set_base_velocity(vx_cmd, 0.0, omega_cmd)
        return distance
    
    def follow_path(self, current_pose, path, path_index=0):
        """
        Move the robot along the path (list of (x, y) waypoints).
        Returns current distance to target and updated index.
        """
        if not path or path_index >= len(path):
            self.stop()
            return 0.0, path_index
        distance = self.goto_waypoint(current_pose, path[path_index])
        if distance < self.goal_tolerance:
            path_index += 1
            if path_index < len(path):
                print(f"Reached waypoint {path_index}/{len(path)}")
        return distance, path_index
    
    def rotate_to_heading(self, current_pose, target_heading):
        _, _, current_heading = current_pose
        heading_error = angle_wrap(target_heading - current_heading)
        if abs(heading_error) < self.angle_tolerance:
            self.stop()
            return heading_error
        omega = self.kp_angular * heading_error
        self.set_base_velocity(0.0, 0.0, omega)
        return heading_error
    def move_relative(self, dx, dy, dtheta=0.0):
        vx = self.kp_linear * dx
        vy = self.kp_linear * dy
        omega = self.kp_angular * dtheta
        self.set_base_velocity(vx, vy, omega)
    def emergency_stop(self):
        print("🛑 EMERGENCY STOP")
        self.stop()
