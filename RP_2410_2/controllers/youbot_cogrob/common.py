"""
Common utility functions for KUKA youBot cognitive robotics system.
Provides shared mathematical and helper functions used across modules.
"""

import math
import numpy as np

def angle_wrap(angle):
    """
    Wrap angle to the range [-pi, pi] in a numerically stable way.
    """
    # Normalize using floating-point modulo for stability
    angle = math.fmod(angle + math.pi, 2 * math.pi)
    if angle < 0:
        angle += 2 * math.pi
    return angle - math.pi


def distance_2d(pos1, pos2):
    """
    Calculate Euclidean distance between two 2D points (robust version).
    """
    try:
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        return math.hypot(dx, dy)
    except Exception:
        return float('inf')


def distance_3d(pos1, pos2):
    """
    Calculate Euclidean distance between two 3D points.
    
    Args:
        pos1: (x, y, z) first position
        pos2: (x, y, z) second position
        
    Returns:
        Distance between points
    """
    dx = pos2[0] - pos1[0]
    dy = pos2[1] - pos1[1]
    dz = pos2[2] - pos1[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def normalize_angle_difference(angle1, angle2):
    """
    Calculate the normalized difference between two angles.
    
    Args:
        angle1: First angle in radians
        angle2: Second angle in radians
        
    Returns:
        Angle difference in [-pi, pi]
    """
    diff = angle2 - angle1
    return angle_wrap(diff)

def clamp(value, min_val, max_val):
    """
    Clamp value to specified range.
    
    Args:
        value: Value to clamp
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        Clamped value
    """
    return max(min_val, min(value, max_val))

def lerp(a, b, t):
    """
    Linear interpolation between two values.
    
    Args:
        a: Start value
        b: End value
        t: Interpolation parameter [0, 1]
        
    Returns:
        Interpolated value
    """
    return a + t * (b - a)

def pose_to_transform_matrix(x, y, theta):
    """
    Convert 2D pose to homogeneous transformation matrix.
    
    Args:
        x: X position
        y: Y position
        theta: Rotation angle in radians
        
    Returns:
        3x3 transformation matrix
    """
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    
    return np.array([
        [cos_theta, -sin_theta, x],
        [sin_theta,  cos_theta, y],
        [0,          0,         1]
    ])

def transform_point(point, transform_matrix):
    """
    Transform a 2D point using homogeneous transformation matrix.
    
    Args:
        point: (x, y) point to transform
        transform_matrix: 3x3 transformation matrix
        
    Returns:
        Transformed (x, y) point
    """
    homogeneous_point = np.array([point[0], point[1], 1])
    transformed = transform_matrix @ homogeneous_point
    return (transformed[0], transformed[1])

def moving_average(values, window_size):
    """
    Calculate moving average of a list of values.
    
    Args:
        values: List of numerical values
        window_size: Size of the moving window
        
    Returns:
        List of moving averages
    """
    if len(values) < window_size:
        return values
    
    averages = []
    for i in range(len(values) - window_size + 1):
        window = values[i:i + window_size]
        averages.append(sum(window) / window_size)
    
    return averages

def is_point_in_circle(point, center, radius):
    """
    Check if a point is inside a circle.
    
    Args:
        point: (x, y) point to check
        center: (x, y) circle center
        radius: Circle radius
        
    Returns:
        True if point is inside circle
    """
    return distance_2d(point, center) <= radius

def is_point_in_rectangle(point, rect_min, rect_max):
    """
    Check if a point is inside a rectangle.
    
    Args:
        point: (x, y) point to check
        rect_min: (x, y) minimum corner of rectangle
        rect_max: (x, y) maximum corner of rectangle
        
    Returns:
        True if point is inside rectangle
    """
    x, y = point
    return (rect_min[0] <= x <= rect_max[0] and 
            rect_min[1] <= y <= rect_max[1])

def calculate_heading(from_pos, to_pos):
    """
    Calculate heading angle from one position to another.
    
    Args:
        from_pos: (x, y) starting position
        to_pos: (x, y) target position
        
    Returns:
        Heading angle in radians
    """
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    return math.atan2(dy, dx)

def rotate_point(point, angle, center=(0, 0)):
    """
    Rotate a point around a center by given angle.
    
    Args:
        point: (x, y) point to rotate
        angle: Rotation angle in radians
        center: (x, y) center of rotation
        
    Returns:
        Rotated (x, y) point
    """
    # Translate to origin
    x = point[0] - center[0]
    y = point[1] - center[1]
    
    # Rotate
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    
    x_rot = x * cos_a - y * sin_a
    y_rot = x * sin_a + y * cos_a
    
    # Translate back
    return (x_rot + center[0], y_rot + center[1])

def smooth_trajectory(waypoints, smoothing_factor=0.5):
    """
    Smooth a trajectory using simple averaging.
    
    Args:
        waypoints: List of (x, y) waypoints
        smoothing_factor: Smoothing strength [0, 1]
        
    Returns:
        Smoothed list of waypoints
    """
    if len(waypoints) < 3:
        return waypoints
    
    smoothed = [waypoints[0]]  # Keep first waypoint
    
    for i in range(1, len(waypoints) - 1):
        prev_wp = waypoints[i - 1]
        curr_wp = waypoints[i]
        next_wp = waypoints[i + 1]
        
        # Average with neighbors
        avg_x = (prev_wp[0] + curr_wp[0] + next_wp[0]) / 3
        avg_y = (prev_wp[1] + curr_wp[1] + next_wp[1]) / 3
        
        # Blend with original
        smooth_x = lerp(curr_wp[0], avg_x, smoothing_factor)
        smooth_y = lerp(curr_wp[1], avg_y, smoothing_factor)
        
        smoothed.append((smooth_x, smooth_y))
    
    smoothed.append(waypoints[-1])  # Keep last waypoint
    return smoothed

def format_time(seconds):
    """
    Format time in seconds to human-readable string.
    
    Args:
        seconds: Time in seconds
        
    Returns:
        Formatted time string
    """
    if seconds < 60:
        return f"{seconds:.1f}s"
    elif seconds < 3600:
        minutes = int(seconds // 60)
        secs = seconds % 60
        return f"{minutes}m {secs:.1f}s"
    else:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = seconds % 60
        return f"{hours}h {minutes}m {secs:.1f}s"

def create_color_map():
    """
    Create a color mapping for visualization.
    
    Returns:
        Dictionary mapping color names to RGB tuples
    """
    return {
        "red": (255, 0, 0),
        "green": (0, 255, 0),
        "blue": (0, 0, 255),
        "yellow": (255, 255, 0),
        "cyan": (0, 255, 255),
        "magenta": (255, 0, 255),
        "white": (255, 255, 255),
        "black": (0, 0, 0),
        "gray": (128, 128, 128)
    }

def validate_pose(pose):
    """
    Validate that a pose tuple has correct format.
    
    Args:
        pose: Pose tuple to validate
        
    Returns:
        True if pose is valid
    """
    if not isinstance(pose, (tuple, list)) or len(pose) != 3:
        return False
    
    try:
        x, y, theta = pose
        return all(isinstance(val, (int, float)) for val in [x, y, theta])
    except (ValueError, TypeError):
        return False

def validate_position(position):
    """
    Validate that a position tuple has correct format.
    
    Args:
        position: Position tuple to validate
        
    Returns:
        True if position is valid
    """
    if not isinstance(position, (tuple, list)) or len(position) != 2:
        return False
    
    try:
        x, y = position
        return all(isinstance(val, (int, float)) for val in [x, y])
    except (ValueError, TypeError):
        return False
