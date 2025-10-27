"""
Perception system for KUKA youBot cube detection and localization.
Handles color detection using OpenCV and pixel-to-world coordinate transformation.
"""

import cv2
import numpy as np
import math

# HSV color ranges for cube detection
HSV_RANGES = {
    "red1":   ((0, 120, 70), (10, 255, 255)),
    "red2":   ((170, 120, 70), (180, 255, 255)),
    "green":  ((35, 60, 60), (85, 255, 255)),
    "blue":   ((90, 60, 60), (130, 255, 255)),
    "yellow": ((20, 60, 60), (35, 255, 255)),
}

class PerceptionSystem:
    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step
        
        # Initialize camera
        self.camera = robot.getDevice("camera")
        self.camera.enable(time_step)
        
        # Get actual camera dimensions from Webots
        self.image_width = self.camera.getWidth()
        self.image_height = self.camera.getHeight()
        
        # Camera parameters (adjust based on your camera setup)
        self.camera_height = 0.3  # Height of camera above ground [m]
        self.camera_fov = math.radians(60)  # Field of view in radians
        
        # Camera mounting (adjust based on robot configuration)
        self.camera_offset_x = 0.1  # Forward offset from robot center [m]
        self.camera_offset_y = 0.0  # Lateral offset from robot center [m]
        self.camera_tilt = math.radians(-30)  # Downward tilt angle [rad]
        
        print("Perception system initialized")
        print(f"   Camera FOV: {math.degrees(self.camera_fov):.1f}°")
        print(f"   Image size: {self.image_width}x{self.image_height}")

    def detect_objects(self):
        """Detect colored cubes in the current camera image."""
        # Get camera image
        image_data = self.camera.getImage()
        if not image_data:
            return []
        
        # Get actual camera dimensions
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        # Convert to OpenCV format - Webots returns BGRA format
        image = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
        
        # Keep only BGR channels (drop alpha channel)
        bgr_image = image[:, :, :3]
        
        return self.detect_colors(bgr_image)

    def detect_colors(self, bgr_frame):
        """Detect colored objects using HSV color segmentation."""
        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        detections = []
        
        # Create masks for each color
        masks = {}
        for name, (lower, upper) in HSV_RANGES.items():
            masks[name] = cv2.inRange(hsv, np.array(lower), np.array(upper))
        
        # Combine red masks (red wraps around in HSV)
        if "red1" in masks and "red2" in masks:
            masks["red"] = cv2.bitwise_or(masks["red1"], masks["red2"])
            masks.pop("red1", None)
            masks.pop("red2", None)
        
        # Process each color mask
        for color, mask in masks.items():
            # Apply morphological operations to clean up mask
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by minimum area
                if area < 50:
                    continue
                
                # Get bounding rectangle and centroid
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2
                
                # Calculate aspect ratio for shape filtering
                aspect_ratio = float(w) / h if h > 0 else 0
                
                # Filter by aspect ratio (cubes should be roughly square)
                if not (0.5 <= aspect_ratio <= 2.0):
                    continue
                
                # Calculate confidence based on area and shape
                confidence = min(area / 500.0, 1.0) * (1.0 - abs(aspect_ratio - 1.0))
                
                detection = {
                    "color": color,
                    "centroid": (cx, cy),
                    "area": float(area),
                    "confidence": confidence,
                    "bbox": (x, y, w, h)
                }
                
                detections.append(detection)
        
        return detections

    def pixel_to_world(self, pixel_pos, robot_pose):
        """
        Convert pixel coordinates to world coordinates.
        
        Args:
            pixel_pos: (u, v) pixel coordinates
            robot_pose: (x, y, theta) robot pose in world frame
            
        Returns:
            (world_x, world_y) or None if conversion fails
        """
        u, v = pixel_pos
        robot_x, robot_y, robot_theta = robot_pose
        
        # Convert pixel to normalized image coordinates
        u_norm = (u - self.image_width / 2) / (self.image_width / 2)
        v_norm = (v - self.image_height / 2) / (self.image_height / 2)
        
        # Calculate ray direction in camera frame
        # Assuming camera looks forward and down
        ray_x_cam = 1.0  # Forward direction
        ray_y_cam = u_norm * math.tan(self.camera_fov / 2)
        ray_z_cam = -v_norm * math.tan(self.camera_fov / 2) + math.tan(self.camera_tilt)
        
        # Normalize ray direction
        ray_length = math.sqrt(ray_x_cam**2 + ray_y_cam**2 + ray_z_cam**2)
        if ray_length == 0:
            return None
            
        ray_x_cam /= ray_length
        ray_y_cam /= ray_length
        ray_z_cam /= ray_length
        
        # Calculate intersection with ground plane (z = 0)
        # Camera position in world frame
        cam_x_world = robot_x + self.camera_offset_x * math.cos(robot_theta) - self.camera_offset_y * math.sin(robot_theta)
        cam_y_world = robot_y + self.camera_offset_x * math.sin(robot_theta) + self.camera_offset_y * math.cos(robot_theta)
        cam_z_world = self.camera_height
        
        # Transform ray to world frame
        cos_theta = math.cos(robot_theta)
        sin_theta = math.sin(robot_theta)
        
        ray_x_world = ray_x_cam * cos_theta - ray_y_cam * sin_theta
        ray_y_world = ray_x_cam * sin_theta + ray_y_cam * cos_theta
        ray_z_world = ray_z_cam
        
        # Find intersection with ground plane (z = 0)
        if ray_z_world >= 0:  # Ray pointing up, no intersection
            return None
        
        # Parameter t for intersection: cam_z + t * ray_z = 0
        t = -cam_z_world / ray_z_world
        
        if t < 0:  # Intersection behind camera
            return None
        
        # Calculate world intersection point
        world_x = cam_x_world + t * ray_x_world
        world_y = cam_y_world + t * ray_y_world
        
        # Sanity check: reasonable distance from robot
        distance = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)
        if distance > 3.0:  # More than 3m away, likely error
            return None
        
        return (world_x, world_y)

    def get_camera_image_debug(self):
        """Get camera image for debugging purposes."""
        image_data = self.camera.getImage()
        if not image_data:
            return None
        
        # Get actual camera dimensions
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        # Convert to OpenCV format
        image = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
        
        # Keep only BGR channels and return
        return image[:, :, :3]

    def visualize_detections(self, detections, save_path=None):
        """Visualize detections on camera image for debugging."""
        image = self.get_camera_image_debug()
        if image is None:
            return None
        
        # Draw detections
        for det in detections:
            cx, cy = det["centroid"]
            color = det["color"]
            confidence = det.get("confidence", 0)
            
            # Color mapping for visualization
            color_map = {
                "red": (0, 0, 255),
                "green": (0, 255, 0),
                "blue": (255, 0, 0),
                "yellow": (0, 255, 255)
            }
            
            draw_color = color_map.get(color, (255, 255, 255))
            
            # Draw circle at centroid
            cv2.circle(image, (cx, cy), 5, draw_color, -1)
            
            # Draw bounding box if available
            if "bbox" in det:
                x, y, w, h = det["bbox"]
                cv2.rectangle(image, (x, y), (x + w, y + h), draw_color, 2)
            
            # Draw label
            label = f"{color} ({confidence:.2f})"
            cv2.putText(image, label, (cx - 20, cy - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, draw_color, 1)
        
        if save_path:
            cv2.imwrite(save_path, image)
        
        return image
