#!/usr/bin/env python3

"""Forest Traversal Mission for a swarm of drones with enhanced formation obstacle avoidance."""

__authors__ = 'Your Name'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import sys
import math
import yaml
import time
import numpy as np
from typing import List, Dict, Tuple, Optional
from math import radians, cos, sin, sqrt, atan2
import rclpy
from rclpy.node import Node
from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from std_msgs.msg import ColorRGBA

# Constants
SAFE_DISTANCE = 0.5  # Safe distance from obstacles (m)
FORMATION_DISTANCE = 1.0  # Distance between drones in formation (m)
MAX_SPEED = 0.5  # Maximum speed (m/s)
OBSTACLE_CHECK_RADIUS = 2.0  # Radius to check for obstacles (m)
MIN_INTER_DRONE_DISTANCE = 0.8  # Minimum distance between drones (m)

class Choreographer:
    """Formation Choreographer"""

    @staticmethod
    def line_formation(length: float, num_drones: int, orientation: float = 0.0, center: list = [0.0, 0.0]):
        """Line formation"""
        theta = radians(orientation)
        waypoints = []
        
        if num_drones == 1:
            return [center]
            
        step = length / (num_drones - 1) if num_drones > 1 else 0
        
        for i in range(num_drones):
            offset = -length/2 + i * step
            x = offset * cos(theta) + center[0]
            y = offset * sin(theta) + center[1]
            waypoints.append([x, y])
            
        return waypoints

    @staticmethod
    def v_formation(length: float, num_drones: int, orientation: float = 0.0, center: list = [0.0, 0.0]):
        """V formation"""
        theta = radians(orientation)
        waypoints = []
        
        if num_drones == 1:
            return [center]
            
        # Leader position
        waypoints.append([center[0], center[1]])
        
        # Calculate positions for followers
        angle_between_drones = radians(30)  # 30 degrees between drones
        
        # Left wing
        for i in range(1, (num_drones + 1) // 2):
            distance = i * length
            x = -distance * cos(theta + angle_between_drones) + center[0]
            y = -distance * sin(theta + angle_between_drones) + center[1]
            waypoints.append([x, y])
            
        # Right wing
        for i in range(1, num_drones // 2 + 1):
            distance = i * length
            x = -distance * cos(theta - angle_between_drones) + center[0]
            y = -distance * sin(theta - angle_between_drones) + center[1]
            waypoints.append([x, y])
            
        return waypoints

    @staticmethod
    def column_formation(length: float, num_drones: int, orientation: float = 0.0, center: list = [0.0, 0.0]):
        """Column formation"""
        # Just rotate the line formation by 90 degrees
        return Choreographer.line_formation(length, num_drones, orientation + 90, center)

    @staticmethod
    def distance(p1, p2):
        """Calculate Euclidean distance between two points"""
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        
    @staticmethod
    def adjust_formation_for_obstacles(formation_waypoints, obstacles, stage_center, safe_distance):
        """Adjust formation to avoid obstacles
        
        Args:
            formation_waypoints: List of waypoints for formation
            obstacles: List of obstacle positions relative to stage center
            stage_center: Stage center position
            safe_distance: Safe distance from obstacles
            
        Returns:
            Adjusted formation waypoints
        """
        adjusted_waypoints = formation_waypoints.copy()
        
        # Convert obstacles to absolute positions
        absolute_obstacles = []
        for obs in obstacles:
            absolute_obstacles.append([obs[0] + stage_center[0], obs[1] + stage_center[1]])
        
        # Check each drone position against obstacles
        for i, wp in enumerate(adjusted_waypoints):
            # For each obstacle, check if drone is too close
            for obs in absolute_obstacles:
                dist = Choreographer.distance(wp, obs)
                
                # If too close, move the drone away from obstacle
                if dist < safe_distance:
                    # Vector from obstacle to drone
                    vec = [wp[0] - obs[0], wp[1] - obs[1]]
                    vec_len = sqrt(vec[0]**2 + vec[1]**2)
                    
                    # Normalize vector
                    if vec_len > 0:
                        vec = [vec[0]/vec_len, vec[1]/vec_len]
                    else:
                        # If drone is exactly at obstacle position (unlikely), move it in a random direction
                        angle = np.random.uniform(0, 2*np.pi)
                        vec = [cos(angle), sin(angle)]
                    
                    # Move drone away from obstacle to maintain safe distance
                    adjusted_waypoints[i] = [
                        obs[0] + vec[0] * safe_distance * 1.2,  # Add 20% margin
                        obs[1] + vec[1] * safe_distance * 1.2
                    ]
        
        # After adjusting for obstacles, check inter-drone distances
        # to maintain minimum separation
        for i in range(len(adjusted_waypoints)):
            for j in range(i+1, len(adjusted_waypoints)):
                dist = Choreographer.distance(adjusted_waypoints[i], adjusted_waypoints[j])
                
                if dist < MIN_INTER_DRONE_DISTANCE:
                    # Vector from drone j to drone i
                    vec = [
                        adjusted_waypoints[i][0] - adjusted_waypoints[j][0],
                        adjusted_waypoints[i][1] - adjusted_waypoints[j][1]
                    ]
                    vec_len = sqrt(vec[0]**2 + vec[1]**2)
                    
                    # Normalize vector
                    if vec_len > 0:
                        vec = [vec[0]/vec_len, vec[1]/vec_len]
                    else:
                        # If drones are at same position, move in random direction
                        angle = np.random.uniform(0, 2*np.pi)
                        vec = [cos(angle), sin(angle)]
                    
                    # Move drones apart
                    move_distance = (MIN_INTER_DRONE_DISTANCE - dist) / 2
                    adjusted_waypoints[i] = [
                        adjusted_waypoints[i][0] + vec[0] * move_distance,
                        adjusted_waypoints[i][1] + vec[1] * move_distance
                    ]
                    adjusted_waypoints[j] = [
                        adjusted_waypoints[j][0] - vec[0] * move_distance,
                        adjusted_waypoints[j][1] - vec[1] * move_distance
                    ]
                    
        return adjusted_waypoints

class PathPlanner:
    """Path planning for forest traversal"""
    
    def __init__(self, obstacles, start_point, end_point, stage_center, safe_distance=SAFE_DISTANCE):
        """Initialize the path planner
        
        Args:
            obstacles: List of obstacle positions relative to stage center
            start_point: Start point relative to stage center
            end_point: End point relative to stage center
            stage_center: Stage center coordinates [x, y]
            safe_distance: Safe distance from obstacles
        """
        self.safe_distance = safe_distance
        self.stage_center = stage_center
        
        # Convert relative positions to absolute positions
        self.start = [start_point[0] + stage_center[0], start_point[1] + stage_center[1]]
        self.end = [end_point[0] + stage_center[0], end_point[1] + stage_center[1]]
        self.obstacles = obstacles
        
        # Expanded obstacles with safe distance for path planning
        self.expanded_obstacles = []
        for obs in self.obstacles:
            self.expanded_obstacles.append([obs[0] + self.stage_center[0], obs[1] + self.stage_center[1], self.safe_distance])
    
    def is_collision_free(self, p1, p2):
        """Check if a line segment from p1 to p2 is collision-free
        
        Args:
            p1: Start point [x, y]
            p2: End point [x, y]
            
        Returns:
            bool: True if collision-free, False otherwise
        """
        for obs in self.expanded_obstacles:
            # Check if the line segment intersects with the expanded obstacle
            # Using line-circle intersection algorithm
            
            # Vector from p1 to obstacle center
            oc = [obs[0] - p1[0], obs[1] - p1[1]]
            
            # Direction vector of the line
            dir_vec = [p2[0] - p1[0], p2[1] - p1[1]]
            length = sqrt(dir_vec[0]**2 + dir_vec[1]**2)
            
            if length < 1e-6:  # Avoid division by zero
                # If p1 and p2 are very close, just check distance to obstacle
                dist = sqrt(oc[0]**2 + oc[1]**2)
                if dist < obs[2]:
                    return False
                continue
                
            # Normalize direction vector
            dir_vec = [dir_vec[0] / length, dir_vec[1] / length]
            
            # Projection of oc onto dir_vec
            t = oc[0] * dir_vec[0] + oc[1] * dir_vec[1]
            
            # Closest point on the line to the obstacle center
            if t < 0:
                closest = p1
            elif t > length:
                closest = p2
            else:
                closest = [p1[0] + t * dir_vec[0], p1[1] + t * dir_vec[1]]
            
            # Check distance from closest point to obstacle center
            dist = sqrt((closest[0] - obs[0])**2 + (closest[1] - obs[1])**2)
            if dist < obs[2]:
                return False
                
        return True
    
    def find_path_rrt(self, max_iterations=3000, step_size=0.3):
        """Find a path using RRT (Rapidly-exploring Random Tree) with improved sampling
        
        Args:
            max_iterations: Maximum number of iterations
            step_size: Step size for extending the tree
            
        Returns:
            List of waypoints from start to end
        """
        print(f"Starting RRT path planning from {self.start} to {self.end}")
        print(f"Number of obstacles: {len(self.obstacles)}")
        
        # Determine the planning area with padding
        padding = 5.0  # Larger area to explore
        x_min = min(self.start[0], self.end[0]) - padding
        x_max = max(self.start[0], self.end[0]) + padding
        y_min = min(self.start[1], self.end[1]) - padding
        y_max = max(self.start[1], self.end[1]) + padding
        
        # Initialize the tree with the start point
        tree = {0: {'point': self.start, 'parent': None}}
        
        # Direct path check - if we can go directly, don't bother with RRT
        if self.is_collision_free(self.start, self.end):
            print("Direct path to goal is collision-free!")
            return [self.start, self.end]
        
        # Add some intermediate waypoints if direct path is not possible
        mid_x = (self.start[0] + self.end[0]) / 2
        mid_y = (self.start[1] + self.end[1]) / 2
        
        # Try a few predetermined intermediate points
        alternate_paths = [
            [self.start, [mid_x, mid_y], self.end],
            [self.start, [mid_x + 2.0, mid_y], self.end],
            [self.start, [mid_x - 2.0, mid_y], self.end],
            [self.start, [mid_x, mid_y + 2.0], self.end],
            [self.start, [mid_x, mid_y - 2.0], self.end],
        ]
        
        for path in alternate_paths:
            if (self.is_collision_free(path[0], path[1]) and 
                self.is_collision_free(path[1], path[2])):
                print("Found collision-free path using predetermined waypoint")
                return path
        
        # If predefined paths don't work, try the RRT algorithm
        print("Trying RRT path finding...")
        
        # Use bidirectional RRT
        start_tree = {0: {'point': self.start, 'parent': None}}
        goal_tree = {0: {'point': self.end, 'parent': None}}
        
        for i in range(max_iterations):
            # Print progress occasionally
            if i % 500 == 0 and i > 0:
                print(f"RRT iteration {i}/{max_iterations}, tree size: {len(start_tree) + len(goal_tree)}")
            
            # Alternate between trees
            if i % 2 == 0:
                # Extend start tree
                source_tree, target_tree = start_tree, goal_tree
                is_start_tree = True
            else:
                # Extend goal tree
                source_tree, target_tree = goal_tree, start_tree
                is_start_tree = False
            
            # Biased sampling: increase probability of sampling goal or areas along direct path
            rand_val = np.random.random()
            if rand_val < 0.1:  # 10% chance to sample goal or start
                if is_start_tree:
                    sample = self.end
                else:
                    sample = self.start
            elif rand_val < 0.3:  # 20% chance to sample along the direct path
                alpha = np.random.random()
                sample = [
                    self.start[0] + alpha * (self.end[0] - self.start[0]),
                    self.start[1] + alpha * (self.end[1] - self.start[1])
                ]
                # Add some noise to avoid getting stuck in narrow passages
                sample[0] += np.random.uniform(-1.0, 1.0)
                sample[1] += np.random.uniform(-1.0, 1.0)
            else:  # Regular random sampling
                # Sample with bias toward the goal region
                if np.random.random() < 0.2:  # 20% chance of sampling near goal
                    center = self.end if is_start_tree else self.start
                    radius = 3.0  # Sampling radius
                    angle = np.random.uniform(0, 2 * np.pi)
                    distance = np.random.uniform(0, radius)
                    sample = [
                        center[0] + distance * cos(angle),
                        center[1] + distance * sin(angle)
                    ]
                else:
                    # Regular uniform sampling in the entire space
                    sample = [
                        np.random.uniform(x_min, x_max),
                        np.random.uniform(y_min, y_max)
                    ]
            
            # Find the nearest node in the source tree
            nearest_idx = min(source_tree.keys(), key=lambda idx: 
                          (source_tree[idx]['point'][0] - sample[0])**2 + 
                          (source_tree[idx]['point'][1] - sample[1])**2)
            nearest = source_tree[nearest_idx]['point']
            
            # Compute direction to sample
            dir_x = sample[0] - nearest[0]
            dir_y = sample[1] - nearest[1]
            dist = sqrt(dir_x**2 + dir_y**2)
            
            if dist < 1e-6:
                continue
                
            # Normalize direction
            dir_x /= dist
            dir_y /= dist
            
            # Create new point by stepping toward sample
            new_point = [nearest[0] + dir_x * step_size, nearest[1] + dir_y * step_size]
            
            # Check if the path is collision-free
            if not self.is_collision_free(nearest, new_point):
                # If collision, try to find an alternative direction
                for angle_offset in [30, -30, 60, -60, 90, -90]:
                    rad_offset = radians(angle_offset)
                    new_dir_x = dir_x * cos(rad_offset) - dir_y * sin(rad_offset)
                    new_dir_y = dir_x * sin(rad_offset) + dir_y * cos(rad_offset)
                    alt_point = [nearest[0] + new_dir_x * step_size, nearest[1] + new_dir_y * step_size]
                    
                    if self.is_collision_free(nearest, alt_point):
                        new_point = alt_point
                        break
                else:
                    # If all alternatives fail, skip this iteration
                    continue
            
            # Add new point to the source tree
            new_idx = max(source_tree.keys()) + 1
            source_tree[new_idx] = {'point': new_point, 'parent': nearest_idx}
            
            # Check if we can connect to the target tree
            # Find closest node in target tree
            closest_idx = min(target_tree.keys(), key=lambda idx: 
                           (target_tree[idx]['point'][0] - new_point[0])**2 + 
                           (target_tree[idx]['point'][1] - new_point[1])**2)
            closest = target_tree[closest_idx]['point']
            
            # Check if we can connect the trees
            if (Choreographer.distance(new_point, closest) < step_size * 2 and 
                self.is_collision_free(new_point, closest)):
                # We found a path! Reconstruct it
                if is_start_tree:
                    # Path from start to new_point
                    forward_path = []
                    current_idx = new_idx
                    while current_idx is not None:
                        forward_path.append(source_tree[current_idx]['point'])
                        current_idx = source_tree[current_idx]['parent']
                    forward_path.reverse()
                    
                    # Path from closest to goal
                    backward_path = []
                    current_idx = closest_idx
                    while current_idx is not None:
                        backward_path.append(target_tree[current_idx]['point'])
                        current_idx = target_tree[current_idx]['parent']
                    
                    # Combine paths
                    complete_path = forward_path + backward_path
                else:
                    # Path from start to closest
                    forward_path = []
                    current_idx = closest_idx
                    while current_idx is not None:
                        forward_path.append(target_tree[current_idx]['point'])
                        current_idx = target_tree[current_idx]['parent']
                    forward_path.reverse()
                    
                    # Path from new_point to goal
                    backward_path = []
                    current_idx = new_idx
                    while current_idx is not None:
                        backward_path.append(source_tree[current_idx]['point'])
                        current_idx = source_tree[current_idx]['parent']
                    
                    # Combine paths
                    complete_path = forward_path + backward_path
                
                # Simplify the path
                print(f"Path found! Length before simplification: {len(complete_path)}")
                simplified_path = self.simplify_path(complete_path)
                print(f"Path length after simplification: {len(simplified_path)}")
                return simplified_path
        
        # If RRT fails, fall back to a simple zigzag path
        print("RRT failed to find a path, trying zigzag path...")
        
        zigzag_path = [self.start]
        
        # Create zigzag pattern
        direct_vector = [self.end[0] - self.start[0], self.end[1] - self.start[1]]
        direct_distance = sqrt(direct_vector[0]**2 + direct_vector[1]**2)
        
        # Normalize direction vector
        if direct_distance > 0:
            direct_vector = [direct_vector[0]/direct_distance, direct_vector[1]/direct_distance]
        
        # Perpendicular vector for zigzag
        perp_vector = [-direct_vector[1], direct_vector[0]]
        
        # Create zigzag points
        num_zigzags = 5
        zigzag_amplitude = 2.0
        for i in range(1, num_zigzags+1):
            # Position along direct path
            t = i / (num_zigzags + 1)
            mid_point = [
                self.start[0] + t * (self.end[0] - self.start[0]),
                self.start[1] + t * (self.end[1] - self.start[1])
            ]
            
            # Add zigzag offset
            if i % 2 == 0:
                offset = 1.0
            else:
                offset = -1.0
                
            zigzag_point = [
                mid_point[0] + offset * zigzag_amplitude * perp_vector[0],
                mid_point[1] + offset * zigzag_amplitude * perp_vector[1]
            ]
            
            zigzag_path.append(zigzag_point)
        
        zigzag_path.append(self.end)
        
        # Check if zigzag path is collision-free
        is_valid = True
        for i in range(len(zigzag_path) - 1):
            if not self.is_collision_free(zigzag_path[i], zigzag_path[i+1]):
                is_valid = False
                break
                
        if is_valid:
            print("Found valid zigzag path")
            return zigzag_path
            
        # Last resort - manual waypoints
        print("WARNING: Could not find a path. Creating a manual path.")
        # Create a simple path with waypoints far from obstacles
        x_diff = self.end[0] - self.start[0]
        y_diff = self.end[1] - self.start[1]
        
        # Go around the stage center to avoid central obstacles
        detour_path = [
            self.start,
            [self.start[0] + x_diff * 0.25, self.start[1] + y_diff * 0.25 + 3.0],
            [self.start[0] + x_diff * 0.5, self.start[1] + y_diff * 0.5 + 4.0],
            [self.start[0] + x_diff * 0.75, self.start[1] + y_diff * 0.75 + 3.0],
            self.end
        ]
        
        print("Created manual detour path")
        return detour_path
    
    def simplify_path(self, path):
        """Simplify a path by removing unnecessary waypoints
        
        Args:
            path: List of waypoints
            
        Returns:
            Simplified path
        """
        if len(path) <= 2:
            return path
            
        simplified = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Try to connect to the furthest possible point
            for j in range(len(path) - 1, i, -1):
                if self.is_collision_free(path[i], path[j]):
                    simplified.append(path[j])
                    i = j
                    break
            i += 1
                
        return simplified

class ForestDrone(DroneInterface):
    """Drone Interface extended for forest traversal"""

    def __init__(self, namespace: str, verbose: bool = False, use_sim_time: bool = False):
        super().__init__(namespace, verbose=verbose, use_sim_time=use_sim_time)

        self.__speed = MAX_SPEED
        self.__yaw_mode = YawMode.PATH_FACING
        self.__yaw_angle = None
        self.__frame_id = "earth"

        self.current_behavior: Optional[BehaviorHandler] = None
        self.led_pub = self.create_publisher(ColorRGBA, f"/{namespace}/leds/control", 10)
        
        # Current position
        self.current_position = [0.0, 0.0, 0.0]

    def change_led_colour(self, colour):
        """Change the colours

        Args:
            colour (tuple): The LED RGB Colours (0-255)
        """
        msg = ColorRGBA()
        msg.r = colour[0]/255.0
        msg.g = colour[1]/255.0
        msg.b = colour[2]/255.0
        self.led_pub.publish(msg)

    def do_behavior(self, beh, *args) -> None:
        """Start behavior and save current to check if finished or not"""
        self.current_behavior = getattr(self, beh)
        self.current_behavior(*args)

    def go_to_position(self, x, y, z) -> None:
        """Go to a position
        
        Args:
            x: X coordinate
            y: Y coordinate
            z: Z coordinate
        """
        self.do_behavior("go_to", x, y, z, self.__speed,
                         self.__yaw_mode, self.__yaw_angle, self.__frame_id, False)
        # Update current position
        self.current_position = [x, y, z]

    def goal_reached(self) -> bool:
        """Check if current behavior has finished"""
        if not self.current_behavior:
            return False

        if self.current_behavior.status == BehaviorStatus.IDLE:
            return True
        return False
        
    def get_position(self):
        """Get current position"""
        return self.current_position

class ForestSwarm:
    """Forest Traversal Swarm"""

    def __init__(self, drones_ns: List[str], config_file: str, verbose: bool = False, use_sim_time: bool = False):
        """Initialize the forest swarm
        
        Args:
            drones_ns: List of drone namespaces
            config_file: Path to the configuration file
            verbose: Verbose flag
            use_sim_time: Use simulation time flag
        """
        self.drones: Dict[int, ForestDrone] = {}
        for index, name in enumerate(drones_ns):
            self.drones[index] = ForestDrone(name, verbose, use_sim_time)
            
        self.num_drones = len(drones_ns)
        self.load_config(config_file)
        
        # Print information about the environment
        print(f"Stage center: {self.stage_center}")
        print(f"Start point (relative): {self.start_point}")
        print(f"End point (relative): {self.end_point}")
        print(f"Number of obstacles: {len(self.obstacles)}")
        
        # Reduce safe distance if finding path is difficult
        # For initial path planning, we can use a smaller distance
        # and then adjust the formation more carefully later
        initial_safe_distance = SAFE_DISTANCE * 0.8  # Slightly smaller for path finding
        
        # Initialize path planner
        self.path_planner = PathPlanner(
            self.obstacles,
            self.start_point,
            self.end_point,
            self.stage_center,
            safe_distance=initial_safe_distance
        )
        
        # Find a path for the center of the formation
        self.path = self.path_planner.find_path_rrt()
        
        # If path is still None, create a very simple direct path and handle obstacles during execution
        if self.path is None:
            print("WARNING: Could not find path with RRT. Creating a simple direct path.")
            abs_start = [self.start_point[0] + self.stage_center[0], self.start_point[1] + self.stage_center[1]]
            abs_end = [self.end_point[0] + self.stage_center[0], self.end_point[1] + self.stage_center[1]]
            
            # Create a path with intermediate points
            mid_x = (abs_start[0] + abs_end[0]) / 2
            mid_y = (abs_start[1] + abs_end[1]) / 2
            
            # Create points for a wide detour
            self.path = [
                abs_start,
                [mid_x, mid_y + 3.0],  # Go north to avoid potential obstacles
                abs_end
            ]
            
        print(f"Path found with {len(self.path)} waypoints: {self.path}")
        
        # Current formation
        self.current_formation = "column"
        
        # Flight height
        self.flight_height = 1.5
        
    def load_config(self, config_file):
        """Load configuration from file
        
        Args:
            config_file: Path to the configuration file
        """
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                
            # Extract stage information
            stage_data = config.get('stage3', {})
            if not stage_data:
                print(f"WARNING: 'stage3' not found in {config_file}. Looking for it directly in the root.")
                # Try to find the data directly in the root of the config
                stage_data = config
                
            self.stage_center = stage_data.get('stage_center', [0.0, -6.0])  # Default: [0.0, -6.0]
            self.start_point = stage_data.get('start_point', [-4.0, 0.0])    # Default: [-4.0, 0.0]
            self.end_point = stage_data.get('end_point', [4.0, 0.0])         # Default: [4.0, 0.0]
            
            # Get obstacles - if none found, create a small test set
            self.obstacles = stage_data.get('obstacles', [])
            if not self.obstacles:
                print("WARNING: No obstacles found in config. Using default obstacle configuration.")
                # Create some default obstacles
                self.obstacles = [
                    [2.0, 0.0],
                    [1.0, -3.0],
                    [0.0, 2.0],
                    [-1.0, -3.0],
                    [-2.0, 2.0]
                ]
                
            self.obstacle_height = stage_data.get('obstacle_height', 5.0)
            self.obstacle_diameter = stage_data.get('obstacle_diameter', 0.4)
            
            print(f"Configuration loaded from {config_file}")
            
        except Exception as e:
            print(f"ERROR loading configuration: {e}")
            print("Using default configuration")
            
            # Set default values
            self.stage_center = [0.0, -6.0]
            self.start_point = [-4.0, 0.0]
            self.end_point = [4.0, 0.0]
            self.obstacles = [
                [2.0, 0.0],
                [1.0, -3.0],
                [0.0, 2.0],
                [-1.0, -3.0],
                [-2.0, 2.0]
            ]
            self.obstacle_height = 5.0
            self.obstacle_diameter = 0.4

    def shutdown(self):
        """Shutdown all drones in swarm"""
        for drone in self.drones.values():
            drone.shutdown()

    def wait(self):
        """Wait until all drones has reached their goal"""
        all_finished = False
        while not all_finished:
            all_finished = True
            for drone in self.drones.values():
                all_finished = all_finished and drone.goal_reached()
                
            # Small sleep to avoid CPU hogging
            time.sleep(0.1)

    def get_ready(self) -> bool:
        """Arm and offboard for all drones in swarm"""
        success = True
        for drone in self.drones.values():
            # Arm
            success_arm = drone.arm()

            # Offboard
            success_offboard = drone.offboard()
            success = success and success_arm and success_offboard
        return success

    def takeoff(self):
        """Takeoff swarm and wait for all drones"""
        for drone in self.drones.values():
            drone.do_behavior("takeoff", self.flight_height, 0.7, False)
            drone.change_led_colour((0, 255, 0))  # Green during takeoff
        self.wait()

    def land(self):
        """Land swarm and wait for all drones"""
        for drone in self.drones.values():
            drone.do_behavior("land", 0.4, False)
        self.wait()
    
    def get_formation_waypoints(self, center, formation_type="column", orientation=0.0):
        """Get formation waypoints based on formation type and center position
        
        Args:
            center: Center position [x, y]
            formation_type: Formation type ("line", "v", "column")
            orientation: Formation orientation in degrees
            
        Returns:
            List of waypoints for formation
        """
        if formation_type == "line":
            return Choreographer.line_formation(FORMATION_DISTANCE * (self.num_drones - 1), 
                                               self.num_drones, orientation, center)
        elif formation_type == "v":
            return Choreographer.v_formation(FORMATION_DISTANCE, 
                                            self.num_drones, orientation, center)
        else:  # Default to column
            return Choreographer.column_formation(FORMATION_DISTANCE * (self.num_drones - 1), 
                                                 self.num_drones, orientation, center)
    
    def check_formation_collision(self, formation_waypoints):
        """Check if formation will collide with obstacles
        
        Args:
            formation_waypoints: List of waypoints for formation
            
        Returns:
            True if collision, False otherwise
        """
        # Check every drone position in the formation against all obstacles
        for wp in formation_waypoints:
            for obs in self.obstacles:
                # Convert obstacle to absolute position
                obs_pos = [obs[0] + self.stage_center[0], obs[1] + self.stage_center[1]]
                dist = Choreographer.distance(wp, obs_pos)
                if dist < SAFE_DISTANCE + self.obstacle_diameter/2:
                    return True
                    
        return False
    
    def get_best_formation(self, center, direction):
        """Get the best formation based on obstacles and direction
        
        Args:
            center: Center position [x, y]
            direction: Direction vector [dx, dy]
            
        Returns:
            Tuple of (formation_type, orientation, waypoints)
        """
        # Normalize direction
        direction_len = sqrt(direction[0]**2 + direction[1]**2)
        if direction_len < 1e-6:
            direction = [1.0, 0.0]  # Default to forward if no direction
        else:
            direction = [direction[0]/direction_len, direction[1]/direction_len]
            
        # Calculate orientation angle in degrees
        orientation = math.degrees(atan2(direction[1], direction[0]))
        
        # Try different formations in priority order
        formations_to_try = ["line", "v", "column"]
        
        for formation in formations_to_try:
            # For line formation, try perpendicular to movement direction first (minimizes width)
            if formation == "line":
                perp_orientation = orientation + 90
                waypoints = self.get_formation_waypoints(center, formation, perp_orientation)
                if not self.check_formation_collision(waypoints):
                    return (formation, perp_orientation, waypoints)
                    
                # Try original orientation
                waypoints = self.get_formation_waypoints(center, formation, orientation)
                if not self.check_formation_collision(waypoints):
                    return (formation, orientation, waypoints)
            else:
                # For other formations, just try with movement direction
                waypoints = self.get_formation_waypoints(center, formation, orientation)
                if not self.check_formation_collision(waypoints):
                    return (formation, orientation, waypoints)
                    
        # If all formations would collide, use column as default with original orientation
        # and adjust positions to avoid obstacles
        initial_waypoints = self.get_formation_waypoints(center, "column", orientation)
        adjusted_waypoints = Choreographer.adjust_formation_for_obstacles(
            initial_waypoints, 
            self.obstacles, 
            self.stage_center, 
            SAFE_DISTANCE + self.obstacle_diameter/2
        )
        
        return ("column", orientation, adjusted_waypoints)
    
    def check_obstacles_near_path(self, segment_start, segment_end, check_radius=OBSTACLE_CHECK_RADIUS):
        """Check if there are obstacles near a path segment
        
        Args:
            segment_start: Start point of segment [x, y]
            segment_end: End point of segment [x, y]
            check_radius: Radius around the segment to check for obstacles
            
        Returns:
            List of obstacles near the path segment
        """
        nearby_obstacles = []
        
        # Direction vector of segment
        dx = segment_end[0] - segment_start[0]
        dy = segment_end[1] - segment_start[1]
        segment_length = sqrt(dx*dx + dy*dy)
        
        if segment_length < 1e-6:
            return nearby_obstacles
            
        # Normalize direction vector
        dx /= segment_length
        dy /= segment_length
        
        for obs in self.obstacles:
            # Convert obstacle to absolute position
            obs_pos = [obs[0] + self.stage_center[0], obs[1] + self.stage_center[1]]
            
            # Vector from segment start to obstacle
            vx = obs_pos[0] - segment_start[0]
            vy = obs_pos[1] - segment_start[1]
            
            # Projection onto segment direction
            proj = vx*dx + vy*dy
            
            # Clamp projection to segment length
            proj = max(0, min(segment_length, proj))
            
            # Closest point on segment to obstacle
            closest_x = segment_start[0] + proj * dx
            closest_y = segment_start[1] + proj * dy
            
            # Distance from closest point to obstacle
            dist = Choreographer.distance([closest_x, closest_y], obs_pos)
            
            if dist < check_radius:
                nearby_obstacles.append(obs_pos)
                
        return nearby_obstacles
    
    def navigate_forest(self):
        """Navigate through the forest with intermediate waypoint and formation maintenance"""
        # Define waypoints (use the path from RRT)
        forest_waypoints = self.path.copy()

        intermediate_point = [(-5.0, -1.0), 
                               (-5.0, -2.0), 
                               (-5.0, -3.0), 
                               (-5.0, -4.0), 
                               (-5.0, -5.0)]

        # Starting formation at first waypoint
        if len(forest_waypoints) > 1:
            start_direction = [forest_waypoints[1][0] - forest_waypoints[0][0],
                            forest_waypoints[1][1] - forest_waypoints[0][1]]
        else:
            # Default direction if only one waypoint
            start_direction = [1.0, 0.0]
            
        formation_info = self.get_best_formation(forest_waypoints[0], start_direction)
        self.current_formation, orientation, waypoints = formation_info
        
        # Move drones to intermediate point
        for idx, drone in self.drones.items():
            if idx < len(intermediate_point):
                drone.go_to_position(intermediate_point[idx][0], intermediate_point[idx][1], self.flight_height)
                drone.change_led_colour((255, 255, 0))
        self.wait()
        print("Intermediate point reached, moving to forest waypoints...")

        print(f"Initial formation: {self.current_formation}, orientation: {orientation}")
        print(f"Initial waypoints: {waypoints}")
        
        # Move drones to initial formation
        for idx, drone in self.drones.items():
            if idx < len(waypoints):
                drone.go_to_position(waypoints[idx][0], waypoints[idx][1], self.flight_height)
                if self.current_formation == "line":
                    drone.change_led_colour((255, 0, 0))  # Red for line
                elif self.current_formation == "v":
                    drone.change_led_colour((0, 0, 255))  # Blue for V
                else:
                    drone.change_led_colour((255, 165, 0))  # Orange for column
        self.wait()
        
        # Navigate through waypoints with formation maintenance and obstacle avoidance
        for i in range(1, len(forest_waypoints)):
            current_point = forest_waypoints[i]
            
            # Calculate direction to next waypoint for formation orientation
            if i < len(forest_waypoints) - 1:
                direction = [forest_waypoints[i+1][0] - current_point[0],
                            forest_waypoints[i+1][1] - current_point[1]]
            else:
                # Use same direction for the last waypoint
                direction = [current_point[0] - forest_waypoints[i-1][0],
                            current_point[1] - forest_waypoints[i-1][1]]
            
            # Check for obstacles near the path segment we're about to traverse
            prev_point = forest_waypoints[i-1]
            nearby_obstacles = self.check_obstacles_near_path(prev_point, current_point)
            
            print(f"Moving to waypoint {i} at {current_point}, found {len(nearby_obstacles)} nearby obstacles")
            
            # If obstacles are detected, we need to adjust our formation
            if nearby_obstacles:
                # Determine best formation for current waypoint and obstacles in vicinity
                formation_info = self.get_best_formation(current_point, direction)
                new_formation, new_orientation, formation_positions = formation_info
                
                # Check if formation has changed
                formation_changed = (new_formation != self.current_formation)
                self.current_formation = new_formation
                
                print(f"Adjusted formation due to obstacles: {self.current_formation}, orientation: {new_orientation}")
            else:
                # No obstacles nearby, maintain current formation
                formation_positions = self.get_formation_waypoints(current_point, self.current_formation, 
                                                                  math.degrees(atan2(direction[1], direction[0])))
                formation_changed = False
            
            # Check if any drone position is too close to an obstacle and adjust if needed
            adjusted_positions = Choreographer.adjust_formation_for_obstacles(
                formation_positions,
                self.obstacles,
                self.stage_center,
                SAFE_DISTANCE + self.obstacle_diameter/2
            )
            
            # Move drones to new positions while maintaining formation
            for idx, drone in self.drones.items():
                if idx < len(adjusted_positions):
                    drone.go_to_position(adjusted_positions[idx][0], 
                                      adjusted_positions[idx][1], 
                                      self.flight_height)
                    
                    # Update LED color if formation changed
                    if formation_changed:
                        if self.current_formation == "line":
                            drone.change_led_colour((255, 0, 0))  # Red for line
                        elif self.current_formation == "v":
                            drone.change_led_colour((0, 0, 255))  # Blue for V
                        else:
                            drone.change_led_colour((255, 165, 0))  # Orange for column
            
            # Wait for all drones to reach their positions before proceeding
            self.wait()
            
            # Log the waypoint completion
            print(f"Reached waypoint {i} at {current_point}")
        
        print("Forest traversal complete!")
        
    def perform_dynamic_obstacle_avoidance(self):
        """Continuously monitor for obstacles and adjust formation in real-time"""
        # This could be implemented in a separate thread or process
        # For a comprehensive solution, this would monitor drone positions and
        # obstacles continuously and send commands for avoidance
        pass

def confirm(msg: str = 'Continue') -> bool:
    """Confirm message"""
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    return False

def main():
    parser = argparse.ArgumentParser(
        description='Forest traversal mission')

    parser.add_argument('-n', '--namespaces',
                        type=str, nargs='+',
                        default=['drone0', 'drone1', 'drone2', 'drone3', 'drone4'],
                        help='Namespace of the drones')
    parser.add_argument('-c', '--config',
                        type=str,
                        default='scenarios/scenario1_stage3.yaml',
                        help='Path to the configuration file')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    drones_namespace = args.namespaces
    config_file = args.config
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    rclpy.init()
    swarm = ForestSwarm(
        drones_namespace,
        config_file,
        verbose=verbosity,
        use_sim_time=use_sim_time)

    print("Start forest traversal mission")
    swarm.get_ready()
    swarm.takeoff()

    print("Navigate through the forest")
    swarm.navigate_forest()

    confirm("Land")
    swarm.land()

    print("Shutdown")
    swarm.shutdown()
    rclpy.shutdown()

    sys.exit(0)

if __name__ == '__main__':
    main()