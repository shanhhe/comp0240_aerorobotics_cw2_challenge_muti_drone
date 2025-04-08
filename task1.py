#!/usr/bin/env python3
 
"""
Centralized formation flight controller using Virtual Structure approach.
This implementation handles Stage 1: Changing Formations while following a circular trajectory.
"""
 
import argparse
import sys
import time
import yaml
import math
import numpy as np
from typing import List, Dict, Optional, Tuple
import rclpy
from rclpy.node import Node
from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from std_msgs.msg import ColorRGBA
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
 
# ./launch_as2.bash -s scenarios/scenario1_stage1.yaml -w config_sim/config/world_swarm.yaml

class FormationPatterns:
    """Class to generate various formation patterns for the drone swarm"""
    
    @staticmethod
    def line_formation(num_drones: int, line_length: float, center: List[float], height: float) -> List[List[float]]:
        """Generate a line formation with specified drones"""
        positions = []
        if num_drones == 1:
            return [[center[0], center[1], height]]
            
        for i in range(num_drones):
            x = center[0] + line_length * (i - (num_drones - 1) / 2) / (num_drones - 1)
            positions.append([x, center[1], height])
        return positions
    
    @staticmethod
    def v_formation(num_drones: int, v_distance: float, v_angle: float, center: List[float], height: float) -> List[List[float]]:
        """Generate a V formation with specified drones"""
        positions = []
        
        # Leader at apex position
        positions.append([center[0], center[1], height])
        
        # If only one drone, return just the leader position
        if num_drones == 1:
            return positions
            
        # Calculate angle in radians
        angle_rad = math.radians(v_angle / 2)
        
        # Add drones to each wing
        left_wing = (num_drones - 1) // 2
        right_wing = num_drones - 1 - left_wing
        
        # Place drones on the left wing
        for i in range(1, left_wing + 1):
            x = center[0] - i * v_distance * math.sin(angle_rad)
            y = center[1] - i * v_distance * math.cos(angle_rad)
            positions.append([x, y, height])
        
        # Place drones on the right wing
        for i in range(1, right_wing + 1):
            x = center[0] + i * v_distance * math.sin(angle_rad)
            y = center[1] - i * v_distance * math.cos(angle_rad)
            positions.append([x, y, height])
            
        return positions
    
    @staticmethod
    def square_formation(num_drones: int, side_length: float, center: List[float], height: float) -> List[List[float]]:
        """Generate a square formation with specified drones"""
        positions = []
        
        # If fewer than 4 drones, make a partial square
        if num_drones <= 4:
            # Four corners of the square
            corner_positions = [
                [center[0] - side_length/2, center[1] - side_length/2, height],  # Bottom-left
                [center[0] + side_length/2, center[1] - side_length/2, height],  # Bottom-right
                [center[0] + side_length/2, center[1] + side_length/2, height],  # Top-right
                [center[0] - side_length/2, center[1] + side_length/2, height],  # Top-left
            ]
            return corner_positions[:num_drones]
        
        # For more than 4 drones, distribute them evenly along the perimeter
        perimeter = 4 * side_length
        segment_length = perimeter / num_drones
        
        for i in range(num_drones):
            dist = i * segment_length
            
            # Determine which side of the square we're on
            if dist < side_length:  # Bottom side
                x = center[0] - side_length/2 + dist
                y = center[1] - side_length/2
            elif dist < 2 * side_length:  # Right side
                x = center[0] + side_length/2
                y = center[1] - side_length/2 + (dist - side_length)
            elif dist < 3 * side_length:  # Top side
                x = center[0] + side_length/2 - (dist - 2 * side_length)
                y = center[1] + side_length/2
            else:  # Left side
                x = center[0] - side_length/2
                y = center[1] + side_length/2 - (dist - 3 * side_length)
                
            positions.append([x, y, height])
            
        return positions
    
    @staticmethod
    def orbit_formation(num_drones: int, radius: float, center: List[float], height: float) -> List[List[float]]:
        """Generate a circular orbit formation with specified drones"""
        positions = []
        
        angle_step = 2 * math.pi / num_drones
        
        for i in range(num_drones):
            angle = i * angle_step
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            positions.append([x, y, height])
            
        return positions
    
    @staticmethod
    def grid_formation(num_drones: int, grid_spacing: float, center: List[float], height: float) -> List[List[float]]:
        """Generate a grid formation with specified drones"""
        positions = []
        
        # Calculate grid dimensions
        grid_side = math.ceil(math.sqrt(num_drones))
        
        for i in range(num_drones):
            row = i // grid_side
            col = i % grid_side
            
            x = center[0] + (col - (grid_side - 1) / 2) * grid_spacing
            y = center[1] + (row - (grid_side - 1) / 2) * grid_spacing
            
            positions.append([x, y, height])
            
        return positions
    
    @staticmethod
    def staggered_formation(num_drones: int, spacing: float, center: List[float], height: float) -> List[List[float]]:
        """Generate a staggered formation with specified drones"""
        positions = []
        
        # Calculate number of rows and columns
        num_cols = math.ceil(math.sqrt(num_drones))
        num_rows = math.ceil(num_drones / num_cols)
        
        for i in range(num_drones):
            row = i // num_cols
            col = i % num_cols
            
            # Apply staggering - offset every other row
            offset = (spacing / 2) if row % 2 else 0
            
            x = center[0] + (col - (num_cols - 1) / 2) * spacing + offset
            y = center[1] + (row - (num_rows - 1) / 2) * spacing
            
            positions.append([x, y, height])
            
        return positions
    
    @staticmethod
    def column_formation(num_drones: int, spacing: float, center: List[float], height: float) -> List[List[float]]:
        """Generate a column formation with specified drones"""
        positions = []
        
        for i in range(num_drones):
            y = center[1] + (i - (num_drones - 1) / 2) * spacing
            positions.append([center[0], y, height])
            
        return positions
 
 
class FormationController:
    """Centralized formation controller using virtual structure approach"""
    
    def __init__(self, formation_type: str, num_drones: int, config: Dict):
        """Initialize the formation controller"""
        self.formation_type = formation_type
        self.num_drones = num_drones
        self.config = config
        
        # Default parameters
        self.line_length = 2.0
        self.v_distance = 1.0
        self.v_angle = 60.0
        self.square_side = 2.0
        self.orbit_radius = 1.0
        self.grid_spacing = 1.0
        self.staggered_spacing = 1.0
        self.column_spacing = 1.0
        self.formation_height = 2.0
        
        # Load parameters from config if available
        if config and 'stage1' in config:
            if 'trajectory' in config['stage1']:
                if 'diameter' in config['stage1']['trajectory']:
                    # Adjust formation sizes based on trajectory diameter
                    diameter = config['stage1']['trajectory']['diameter']
                    self.line_length = diameter * 0.7
                    self.v_distance = diameter * 0.3
                    self.square_side = diameter * 0.5
                    self.orbit_radius = diameter * 0.4
                    self.grid_spacing = diameter * 0.3
                    self.staggered_spacing = diameter * 0.3
                    self.column_spacing = diameter * 0.3
        
    def get_formation_positions(self, center: List[float], height: float = None) -> List[List[float]]:
        """Generate drone positions based on the current formation type"""
        if height is None:
            height = self.formation_height
            
        if self.formation_type == "line":
            return FormationPatterns.line_formation(self.num_drones, self.line_length, center, height)
        elif self.formation_type == "v":
            return FormationPatterns.v_formation(self.num_drones, self.v_distance, self.v_angle, center, height)
        elif self.formation_type == "square":
            return FormationPatterns.square_formation(self.num_drones, self.square_side, center, height)
        elif self.formation_type == "orbit":
            return FormationPatterns.orbit_formation(self.num_drones, self.orbit_radius, center, height)
        elif self.formation_type == "grid":
            return FormationPatterns.grid_formation(self.num_drones, self.grid_spacing, center, height)
        elif self.formation_type == "staggered":
            return FormationPatterns.staggered_formation(self.num_drones, self.staggered_spacing, center, height)
        elif self.formation_type == "columnN":
            return FormationPatterns.column_formation(self.num_drones, self.column_spacing, center, height)
        else:
            # Default to orbit if formation not recognized
            return FormationPatterns.orbit_formation(self.num_drones, self.orbit_radius, center, height)
    
    def generate_circular_trajectory(self, center: List[float], radius: float, num_points: int, height: float) -> List[List[float]]:
        """Generate points for a circular trajectory around the center point"""
        trajectory = []
        
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            trajectory.append([x, y, height])
            
        return trajectory
    
    def calculate_transformed_formation(self, trajectory_point: List[float],
                                        next_point: List[float],
                                        formation_positions: List[List[float]]) -> List[List[float]]:
        """
        Transform the formation based on the trajectory's direction.
        This aligns the formation to face the direction of travel.
        """
        # Calculate the direction vector of travel
        direction_vector = [next_point[0] - trajectory_point[0],
                            next_point[1] - trajectory_point[1]]
        
        # Normalize the direction vector
        direction_length = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
        if direction_length > 0:
            direction_vector = [direction_vector[0]/direction_length,
                                direction_vector[1]/direction_length]
        else:
            direction_vector = [1, 0]  # Default direction if stationary
        
        # Calculate the rotation angle from the default formation orientation
        # Assume formations by default face the positive x-axis
        rotation_angle = math.atan2(direction_vector[1], direction_vector[0])
        
        # Translate and rotate each drone's position in the formation
        transformed_positions = []
        
        for pos in formation_positions:
            # Get the relative position from the center of the formation
            rel_x = pos[0] - trajectory_point[0]
            rel_y = pos[1] - trajectory_point[1]
            
            # Apply rotation to the relative position
            rotated_x = rel_x * math.cos(rotation_angle) - rel_y * math.sin(rotation_angle)
            rotated_y = rel_x * math.sin(rotation_angle) + rel_y * math.cos(rotation_angle)
            
            # Translate back to global coordinates
            final_x = trajectory_point[0] + rotated_x
            final_y = trajectory_point[1] + rotated_y
            
            transformed_positions.append([final_x, final_y, pos[2]])
            
        return transformed_positions
 
 
class Drone(DroneInterface):
    """Drone interface with additional methods for formation flight"""
    
    def __init__(self, namespace: str, verbose: bool = False, use_sim_time: bool = False):
        super().__init__(namespace, verbose=verbose, use_sim_time=use_sim_time)
        
        self.current_behavior: Optional[BehaviorHandler] = None
        self.led_pub = self.create_publisher(ColorRGBA, f"/{namespace}/leds/control", 10)
        
        # Default parameters
        self.default_speed = 0.5
        self.yaw_mode = YawMode.PATH_FACING
        self.frame_id = "earth"
    
    def change_led_color(self, color: List[int]) -> None:
        """Change LED color of the drone"""
        msg = ColorRGBA()
        msg.r = color[0] / 255.0
        msg.g = color[1] / 255.0
        msg.b = color[2] / 255.0
        if len(color) > 3:
            msg.a = color[3] / 255.0
        else:
            msg.a = 1.0
        self.led_pub.publish(msg)
    
    def do_behavior(self, behavior_name: str, *args) -> None:
        """Start a behavior and save it as current behavior"""
        self.current_behavior = getattr(self, behavior_name)
        self.current_behavior(*args)
    
    def go_to_position(self, position: List[float], speed: float = None) -> None:
        """Go to a specified position"""
        if speed is None:
            speed = self.default_speed
            
        self.do_behavior(
            "go_to",
            position[0], position[1], position[2],
            speed,
            self.yaw_mode,
            None,  # yaw_angle
            self.frame_id,
            False  # wait_for_completion
        )
    
    def is_goal_reached(self) -> bool:
        """Check if the current behavior has completed"""
        if not self.current_behavior:
            return False
            
        return self.current_behavior.status == BehaviorStatus.IDLE
 
 
class CentralizedSwarmController:
    """Main centralized controller for the drone swarm"""
    
    def __init__(self, drone_namespaces: List[str], config_file: str = None,
                 verbose: bool = False, use_sim_time: bool = True):
        """Initialize the centralized swarm controller"""
        
        # Load configuration from YAML file if provided
        self.config = None
        if config_file:
            try:
                with open(config_file, 'r') as file:
                    self.config = yaml.safe_load(file)
            except Exception as e:
                print(f"Error loading config file: {e}")
                self.config = {}
        
        # Initialize drones
        self.drones = {}
        for i, namespace in enumerate(drone_namespaces):
            self.drones[i] = Drone(namespace, verbose, use_sim_time)
        
        # Get formation types from config
        self.formation_types = ["line", "v", "square", "orbit", "grid", "staggered", "columnN"]
        if self.config and 'stage1' in self.config and 'formations' in self.config['stage1']:
            self.formation_types = self.config['stage1']['formations']
        
        # Get trajectory parameters
        self.trajectory_diameter = 3.0
        self.stage_center = [0.0, 0.0]
        if self.config and 'stage1' in self.config:
            if 'trajectory' in self.config['stage1'] and 'diameter' in self.config['stage1']['trajectory']:
                self.trajectory_diameter = self.config['stage1']['trajectory']['diameter']
            if 'stage_center' in self.config['stage1']:
                self.stage_center = self.config['stage1']['stage_center']
        
        # Initialize formation controller
        self.current_formation_index = 0
        self.current_formation = self.formation_types[self.current_formation_index]
        self.formation_controller = FormationController(
            self.current_formation,
            len(self.drones),
            self.config
        )
        
        # Movement parameters
        self.flight_height = 1.5
        self.circular_trajectory = None
        self.trajectory_radius = self.trajectory_diameter / 2
        self.waypoint_index = 0
        self.num_trajectory_points = 20  # Number of points on the circular trajectory
        
        # Generate the circular trajectory once
        self.generate_trajectory()
        
        # Colors for different formations
        self.formation_colors = {
            "line": [255, 0, 0],        # Red
            "v": [0, 255, 0],           # Green
            "square": [0, 0, 255],      # Blue
            "orbit": [255, 255, 0],     # Yellow
            "grid": [255, 0, 255],      # Magenta
            "staggered": [0, 255, 255], # Cyan
            "columnN": [255, 165, 0],   # Orange
            "free": [255, 255, 255]     # White
        }
    
    def generate_trajectory(self) -> None:
        """Generate the circular trajectory"""
        self.circular_trajectory = self.formation_controller.generate_circular_trajectory(
            self.stage_center,
            self.trajectory_radius,
            self.num_trajectory_points,
            self.flight_height
        )
    
    def change_formation(self) -> None:
        """Change to the next formation type"""
        self.current_formation_index = (self.current_formation_index + 1) % len(self.formation_types)
        self.current_formation = self.formation_types[self.current_formation_index]
        self.formation_controller.formation_type = self.current_formation
        
        # Update drone LED colors based on formation
        color = self.formation_colors.get(self.current_formation, [255, 255, 255])
        for drone in self.drones.values():
            drone.change_led_color(color)
            
        print(f"Changed formation to: {self.current_formation}")
    
    def shutdown(self) -> None:
        """Shut down all drones in the swarm"""
        for drone in self.drones.values():
            drone.shutdown()
    
    def get_ready(self) -> bool:
        """Arm and set to offboard mode for all drones"""
        success = True
        for drone in self.drones.values():
            success_arm = drone.arm()
            success_offboard = drone.offboard()
            success = success and success_arm and success_offboard
        return success
    
    def takeoff(self) -> None:
        """Take off all drones and wait until complete"""
        for drone in self.drones.values():
            drone.do_behavior("takeoff", self.flight_height, 0.7, False)
            drone.change_led_color([0, 255, 0])
        
        self.wait_for_completion()
    
    def land(self) -> None:
        """Land all drones and wait until complete"""
        for drone in self.drones.values():
            drone.do_behavior("land", 0.4, False)
        
        self.wait_for_completion()
    
    def wait_for_completion(self) -> None:
        """Wait until all drones have completed their current behaviors"""
        all_complete = False
        while not all_complete:
            all_complete = True
            for drone in self.drones.values():
                all_complete = all_complete and drone.is_goal_reached()
            time.sleep(0.1)  # Small sleep to prevent CPU hogging
    
    def move_to_next_waypoint(self) -> None:
        """Move the swarm to the next waypoint on the circular trajectory"""
        current_point = self.circular_trajectory[self.waypoint_index]
        next_point_index = (self.waypoint_index + 1) % len(self.circular_trajectory)
        next_point = self.circular_trajectory[next_point_index]
        
        # Get base formation positions
        formation_positions = self.formation_controller.get_formation_positions(
            current_point,
            self.flight_height
        )
        
        # Transform formation to align with trajectory direction
        transformed_positions = self.formation_controller.calculate_transformed_formation(
            current_point,
            next_point,
            formation_positions
        )
        
        # Send each drone to its position in the formation
        for i, drone in enumerate(self.drones.values()):
            if i < len(transformed_positions):
                drone.go_to_position(transformed_positions[i])
        
        # Update waypoint index for next time
        self.waypoint_index = next_point_index
    
    def perform_circular_trajectory(self, num_rotations: int = 1,
                                   formation_changes: int = 5) -> None:
        """
        Perform a circular trajectory with specified number of rotations
        and formation changes.
        """
        total_waypoints = num_rotations * len(self.circular_trajectory)
        changes_every = total_waypoints // formation_changes if formation_changes > 0 else total_waypoints + 1
        
        for i in range(total_waypoints):
            self.move_to_next_waypoint()
            self.wait_for_completion()
            
            # Change formation periodically
            if i > 0 and i % changes_every == 0:
                self.change_formation()
 
 
def load_scenario_config(file_path: str) -> Dict:
    """Load scenario configuration from a YAML file"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading scenario file: {e}")
        return {}
 
 
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Centralized Virtual Structure Formation Control')
    
    parser.add_argument('-n', '--namespaces', nargs='+',
                        default=['drone0', 'drone1', 'drone2', 'drone3', 'drone4'],
                        help='Namespaces of the drones to be used')
    parser.add_argument('-s', '--scenario', type=str,
                        default='scenarios/scenario1_stage1.yaml',
                        help='Scenario configuration file')
    parser.add_argument('-v', '--verbose', action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-t', '--use_sim_time', action='store_true',
                        default=True,
                        help='Use simulation time')
    
    args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init()
    
    # Load scenario configuration
    config = load_scenario_config(args.scenario)
    
    # Create centralized controller
    controller = CentralizedSwarmController(
        args.namespaces,
        args.scenario,
        verbose=args.verbose,
        use_sim_time=args.use_sim_time
    )
    
    try:
        # Start the mission
        print("Preparing for takeoff. Press Enter to continue...")
        input()
        
        if controller.get_ready():
            controller.takeoff()
            
            print("Starting circular trajectory with formation changes...")
            controller.perform_circular_trajectory(num_rotations=2, formation_changes=6)
            
            print("Landing...")
            controller.land()
        else:
            print("Failed to prepare drones!")
    
    except KeyboardInterrupt:
        print("\nMission interrupted by user")
    
    finally:
        # Ensure proper shutdown
        print("Shutting down...")
        controller.shutdown()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()