#!/usr/bin/env python3

"""Window traversal mission for a swarm of drones with direct approach."""

__authors__ = 'Rafael Perez-Segui, Miguel Fernandez-Cortizas'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import sys
import yaml
import time
import os
from typing import List, Optional, Dict, Tuple
from math import radians, cos, sin
import random
import rclpy
from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler

from std_msgs.msg import ColorRGBA


class WindowInfo:
    """Store window information from configuration file"""
    
    def __init__(self, window_id, config, stage_center):
        self.id = window_id
        # Important: the coordinates in the YAML are [y, x], so we need to flip them
        self.center_x, self.center_y = config['center']
        # Adjust coordinates relative to stage center
        self.center_x += stage_center[0]
        self.center_y += stage_center[1]
        self.gap_width = config['gap_width']
        self.distance_floor = config['distance_floor']
        self.height = config['height']
        self.thickness = config['thickness']
        
        # Calculate entrance and exit points
        # Since the x-coordinate in the YAML is actually along the y-axis in the world frame,
        # we need to set up our waypoints accordingly
        self.entrance_y = self.center_y - self.thickness/2
        self.exit_y = self.center_y + self.thickness/2
        
        # Z coordinates for top and bottom of window
        self.bottom_z = self.distance_floor
        self.top_z = self.distance_floor + self.height
        
        # Center Z coordinate
        self.center_z = self.bottom_z + self.height/2
        
        # Safety margin to avoid collisions with walls
        self.safety_margin = 0.3
        
        print(f"Window {window_id} info:")
        print(f"  Center: ({self.center_x}, {self.center_y}, {self.center_z})")
        print(f"  Gap width: {self.gap_width}m, Height: {self.height}m")
        print(f"  Entrance Y: {self.entrance_y}, Exit Y: {self.exit_y}")
        
    def get_line_formation_positions(self, num_drones, approach_distance=0.5):
        """Calculate positions for line formation before and after window"""
        # For a line formation perpendicular to the window
        # We'll space drones along the x-axis (perpendicular to window)
        
        # Calculate available width considering safety margins
        available_width = self.gap_width - 2 * self.safety_margin
        
        # If we can't fit all drones side by side with safety margins, stack them vertically
        # if num_drones > 1 and available_width / num_drones < 0.4:  # 0.4m minimum space between drones
        #     return self.get_vertical_line_formation(num_drones, approach_distance)
        
        # Otherwise, create a horizontal line formation
        if num_drones == 1:
            # If only one drone, place it at the center
            x_positions = [self.center_x]
        else:
            # Space drones evenly within the available width
            spacing = available_width / (num_drones - 1) if num_drones > 1 else 0
            start_x = self.center_x - available_width/2
            x_positions = [start_x + i * spacing for i in range(num_drones)]
        
        # Entry points (before window)
        entry_points = [(x, self.entrance_y - approach_distance, self.center_z) for x in x_positions]
        
        # Window passage points
        window_points = [(x, self.entrance_y, self.center_z) for x in x_positions]
        
        # Exit points (after window)
        exit_points = [(x, self.exit_y + approach_distance, self.center_z) for x in x_positions]
        
        return entry_points, window_points, exit_points
    
    def get_square_formation_positions(self, num_drones, approach_distance=0.9):
        """Calculate positions for square formation before and after window"""
        # Square formation perpendicular to the window

        x = self.center_x
        y = self.center_y
        z = self.center_z

        available_height = self.height - 2 * self.safety_margin

        # Generate up to 5 positions (center + 4 corners of a square)
        positions = [
            (x, y, z),  # Center
            (x - available_height / 2, y, z - available_height / 2),  # Bottom-left
            (x - available_height / 2, y, z + available_height / 2),  # Bottom-right
            (x + available_height / 2, y, z - available_height / 2),  # Top-left
            (x + available_height / 2, y, z + available_height / 2),  # Top-right
        ]
        
        # Truncate to the number of drones
        positions = positions[:num_drones]

        # Three-stage positions
        entry_points = [(x, y + approach_distance, z) for x, y, z in positions]
        window_points = positions
        exit_points = [(x, y - approach_distance, z) for x, y, z in positions]

        return entry_points, window_points, exit_points

        
    def get_single_file_formation(self, num_drones, approach_distance=0.5):
        """Create a single-file formation to pass through very narrow gaps"""
        # All drones will pass through the center of the window
        x_position = self.center_x
        z_position = self.center_z
        
        # Entry points - staggered along the approach axis
        entry_points = []
        for i in range(num_drones):
            # Space drones along the approach path with 0.5m between them
            y_offset = approach_distance + (num_drones - i - 1) * 0.5
            entry_points.append((x_position, self.entrance_y - 1.0 - y_offset, z_position))
        
        # Window passage points - all at the same position but will be reached sequentially
        window_points = [(x_position, self.entrance_y - (num_drones - i - 1) * 0.5, z_position) for i in range(num_drones)]
        
        # Exit points - staggered along the exit axis
        exit_points = []
        for i in range(num_drones):
            # Space drones along the exit path with 0.5m between them
            y_offset = approach_distance + i * 0.5
            exit_points.append((x_position, self.exit_y + y_offset, z_position))
        
        return entry_points, window_points, exit_points


class Dancer(DroneInterface):
    """Drone Interface extended with path to perform and async behavior wait"""

    def __init__(self, namespace: str, initial_position: List[float], verbose: bool = False, use_sim_time: bool = False):
        super().__init__(namespace, verbose=verbose, use_sim_time=use_sim_time)

        self.__path = []
        self.__current = 0
        self.initial_position = initial_position  # Store the drone's initial position

        self.__speed = 0.5  # Default speed
        self.__yaw_mode = YawMode.PATH_FACING
        self.__yaw_angle = None
        self.__frame_id = "earth"

        self.current_behavior: Optional[BehaviorHandler] = None

        self.led_pub = self.create_publisher(ColorRGBA, f"/{namespace}/leds/control", 10)

    def set_path(self, path: list) -> None:
        """Set the path for this drone"""
        self.__path = path
        self.__current = 0

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

    def change_leds_random_colour(self):
        self.change_led_colour([random.randint(0, 255) for _ in range(3)])

    def reset(self) -> None:
        """Set current waypoint in path to start point"""
        self.__current = 0

    def do_behavior(self, beh, *args) -> None:
        """Start behavior and save current to check if finished or not"""
        self.current_behavior = getattr(self, beh)
        self.current_behavior(*args)

    def go_to_next(self, speed=None) -> None:
        """Got to next position in path"""
        if self.__current < len(self.__path):
            point = self.__path[self.__current]
            # Use custom speed if provided, otherwise use default
            current_speed = speed if speed is not None else self.__speed
            self.do_behavior("go_to", point[0], point[1], point[2], current_speed,
                            self.__yaw_mode, self.__yaw_angle, self.__frame_id, False)
            self.__current += 1
            self.change_leds_random_colour()
        else:
            print(f"Drone {self.drone_id} has completed its path.")

    def goal_reached(self) -> bool:
        """Check if current behavior has finished"""
        if not self.current_behavior:
            return False

        if self.current_behavior.status == BehaviorStatus.IDLE:
            return True
        return False


class WindowTraversalConductor:
    """Centralized conductor for window traversal mission"""

    def __init__(self, scenario_file: str, world_file: str, verbose: bool = False,
                 use_sim_time: bool = False):
        # Load configuration from scenario file
        with open(scenario_file, 'r') as file:
            self.scenario_config = yaml.safe_load(file)
        
        # Load configuration from world file to get initial drone positions
        with open(world_file, 'r') as file:
            self.world_config = yaml.safe_load(file)
        
        # Extract drone information from world file
        drone_data = self.get_drone_data_from_world()
        
        # Initialize drones with their initial positions
        self.drones: Dict[int, Dancer] = {}
        for index, (drone_name, position) in enumerate(drone_data):
            self.drones[index] = Dancer(drone_name, position, verbose, use_sim_time)
            print(f"Initialized drone {drone_name} at position {position}")
        
        # Extract window information
        self.stage_center = self.scenario_config['stage2']['stage_center']
        self.room_height = self.scenario_config['stage2']['room_height']
        
        print(f"Stage center: {self.stage_center}")
        print(f"Room height: {self.room_height}")
        
        # Parse window configurations
        self.windows = {}
        for window_id, window_config in self.scenario_config['stage2']['windows'].items():
            # Convert window_id to int if it's a string (since we'll access with integers)
            window_id_int = int(window_id)
            self.windows[window_id_int] = WindowInfo(window_id_int, window_config, self.stage_center)
        
        # Calculate paths for each drone
        self.calculate_paths()

    def get_drone_data_from_world(self):
        """Extract drone names and initial positions from world configuration file"""
        drone_data = []
        for drone in self.world_config.get('drones', []):
            name = drone['model_name']
            # Convert position to the correct format [x, y, z]
            position = [float(drone['xyz'][0]), float(drone['xyz'][1]), float(drone['xyz'][2])]
            drone_data.append((name, position))
        return drone_data


    def calculate_paths(self):
        """Calculate traversal paths for all drones - without a pre-window formation"""
        num_drones = len(self.drones)
        
        # Access windows using integer keys
        window2 = self.windows[1]
        window1 = self.windows[2]

        # For Window 1, use a line formation (it's wider)
        w2_entry, w2_window, w2_exit = window2.get_square_formation_positions(num_drones)
        
        w1_entry, w1_window, w1_exit = window1.get_single_file_formation(num_drones)
        
        # Create intermediate waypoint between windows
        intermediate_positions = []
        for i in range(num_drones):
            # Calculate midpoint between exit of window 1 and entry to window 2
            mid_x = (w1_exit[i][0] + w2_entry[i][0]) / 2
            mid_y = (w1_exit[i][1] + w2_entry[i][1]) / 2
            mid_z = (w1_exit[i][2] + w2_entry[i][2]) / 2
            intermediate_positions.append((mid_x, mid_y, mid_z))
        
        # Final position (after second window)
        final_positions = []
        for i in range(num_drones):
            # Move 3 meters beyond the second window exit
            final_x = w2_exit[i][0]
            final_y = w2_exit[i][1] - 0.3  # 3 meters beyond window 2 exit
            final_z = w2_exit[i][2]
            final_positions.append((final_x, final_y, final_z))
        
        # Set paths for each drone - starting from their individual initial positions
        for drone_idx in range(num_drones):
            # Now, create a path that goes directly from takeoff to window 1 entry
            path = [
                # Second window waypoints
                w2_entry[drone_idx],
                w2_window[drone_idx],
                w2_exit[drone_idx],
                
                # First window waypoints - direct approach from takeoff position
                w1_exit[drone_idx],
                w1_window[drone_idx],
                w1_entry[drone_idx],
            ]
            
            # Set path for this drone
            self.drones[drone_idx].set_path(path)
            
            print(f"Path for drone {drone_idx}:")
            for i, waypoint in enumerate(path):
                print(f"  Waypoint {i}: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f})")

    def shutdown(self):
        """Shutdown all drones in swarm"""
        for drone in self.drones.values():
            drone.shutdown()

    def wait(self):
        """Wait until all drones has reached their goal (aka finished its behavior)"""
        all_finished = False
        while not all_finished:
            all_finished = True
            for drone in self.drones.values():
                all_finished = all_finished and drone.goal_reached()
            time.sleep(0.1)  # Short sleep to reduce CPU usage

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
            # Use the drone's initial z position plus 1 meter for safe takeoff
            takeoff_height = drone.initial_position[2] + 1.0
            drone.do_behavior("takeoff", takeoff_height, 0.7, False)
            drone.change_led_colour((0, 255, 0))
        self.wait()

    def land(self):
        """Land swarm and wait for all drones"""
        for drone in self.drones.values():
            drone.do_behavior("land", 0.4, False)
        self.wait()

    def traverse_windows(self):
        """Execute the window traversal mission with direct approach"""
        print("Starting window traversal mission...")
        
        # Execute each waypoint in the paths
        num_waypoints = len(self.drones[0]._Dancer__path)  # Get number of waypoints from first drone
        
        waypoint_descriptions = [
            "Approaching first window",
            "Passing through first window",
            "Exiting first window",
            "Approaching second window",
            "Passing through second window",
            "Exiting second window",
        ]
        
        for waypoint_idx in range(num_waypoints):
            # Print description of current waypoint
            if waypoint_idx < len(waypoint_descriptions):
                print(waypoint_descriptions[waypoint_idx])
            
            # Determine speed based on waypoint
            speed = 0.8  # Default speed
            
            # Move all drones to next waypoint
            for drone in self.drones.values():
                drone.go_to_next(speed)
            
            # Wait for all drones to complete the movement
            self.wait()
        
        print("Window traversal mission completed!")


def confirm(msg: str = 'Continue') -> bool:
    """Confirm message"""
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    return False


def main():
    parser = argparse.ArgumentParser(
        description='Window traversal mission for drone swarm')

    parser.add_argument('-s', '--scenario',
                        type=str,
                        default='scenarios/scenario1_stage2.yaml',
                        help='Path to scenario configuration file')
    parser.add_argument('-w', '--world',
                        type=str,
                        default='config_sim/config/world_swarm.yaml',
                        help='Path to world configuration file')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-t', '--use_sim_time',
                        action='store_true',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    scenario_file = args.scenario
    world_file = args.world
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    print(f"Scenario file: {scenario_file}")
    print(f"World file: {world_file}")

    rclpy.init()
    window_traversal = WindowTraversalConductor(
        scenario_file,
        world_file,
        verbose=verbosity,
        use_sim_time=use_sim_time)

    if confirm("Get ready and takeoff"):
        window_traversal.get_ready()
        window_traversal.takeoff()

        print("Begin window traversal")
        window_traversal.traverse_windows()

        print("Land")
        window_traversal.land()

    print("Shutdown")
    window_traversal.shutdown()
    rclpy.shutdown()

    sys.exit(0)


if __name__ == '__main__':
    main()