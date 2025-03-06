#!/bin/python3

"""
CAMERA SAMPLE MISSION

This file is an example mission which reads from the aerostack drone camera and prints it to screen

It also flies around using position and velocity control 
"""

# Imports
import time
import rclpy
import argparse
from as2_python_api.drone_interface import DroneInterface

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from ros_gz_interfaces.msg import Float32Array

from cv_bridge import CvBridge
import cv2 

######## Drone Control Class ###################
class DroneMotionRef(DroneInterface):
    """Drone Interface
    
    This is the aerostack2 drone interface for connecting to simulated and real drones. 

    It runs as a ROS2 Node which interacts with the currently available ROS2 topics.
    It defines the variables that represent a single drone, i.e.
    - Platform Information
    - Vehicle Pose and Twist (angular velocity)
    - Functions to control the hardware of the drone (arm, disarm, change mode, estop)

    It also contains some modules for flying the drone, this includes:
    - Takeoff, Landing (self.takeoff, self.land)
    - GoTo position control (self.go_to) [https://github.com/aerostack2/aerostack2/blob/main/as2_python_api/as2_python_api/modules/go_to_module.py]
    - FollowPath module (self.follow_path) [https://github.com/aerostack2/aerostack2/blob/main/as2_python_api/as2_python_api/modules/follow_path_module.py]
    
    Other module exist which could be used to. Their interfaces and functions can be referenced most easily in the code. 

    Some Documentation is here: https://aerostack2.github.io/_09_development/_api_documentation/temp_ws/src/as2_python_api/docs/source/as2_python_api.html 
    The Source Code is here: https://github.com/aerostack2/aerostack2/tree/main/as2_python_api 

    Drone Interface Base.py: https://github.com/aerostack2/aerostack2/blob/main/as2_python_api/as2_python_api/drone_interface_base.py 
    Drone Interface.py: https://github.com/aerostack2/aerostack2/blob/main/as2_python_api/as2_python_api/drone_interface.py
    """

    def __init__(self, name, verbose=False, use_sim_time=False):
        super().__init__(name, verbose, use_sim_time)

        # ROS2 create a subscription to the raw image of the sensors.
        # This details the ros message type (Image), the name of the topic
        # And the function that should be called when a message is received on this topic
        self.led_pub = self.create_publisher(Float32Array, f"/{name}/leds/control", 10)
        self.num_leds = 12
        self.led_colours = [(0, 0, 0) for _ in range(self.num_leds)]

    def change_leds(self, led_id, colour):
        """Change the colours

        Args:
            led_id (int | List[int]): The LED ID to change
            colour (tuple | List[tuple]): The LED RGB Colours to change to 0-255
        """
        if not isinstance(led_id, list):
            led_id = [led_id]
            colour = [colour]

        for lid, c in zip(led_id, colour):
            self.led_colours[lid] = c
        
        msg = Float32Array()
        for c in self.led_colours:
            msg.data.extend(c)
        self.led_pub.publish(msg)

    def change_all_leds_same_colour(self, colour):
        self.change_leds(
            [i for i in range(self.num_leds)],
            [colour for _ in range(self.num_leds)]
        )

    def run_test(self):
        """ Run the mission """

        # Set the drone to offboard mode. This prepares the drone to receive
        # commands from outside of the flight controller. 
        self.offboard()
        self.get_logger().info("Offboard Mode")

        # Arming the drone powers up the motors to prepare for flight
        self.arm()
        self.get_logger().info("Armed!")

        # Takeoff to 1 meter
        self.get_logger().info("Taking Off!")
        res = self.takeoff(height=1.0, speed=0.5)
        if res:
            self.get_logger().info("Take off complete")
        else:
            self.get_logger().info("Take off Failed, exiting")
            return
        
        # Wait a little bit
        time.sleep(1.0)

        # Position Control fly around a bit
        speed = 1.5
        self.go_to.go_to_point([1, 0, 1.0], speed=speed)
        self.change_all_leds_same_colour((255, 0, 0))
        self.get_logger().info("Point 1")
        self.go_to.go_to_point([2, 0, 2.0], speed=speed)
        self.change_all_leds_same_colour((255, 0, 255))
        self.get_logger().info("Point 2")
        self.go_to.go_to_point([3, 0, 3.0], speed=speed)
        self.change_all_leds_same_colour((255, 255, 0))
        self.get_logger().info("Point 3")
        self.go_to.go_to(3.0, -1.0, 2.5, speed=speed)
        self.change_all_leds_same_colour((125, 0, 125))
        self.get_logger().info("Point 4")
        self.go_to.go_to_point_with_yaw([4, 1, 3.0], angle=45.0, speed=speed)
        self.change_all_leds_same_colour(([78, 190, 255]))
        self.get_logger().info("Point 5")
        self.go_to.go_to_point_with_yaw([3, -2, 2.0], angle=-45.0, speed=speed)
        self.change_all_leds_same_colour((59, 255, 180))
        self.get_logger().info("Point 6")
        self.go_to.go_to_point_with_yaw([0, 0, 1.0], angle=0.0, speed=speed)
        self.change_all_leds_same_colour((129, 120, 180))
        self.get_logger().info("Point 7")

        self.land()

############# Running the mission and Entrypoint #################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
    description="Starts camera mission")
    parser.add_argument('-s', '--simulated',
                        action='store_true', default=False)
    parser.add_argument('-n', '--drone_name', default="cf0")
    args = parser.parse_args()

    if args.simulated:
        print("Mission running in simulation mode")
    else:
        print("Mission running in real mode")

    # Starts ROS2 Node in a script
    rclpy.init()

    # Create the drone object. Connects to the real/simulated drone and runs tests
    uav = DroneMotionRef(args.drone_name, verbose=True)

    # Runs the UAV TEST function
    uav.run_test()

    # Shuts down the UAV
    uav.shutdown()

    # Stop ROS2 Node
    rclpy.shutdown()

    print("Clean exit")
    exit(0)

