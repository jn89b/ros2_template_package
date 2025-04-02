#!/usr/bin/env python3
from re import S
import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.publisher import Publisher
from nav_msgs.msg import Odometry
from drone_interfaces.msg import Telem, CtlTraj
from ros2_template_package import rotation_utils as rot_utils
import mavros
from mavros.base import SENSOR_QOS


"""
For this application we will be sending roll, pitch yaw commands to the drone
"""

class GuidancePublisher(Node):
    def __init__(self, ns=''):
        super().__init__('pub_example')

        self.trajectory_publisher: Publisher = self.create_publisher(
            CtlTraj, 'trajectory', 10)
        
        self.state_sub = self.create_subscription(mavros.local_position.Odometry,
                                                    'mavros/local_position/odom',
                                                    self.mavros_state_callback,
                                                    qos_profile=SENSOR_QOS)
        
        # self.target_sub = self.create_subscription(Odometry,
        #                                             'target_position',
        #                                             self.target_callback,
        #                                             10)
                                                   
        self.state_info = [
            None,  # x
            None,  # y
            None,  # z
            None,  # phi
            None,  # theta
            None,  # psi
            None,  # airspeed
        ]
        
        
    def mavros_state_callback(self, msg: mavros.local_position.Odometry) -> None:
        """
        Converts NED to ENU and publishes the trajectory
        """
        self.state_info[0] = msg.pose.pose.position.x
        self.state_info[1] = msg.pose.pose.position.y
        self.state_info[2] = msg.pose.pose.position.z

        # quaternion attitudes
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = rot_utils.euler_from_quaternion(
            qx, qy, qz, qw)

        self.state_info[3] = roll
        self.state_info[4] = pitch
        self.state_info[5] = yaw  # (yaw+ (2*np.pi) ) % (2*np.pi);

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        # get magnitude of velocity
        self.state_info[6] = np.sqrt(vx**2 + vy**2 + vz**2)
        # self.state_info[6] = #msg.twist.twist.linear.x

    def publish_trajectory(self, trajectory: CtlTraj) -> None:
        """
        Publishes the trajectory
        
        """
        self.trajectory_publisher.publish(trajectory)
        
        