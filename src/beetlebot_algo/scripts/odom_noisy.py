#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import math


class NoisyOdometryNode(Node):
    def __init__(self):
        super().__init__('noisy_odometry_node')
        
        # Variables for dead reckoning error tracking
        self.previous_x = None
        self.previous_y = None
        self.total_distance_traveled = 0.0
        self.dead_reckoning_error_x = 0.0
        self.dead_reckoning_error_y = 0.0
        
        # Declare parameters with descriptions
        self.declare_parameter(
            'position_variance', 
            0.03,
            ParameterDescriptor(description='Variance for position noise (x, y, z)')
        )
        self.declare_parameter(
            'orientation_variance', 
            0.01,
            ParameterDescriptor(description='Variance for orientation noise (quaternion)')
        )
        self.declare_parameter(
            'linear_velocity_variance', 
            0.05,
            ParameterDescriptor(description='Variance for linear velocity noise')
        )
        self.declare_parameter(
            'angular_velocity_variance', 
            0.02,
            ParameterDescriptor(description='Variance for angular velocity noise')
        )
        
        self.declare_parameter(
            'enable_dead_reckoning_error',
            True,
            ParameterDescriptor(description='Enable linear dead reckoning error mode')
        )
        self.declare_parameter(
            'distance_threshold',
            0.1,
            ParameterDescriptor(description='Distance interval (m) to add dead reckoning error')
        )
        self.declare_parameter(
            'dead_reckoning_error_x',
            0.01,
            ParameterDescriptor(description='Constant error added to x position per distance threshold')
        )
        self.declare_parameter(
            'dead_reckoning_error_y',
            0.01,
            ParameterDescriptor(description='Constant error added to y position per distance threshold')
        )
        
        # Get parameter values
        self.pos_var = self.get_parameter('position_variance').value
        self.orient_var = self.get_parameter('orientation_variance').value
        self.lin_vel_var = self.get_parameter('linear_velocity_variance').value
        self.ang_vel_var = self.get_parameter('angular_velocity_variance').value
        
        self.enable_dead_reckoning = self.get_parameter('enable_dead_reckoning_error').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.dr_error_x_per_threshold = self.get_parameter('dead_reckoning_error_x').value
        self.dr_error_y_per_threshold = self.get_parameter('dead_reckoning_error_y').value
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(
            Odometry,
            '/odom/noisy',
            10
        )
        
        self.get_logger().info(f'Noisy Odometry Node started')
        self.get_logger().info(f'Position variance: {self.pos_var}')
        self.get_logger().info(f'Orientation variance: {self.orient_var}')
        self.get_logger().info(f'Dead reckoning error mode: {"ENABLED" if self.enable_dead_reckoning else "DISABLED"}')
        if self.enable_dead_reckoning:
            self.get_logger().info(f'Distance threshold: {self.distance_threshold}m')
            self.get_logger().info(f'Error per threshold - X: {self.dr_error_x_per_threshold}, Y: {self.dr_error_y_per_threshold}')

    def add_gaussian_noise(self, value, variance):
        return value + np.random.normal(0, np.sqrt(variance))

    def update_dead_reckoning_error(self, current_x, current_y):
        """Update dead reckoning error based on distance traveled"""
        if not self.enable_dead_reckoning:
            return
            
        if self.previous_x is not None and self.previous_y is not None:
            # Calculate distance traveled since last update
            distance_increment = math.sqrt(
                (current_x - self.previous_x)**2 + 
                (current_y - self.previous_y)**2
            )
            self.total_distance_traveled += distance_increment
            
            # Check if we've traveled enough distance to add error
            if self.total_distance_traveled >= self.distance_threshold:
                # Calculate how many threshold intervals we've crossed
                num_intervals = int(self.total_distance_traveled / self.distance_threshold)
                
                # Add error for each interval
                self.dead_reckoning_error_x += num_intervals * self.dr_error_x_per_threshold
                self.dead_reckoning_error_y += num_intervals * self.dr_error_y_per_threshold
                
                # Update remaining distance
                self.total_distance_traveled = self.total_distance_traveled % self.distance_threshold
                
                self.get_logger().debug(
                    f'Added dead reckoning error: X={self.dead_reckoning_error_x:.4f}, '
                    f'Y={self.dead_reckoning_error_y:.4f}, Distance={self.total_distance_traveled:.4f}'
                )
        
        # Update previous position
        self.previous_x = current_x
        self.previous_y = current_y

    def odom_callback(self, msg):
        # Update dead reckoning error based on traveled distance
        self.update_dead_reckoning_error(
            msg.pose.pose.position.x, 
            msg.pose.pose.position.y
        )
        
        noisy_msg = Odometry()        
        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id
        
        # Add noise and errors to position
        base_x = self.add_gaussian_noise(msg.pose.pose.position.x, self.pos_var) 
        base_y = self.add_gaussian_noise(msg.pose.pose.position.y, self.pos_var)
        
        noisy_msg.pose.pose.position.x = base_x + self.dead_reckoning_error_x
        noisy_msg.pose.pose.position.y = base_y + self.dead_reckoning_error_y
        noisy_msg.pose.pose.position.z = msg.pose.pose.position.z 

        # Add noise to orientation (quaternion)
        noisy_msg.pose.pose.orientation.x = self.add_gaussian_noise(
            msg.pose.pose.orientation.x, self.orient_var
        )
        noisy_msg.pose.pose.orientation.y = self.add_gaussian_noise(
            msg.pose.pose.orientation.y, self.orient_var
        )
        noisy_msg.pose.pose.orientation.z = self.add_gaussian_noise(
            msg.pose.pose.orientation.z, self.orient_var
        )
        noisy_msg.pose.pose.orientation.w = self.add_gaussian_noise(
            msg.pose.pose.orientation.w, self.orient_var
        )
        
        # Normalize quaternion to maintain unit length
        quat_norm = np.sqrt(
            noisy_msg.pose.pose.orientation.x**2 +
            noisy_msg.pose.pose.orientation.y**2 +
            noisy_msg.pose.pose.orientation.z**2 +
            noisy_msg.pose.pose.orientation.w**2
        )
        
        if quat_norm > 0:
            noisy_msg.pose.pose.orientation.x /= quat_norm
            noisy_msg.pose.pose.orientation.y /= quat_norm
            noisy_msg.pose.pose.orientation.z /= quat_norm
            noisy_msg.pose.pose.orientation.w /= quat_norm
        
        # Copy pose covariance
        noisy_msg.pose.covariance = msg.pose.covariance
        
        # Add noise to linear velocity
        noisy_msg.twist.twist.linear.x = self.add_gaussian_noise(
            msg.twist.twist.linear.x, self.lin_vel_var
        )
        noisy_msg.twist.twist.linear.y = self.add_gaussian_noise(
            msg.twist.twist.linear.y, self.lin_vel_var
        )
        noisy_msg.twist.twist.linear.z = self.add_gaussian_noise(
            msg.twist.twist.linear.z, self.lin_vel_var
        )
        
        # Add noise to angular velocity
        noisy_msg.twist.twist.angular.x = self.add_gaussian_noise(
            msg.twist.twist.angular.x, self.ang_vel_var
        )
        noisy_msg.twist.twist.angular.y = self.add_gaussian_noise(
            msg.twist.twist.angular.y, self.ang_vel_var
        )
        noisy_msg.twist.twist.angular.z = self.add_gaussian_noise(
            msg.twist.twist.angular.z, self.ang_vel_var
        )
        noisy_msg.twist.covariance = msg.twist.covariance
        
        self.publisher.publish(noisy_msg)


def main(args=None):
    rclpy.init(args=args)
    
    noisy_odom_node = NoisyOdometryNode()
    
    try:
        rclpy.spin(noisy_odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        noisy_odom_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()