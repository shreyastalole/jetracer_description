#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time


class SimpleWASDController(Node):
    def __init__(self):
        super().__init__('simple_wasd_controller')
        
        # Parameters
        self.declare_parameter('update_rate', 30.0)
        self.update_rate = self.get_parameter('update_rate').value
        
        # Robot pose for odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Joint positions
        self.wheel_positions = {
            'front_wheel_left_joint': 0.0,
            'front_wheel_right_joint': 0.0,
            'rear_wheel_left_joint': 0.0,
            'rear_wheel_right_joint': 0.0,
        }
        self.steering_angle = 0.0
        
        # Publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_robot)
        
        self.get_logger().info('Simple WASD Controller Started!')
        self.get_logger().info('Use teleop_twist_keyboard to control:')
        self.get_logger().info('  ros2 run teleop_twist_keyboard teleop_twist_keyboard')
    
    def cmd_vel_callback(self, msg):
        """Handle incoming cmd_vel messages"""
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        
        # Calculate simple steering angle for visualization
        if abs(self.current_angular) > 0.01:
            self.steering_angle = self.current_angular * 0.2  # Scale down for visualization
        else:
            self.steering_angle = 0.0
        
        # Limit steering angle
        self.steering_angle = max(-0.5, min(0.5, self.steering_angle))
    
    def update_robot(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Update robot pose
        self.x += self.current_linear * math.cos(self.theta) * dt
        self.y += self.current_linear * math.sin(self.theta) * dt
        self.theta += self.current_angular * dt
        
        # Update wheel positions
        wheel_radius = 0.05
        wheel_angular_vel = self.current_linear / wheel_radius
        
        for wheel in self.wheel_positions:
            self.wheel_positions[wheel] += wheel_angular_vel * dt
        
        # Publish everything
        self.publish_joint_states()
        self.publish_odometry()
    
    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        joint_state.name = [
            'front_steering_left',
            'front_steering_right',
            'front_wheel_left_joint',
            'front_wheel_right_joint',
            'rear_wheel_left_joint',
            'rear_wheel_right_joint'
        ]
        
        joint_state.position = [
            self.steering_angle,
            self.steering_angle,
            self.wheel_positions['front_wheel_left_joint'],
            self.wheel_positions['front_wheel_right_joint'],
            self.wheel_positions['rear_wheel_left_joint'],
            self.wheel_positions['rear_wheel_right_joint']
        ]
        
        joint_state.velocity = [0.0] * len(joint_state.name)
        joint_state.effort = [0.0] * len(joint_state.name)
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_odometry(self):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Set velocity
        odom.twist.twist.linear.x = self.current_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.current_angular
        
        self.odom_pub.publish(odom)
        
        # Broadcast TF
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleWASDController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
