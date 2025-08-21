#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import threading
import time


class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Parameters
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('track_width', 0.3)
        self.declare_parameter('max_steering_angle', 0.5236)  # 30 degrees
        self.declare_parameter('update_rate', 50.0)
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Current command values
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.lock = threading.Lock()
        
        # Robot pose for odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
        
        # Current joint positions for smooth animation
        self.front_left_steering = 0.0
        self.front_right_steering = 0.0
        self.wheel_positions = {
            'front_wheel_left_joint': 0.0,
            'front_wheel_right_joint': 0.0,
            'rear_wheel_left_joint': 0.0,
            'rear_wheel_right_joint': 0.0,
        }
        
        # Publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing joint states
        self.timer = self.create_timer(
            1.0 / self.update_rate, self.publish_joint_states)
        
        self.get_logger().info(f'Ackermann Controller started with wheelbase={self.wheelbase}m, track_width={self.track_width}m')
    
    def cmd_vel_callback(self, msg):
        with self.lock:
            self.linear_vel = msg.linear.x
            self.angular_vel = msg.angular.z
    
    def calculate_ackermann_steering(self, linear_vel, angular_vel):
        """Calculate Ackermann steering angles for front wheels"""
        if abs(angular_vel) < 1e-6:
            # Going straight
            left_steering = 0.0
            right_steering = 0.0
        else:
            # Calculate turning radius
            if abs(linear_vel) < 1e-6:
                # Pure rotation
                left_steering = self.max_steering_angle if angular_vel > 0 else -self.max_steering_angle
                right_steering = -left_steering
            else:
                turning_radius = linear_vel / angular_vel
                
                # Calculate inner and outer wheel steering angles
                if turning_radius > 0:
                    # Turning left
                    left_steering = math.atan(self.wheelbase / (turning_radius - self.track_width / 2))
                    right_steering = math.atan(self.wheelbase / (turning_radius + self.track_width / 2))
                else:
                    # Turning right
                    left_steering = math.atan(self.wheelbase / (turning_radius + self.track_width / 2))
                    right_steering = math.atan(self.wheelbase / (turning_radius - self.track_width / 2))
                
                # Limit steering angles
                left_steering = max(-self.max_steering_angle, min(self.max_steering_angle, left_steering))
                right_steering = max(-self.max_steering_angle, min(self.max_steering_angle, right_steering))
        
        return left_steering, right_steering
    
    def publish_joint_states(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        with self.lock:
            linear_vel = self.linear_vel
            angular_vel = self.angular_vel
        
        # Update robot pose (odometry)
        self.x += linear_vel * math.cos(self.theta) * dt
        self.y += linear_vel * math.sin(self.theta) * dt
        self.theta += angular_vel * dt
        
        # Calculate steering angles
        left_steering, right_steering = self.calculate_ackermann_steering(linear_vel, angular_vel)
        
        # Smooth steering transition (simple low-pass filter)
        alpha = 0.1  # Smoothing factor
        self.front_left_steering = alpha * left_steering + (1 - alpha) * self.front_left_steering
        self.front_right_steering = alpha * right_steering + (1 - alpha) * self.front_right_steering
        
        # Update wheel rotations based on linear velocity
        wheel_radius = 0.05  # 5cm wheel radius
        wheel_angular_vel = linear_vel / wheel_radius
        
        # Update wheel positions
        for wheel in self.wheel_positions:
            self.wheel_positions[wheel] += wheel_angular_vel * dt
            # Keep angles within reasonable bounds
            if self.wheel_positions[wheel] > 2 * math.pi:
                self.wheel_positions[wheel] -= 2 * math.pi
            elif self.wheel_positions[wheel] < -2 * math.pi:
                self.wheel_positions[wheel] += 2 * math.pi
        
        # Publish odometry
        self.publish_odometry(current_time, linear_vel, angular_vel)
        
        # Create and publish joint state message
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
            self.front_left_steering,
            self.front_right_steering,
            self.wheel_positions['front_wheel_left_joint'],
            self.wheel_positions['front_wheel_right_joint'],
            self.wheel_positions['rear_wheel_left_joint'],
            self.wheel_positions['rear_wheel_right_joint']
        ]
        
        joint_state.velocity = [0.0] * len(joint_state.name)
        joint_state.effort = [0.0] * len(joint_state.name)
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_odometry(self, current_time, linear_vel, angular_vel):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Set velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)
        
        # Broadcast TF transform from odom to base_link
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
    controller = AckermannController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
