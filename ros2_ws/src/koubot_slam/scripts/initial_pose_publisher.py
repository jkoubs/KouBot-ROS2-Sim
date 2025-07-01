#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
import tf2_ros

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publish_count = 0
        self.timer = self.create_timer(1.0, self.check_and_publish)

    def check_and_publish(self):
        # Wait for TF from base_footprint to odom to be available
        try:
            self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.get_logger().info('Waiting for TF from base_footprint to odom...')
            return

        if self.pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for AMCL to subscribe to /initialpose...')
            return

        if self.publish_count < 3:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0

            msg.pose.pose.position.x = -2.0
            msg.pose.pose.position.y = 0.0
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.068

            self.pub.publish(msg)
            self.get_logger().info(f'Published initial pose ({self.publish_count + 1}/3)')
            self.publish_count += 1
        else:
            self.get_logger().info('Initial pose publishing complete.')
            self.timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
