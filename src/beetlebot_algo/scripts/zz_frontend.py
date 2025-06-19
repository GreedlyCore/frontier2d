#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from geometry_msgs.msg import Point
from utils import euler_from_quaternion, angle_diff

class GraphSlamVisualizer(Node):
    def __init__(self):
        super().__init__('graph_slam_visualizer')

        self.declare_parameter('translation_threshold', 2.0)
        self.declare_parameter('rotation_threshold', 0.5)
        self.declare_parameter('map_frame', 'beetlebot/odom')

        self.translation_threshold = self.get_parameter('translation_threshold').value
        self.rotation_threshold = self.get_parameter('rotation_threshold').value
        self.map_frame = self.get_parameter('map_frame').value

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/graph_markers', 10)

        self.last_pose = None
        self.node_id = 0
        self.node_positions = []
        self.edges = []

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(ori_q)
        current_pose = np.array([pos.x, pos.y, yaw])

        if self.last_pose is None:
            self.add_node(current_pose)
            self.last_pose = current_pose
            return

        delta = current_pose - self.last_pose
        delta[2] = angle_diff(current_pose[2], self.last_pose[2])

        if np.linalg.norm(delta[:2]) >= self.translation_threshold or abs(delta[2]) >= self.rotation_threshold:
            self.add_node(current_pose)
            self.edges.append((self.node_id - 2, self.node_id - 1))
            self.last_pose = current_pose
            self.publish_markers() # !!!!!!!!!

    def add_node(self, pose):
        self.node_positions.append(pose)
        self.node_id += 1

    def publish_markers(self):
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        node_marker = Marker()
        node_marker.header.frame_id = self.map_frame
        node_marker.header.stamp = self.get_clock().now().to_msg()
        node_marker.ns = 'graph_nodes'
        node_marker.type = Marker.SPHERE
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.1
        node_marker.scale.y = 0.1
        node_marker.scale.z = 0.1
        node_marker.color.r = 0.0
        node_marker.color.g = 1.0
        node_marker.color.b = 0.0
        node_marker.color.a = 1.0

        for idx, pose in enumerate(self.node_positions):
            marker = Marker()
            marker.header = node_marker.header
            marker.ns = node_marker.ns
            marker.id = idx
            marker.type = node_marker.type
            marker.action = node_marker.action
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = 0.0
            marker.scale = node_marker.scale
            marker.color = node_marker.color
            marker_array.markers.append(marker)

        edge_marker = Marker()
        edge_marker.header.frame_id = self.map_frame
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.ns = 'graph_edges'
        edge_marker.id = 0
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.05
        edge_marker.color.r = 0.0
        edge_marker.color.g = 0.0
        edge_marker.color.b = 1.0
        edge_marker.color.a = 1.0
        edge_marker.points = []

        for edge in self.edges:   
            p0 = Point()
            p0.x = edge.from_idx[0]
            p0.y = edge.from_idx[1]
            p0.z = 0.0
            p1 = Point()
            p1.x = edge.to_idx[0]
            p1.y = edge.to_idx[1]
            p1.z = 0.0
            edge_marker.points.extend([p0, p1])

        marker_array.markers.append(edge_marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GraphSlamVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
