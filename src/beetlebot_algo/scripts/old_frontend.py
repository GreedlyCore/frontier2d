#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


from utils import euler_from_quaternion, angle_diff
from GraphUtils import Pose3, Edge


class GraphSLAMFrontend(Node):
    def __init__(self):
        super().__init__('graph_slam_frontend')

        self.declare_parameter('translation_threshold', 2.0)
        self.declare_parameter('rotation_threshold', 0.5)
        self.declare_parameter('map_frame', 'beetlebot/odom')

        self.translation_threshold = self.get_parameter('translation_threshold').value
        self.rotation_threshold = self.get_parameter('rotation_threshold').value
        self.map_frame = self.get_parameter('map_frame').value

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(Odometry, '/lidar', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/graph_markers', 10)
        

        self.nodes = []         # List[Pose3]
        self.edges = []         # List[Edge]
        self.last_pose = None
        self.pose_id = 0
        self.last_scan = None

        # Covariance → information matrix (idk how to init it properly)
        self.covariance = np.diag([0.02, 0.02, 0.01])
        self.information_matrix = np.linalg.inv(self.covariance)

    def scan_callback(self, msg: LaserScan):
        # TODO: add unique, id system (???) and add an id to every vertex, edges?
        self.last_scan = list(msg.ranges)

    def odom_callback(self, msg: Odometry):
        pose = self._extract_pose(msg.pose.pose)

        if self.last_pose is None:
            self.nodes.append(pose)
            self.last_pose = pose
            self.pose_id += 1
            return

        # Compute relative motion: u = x_i-1^-1 ⊕ x_i
        delta = self._relative_pose(self.last_pose, pose)

        # Pose composition: x_i = x_i-1 ⊕ u
        new_pose = self.last_pose.compose(delta)


        if np.linalg.norm(delta.as_vector()[:2]) >= self.translation_threshold or abs(delta.theta) >= self.rotation_threshold:
            # Graph update
            self.nodes.append(new_pose)
            edge = Edge(self.pose_id - 1, self.pose_id, delta, self.information_matrix)
            edge.set_scan(self.last_scan)
            self.edges.append(edge)

            self.last_pose = new_pose
            self.pose_id += 1

            self.publish_markers() 
            # self.get_logger().info(f"Pose[{self.pose_id - 1}]: {new_pose}")
            # self.get_logger().info(f"Edge added: {edge}")    
            

    def _extract_pose(self, pose_msg: Pose):
        pos = pose_msg.position
        ori_q = pose_msg.orientation
        _, _, yaw = euler_from_quaternion(ori_q)
        return Pose3(pos.x, pos.y, yaw)

    def _relative_pose(self, pose1: Pose3, pose2: Pose3):
        """Compute relative pose u between pose1 and pose2: u = x1^-1 ⊕ x2"""
        dx = pose2.x - pose1.x
        dy = pose2.y - pose1.y
        dtheta = pose2.theta - pose1.theta
        dtheta = Pose3._normalize_angle(dtheta)

        cos_theta = np.cos(-pose1.theta)
        sin_theta = np.sin(-pose1.theta)

        x_rel = cos_theta * dx - sin_theta * dy
        y_rel = sin_theta * dx + cos_theta * dy

        return Pose3(x_rel, y_rel, dtheta)

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
        # TODO: depend решиться какая система индексов будет - у nodes или у edges или у обеих??
        for idx in range(len(self.nodes)):
            marker = Marker()
            marker.header = node_marker.header
            marker.ns = node_marker.ns
            marker.id = idx
            marker.type = node_marker.type
            marker.action = node_marker.action
            marker.pose.position.x = self.nodes[idx].x
            marker.pose.position.y = self.nodes[idx].y
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
            from_pose = self.nodes[edge.from_idx]
            to_pose = self.nodes[edge.to_idx]
            p0 = Point()
            p0.x = from_pose.x
            p0.y = from_pose.y
            p0.z = 0.0
            p1 = Point()
            p1.x = to_pose.x
            p1.y = to_pose.y
            p1.z = 0.0
            edge_marker.points.extend([p0, p1])

        marker_array.markers.append(edge_marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GraphSLAMFrontend()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

