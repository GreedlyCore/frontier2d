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

        # Original parameters
        self.declare_parameter('translation_threshold', 2.0)
        self.declare_parameter('rotation_threshold', 0.5)
        self.declare_parameter('map_frame', 'beetlebot/odom')
        
        # Loop closure parameters
        self.declare_parameter('loop_closure_distance_threshold', 0.6)
        self.declare_parameter('loop_closure_chain_length', 3)
        self.declare_parameter('loop_closure_min_separation', 10)  # Minimum node separation for loop closure

        self.translation_threshold = self.get_parameter('translation_threshold').value
        self.rotation_threshold = self.get_parameter('rotation_threshold').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Loop closure parameters
        self.loop_distance_threshold = self.get_parameter('loop_closure_distance_threshold').value
        self.chain_length = self.get_parameter('loop_closure_chain_length').value
        self.min_separation = self.get_parameter('loop_closure_min_separation').value

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/lidar', self.scan_callback, 10)  # Fixed: LaserScan instead of Odometry
        self.marker_pub = self.create_publisher(MarkerArray, '/graph_markers', 10)
        

        self.nodes = []         # List[Pose3]
        self.edges = []         # List[Edge]
        self.node_scans = []    # List[List] - Store scan for each node
        self.last_pose = None
        self.pose_id = 0
        self.last_scan = None

        # Covariance → information matrix (idk how to init it properly)
        self.covariance = np.diag([0.02, 0.02, 0.01])
        self.information_matrix = np.linalg.inv(self.covariance)
        
        # Loop closure information matrix (typically lower confidence)
        self.loop_covariance = np.diag([0.1, 0.1, 0.05])
        self.loop_information_matrix = np.linalg.inv(self.loop_covariance)

        self.get_logger().info(f"Graph SLAM Frontend initialized with loop closure")
        self.get_logger().info(f"Loop distance threshold: {self.loop_distance_threshold}m")
        self.get_logger().info(f"Chain length: {self.chain_length}")

    def scan_callback(self, msg: LaserScan):
        # Convert ranges to list and filter out invalid readings
        ranges = list(msg.ranges)
        # Replace inf and nan with max_range
        filtered_ranges = []
        for r in ranges:
            if np.isinf(r) or np.isnan(r):
                filtered_ranges.append(msg.range_max)
            else:
                filtered_ranges.append(r)
        
        self.last_scan = filtered_ranges

    def odom_callback(self, msg: Odometry):
        pose = self._extract_pose(msg.pose.pose)

        if self.last_pose is None:
            self.nodes.append(pose)
            self.node_scans.append(self.last_scan.copy() if self.last_scan else [])
            self.last_pose = pose
            self.pose_id += 1
            return

        # Compute relative motion: u = x_i-1^-1 ⊕ x_i
        delta = self._relative_pose(self.last_pose, pose)

        # Pose composition: x_i = x_i-1 ⊕ u
        new_pose = self.last_pose.compose(delta)

        if np.linalg.norm(delta.as_vector()[:2]) >= self.translation_threshold or abs(delta.theta) >= self.rotation_threshold:
            
            # Check for loop closure BEFORE adding the new node
            loop_closure_candidate = self._detect_loop_closure(new_pose)
            
            if loop_closure_candidate is not None:
                self.get_logger().info(f"Loop closure detected! Connecting node {self.pose_id} to node {loop_closure_candidate}")
                
                # Create loop closure edge instead of adding new node
                candidate_pose = self.nodes[loop_closure_candidate]
                loop_measurement = self._relative_pose(candidate_pose, new_pose)
                
                # Create loop closure edge
                loop_edge = Edge(loop_closure_candidate, self.pose_id - 1, loop_measurement, self.loop_information_matrix)
                loop_edge.set_scan(self.last_scan.copy() if self.last_scan else [])
                self.edges.append(loop_edge)
                
                # Update last pose but don't add new node
                self.last_pose = new_pose
                
            else:
                # No loop closure - add new node as usual
                self.nodes.append(new_pose)
                self.node_scans.append(self.last_scan.copy() if self.last_scan else [])
                
                # Create odometry edge
                edge = Edge(self.pose_id - 1, self.pose_id, delta, self.information_matrix)
                edge.set_scan(self.last_scan.copy() if self.last_scan else [])
                self.edges.append(edge)

                self.last_pose = new_pose
                self.pose_id += 1

            self.publish_markers() 

    def _detect_loop_closure(self, current_pose: Pose3):
        """
        Detect loop closure by checking if current pose is close to any previous node
        and if their scans are similar (using ICP).
        
        Returns the index of the loop closure candidate node, or None if no loop closure.
        """
        if len(self.nodes) < self.min_separation:
            return None
        
        # Get the chain of recent nodes (excluding very recent ones to avoid false positives)
        chain_start_idx = max(0, len(self.nodes) - self.chain_length - self.min_separation)
        chain_end_idx = len(self.nodes) - self.min_separation
        
        if chain_start_idx >= chain_end_idx:
            return None
        
        # Check each node in the valid range
        for candidate_idx in range(chain_start_idx, chain_end_idx):
            candidate_pose = self.nodes[candidate_idx]
            
            # Distance check
            distance = np.sqrt((current_pose.x - candidate_pose.x)**2 + 
                             (current_pose.y - candidate_pose.y)**2)
            
            if distance <= self.loop_distance_threshold:
                self.get_logger().info(f"Distance check passed for node {candidate_idx}: {distance:.3f}m")
                
                # Get scans for comparison
                candidate_scan = self.node_scans[candidate_idx] if candidate_idx < len(self.node_scans) else []
                current_scan = self.last_scan if self.last_scan else []
                
                # Get chain of recent scans for more robust matching
                chain_scans = []
                chain_start = max(0, len(self.node_scans) - self.chain_length)
                for i in range(chain_start, len(self.node_scans)):
                    if i < len(self.node_scans) and self.node_scans[i]:
                        chain_scans.extend(self.node_scans[i])
                
                # ICP comparison (placeholder - you'll implement this)
                if self._icp_scan_matching(candidate_scan, current_scan, chain_scans):
                    return candidate_idx
        
        return None

    def _icp_scan_matching(self, candidate_scan, current_scan, chain_scans):
        """
        Placeholder for ICP scan matching.
        
        Args:
            candidate_scan: Scan from the candidate loop closure node
            current_scan: Current laser scan
            chain_scans: Combined scans from recent chain of nodes
            
        Returns:
            bool: True if scans match (loop closure), False otherwise
        """
        # TODO: Implement your ICP algorithm here
        # This is where you'll do the actual scan matching
        
        # Placeholder logic - replace with your ICP implementation
        if not candidate_scan or not current_scan:
            self.get_logger().warn("Empty scans for ICP comparison")
            return False
        
        # Basic sanity checks
        if len(candidate_scan) != len(current_scan):
            self.get_logger().warn(f"Scan length mismatch: {len(candidate_scan)} vs {len(current_scan)}")
            return False
        
        # Placeholder: Simple similarity check (replace with ICP)
        # You can use chain_scans for more robust matching if needed
        similarity_score = self._compute_scan_similarity(candidate_scan, current_scan)
        threshold = 0.8  # Adjust based on your ICP implementation
        
        self.get_logger().info(f"Scan similarity score: {similarity_score:.3f}")
        
        # TODO: Replace this with your ICP result
        is_loop_closure = similarity_score > threshold
        
        if is_loop_closure:
            self.get_logger().info("ICP scan matching: LOOP CLOSURE DETECTED")
        else:
            self.get_logger().info("ICP scan matching: No loop closure")
            
        return is_loop_closure

    def _compute_scan_similarity(self, scan1, scan2):
        """
        Placeholder similarity computation - replace with ICP
        """
        if not scan1 or not scan2 or len(scan1) != len(scan2):
            return 0.0
        
        # Simple correlation-based similarity (not suitable for real use)
        s1 = np.array(scan1)
        s2 = np.array(scan2)
        
        # Filter out max range readings
        valid_mask = (s1 < 10.0) & (s2 < 10.0)  # Assume max range is around 10m
        if np.sum(valid_mask) < 10:  # Need at least 10 valid points
            return 0.0
        
        s1_valid = s1[valid_mask]
        s2_valid = s2[valid_mask]
        
        # Normalized correlation
        correlation = np.corrcoef(s1_valid, s2_valid)[0, 1]
        return correlation if not np.isnan(correlation) else 0.0

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

        # Node markers (green spheres)
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

        # Edge markers - separate odometry and loop closure edges
        # Odometry edges (blue lines)
        odom_edge_marker = Marker()
        odom_edge_marker.header.frame_id = self.map_frame
        odom_edge_marker.header.stamp = self.get_clock().now().to_msg()
        odom_edge_marker.ns = 'odometry_edges'
        odom_edge_marker.id = 0
        odom_edge_marker.type = Marker.LINE_LIST
        odom_edge_marker.action = Marker.ADD
        odom_edge_marker.scale.x = 0.03
        odom_edge_marker.color.r = 0.0
        odom_edge_marker.color.g = 0.0
        odom_edge_marker.color.b = 1.0
        odom_edge_marker.color.a = 1.0
        odom_edge_marker.points = []

        # Loop closure edges (red lines)
        loop_edge_marker = Marker()
        loop_edge_marker.header.frame_id = self.map_frame
        loop_edge_marker.header.stamp = self.get_clock().now().to_msg()
        loop_edge_marker.ns = 'loop_closure_edges'
        loop_edge_marker.id = 0
        loop_edge_marker.type = Marker.LINE_LIST
        loop_edge_marker.action = Marker.ADD
        loop_edge_marker.scale.x = 0.08  # Thicker for loop closures
        loop_edge_marker.color.r = 1.0
        loop_edge_marker.color.g = 0.0
        loop_edge_marker.color.b = 0.0
        loop_edge_marker.color.a = 1.0
        loop_edge_marker.points = []

        for edge in self.edges:
            if edge.from_idx < len(self.nodes) and edge.to_idx < len(self.nodes):
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
                
                # Determine if this is a loop closure edge
                # Loop closure: non-consecutive indices with large separation
                is_loop_closure = abs(edge.to_idx - edge.from_idx) > self.min_separation
                
                if is_loop_closure:
                    loop_edge_marker.points.extend([p0, p1])
                else:
                    odom_edge_marker.points.extend([p0, p1])

        marker_array.markers.append(odom_edge_marker)
        marker_array.markers.append(loop_edge_marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GraphSLAMFrontend()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()