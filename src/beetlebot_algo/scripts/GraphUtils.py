#!/usr/bin/env python3
import numpy as np

class Pose3:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def as_vector(self):
        return np.array([self.x, self.y, self.theta])
    
    def as_matrix(self):
        """Convert pose to 3x3 homogeneous transformation matrix"""
        c = np.cos(self.theta)
        s = np.sin(self.theta)
        return np.array([
            [c, -s, self.x],
            [s,  c, self.y],
            [0,  0, 1.0]
        ])
    
    @classmethod
    def from_matrix(cls, matrix):
        """Create Pose3 from 3x3 homogeneous transformation matrix"""
        x = matrix[0, 2]
        y = matrix[1, 2]
        theta = np.arctan2(matrix[1, 0], matrix[0, 0])
        return cls(x, y, theta)

    def distance_to(self, other):
        """Euclidean distance to another pose"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def angle_distance_to(self, other):
        """Angular distance to another pose"""
        return abs(self._normalize_angle(self.theta - other.theta))

    def inverse(self):
        """Compute inverse pose: x^-1"""
        c = np.cos(-self.theta)
        s = np.sin(-self.theta)
        x_inv = -(c * self.x - s * self.y)
        y_inv = -(s * self.x + c * self.y)
        theta_inv = -self.theta
        return Pose3(x_inv, y_inv, self._normalize_angle(theta_inv))

    def __sub__(self, other):
        return Pose3(self.x - other.x, self.y - other.y, 
                    self._normalize_angle(self.theta - other.theta))

    def __repr__(self):
        return f"Pose3: (x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f})"

    def compose(self, delta):
        """Pose composition: x ⊕ u"""
        dx, dy, dtheta = delta.x, delta.y, delta.theta
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)

        x_new = self.x + dx * cos_theta - dy * sin_theta
        y_new = self.y + dx * sin_theta + dy * cos_theta
        theta_new = self.theta + dtheta
        return Pose3(x_new, y_new, self._normalize_angle(theta_new))
    
    def between(self, other):
        """Compute relative transformation from this pose to other pose"""
        return self.inverse().compose(other)

    @staticmethod
    def _normalize_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def copy(self):
        """Create a copy of this pose"""
        return Pose3(self.x, self.y, self.theta)


class Edge:
    def __init__(self, from_idx, to_idx, measurement: Pose3, information: np.ndarray, edge_type="odometry"):
        self.from_idx = from_idx
        self.to_idx = to_idx
        self.measurement = measurement  # Relative pose measurement
        self.information = information  # 3x3 information matrix
        self.edge_type = edge_type  # "odometry" or "loop_closure"
        self._scan = []  
        self._timestamp = None
        self._confidence = 1.0  # Confidence score for loop closures
    
    def is_loop_closure(self):
        """Check if this is a loop closure edge"""
        return self.edge_type == "loop_closure"
    
    def is_odometry(self):
        """Check if this is an odometry edge"""
        return self.edge_type == "odometry"
    
    def set_confidence(self, confidence):
        """Set confidence score (0-1) for loop closure edges"""
        self._confidence = max(0.0, min(1.0, confidence))
    
    def get_confidence(self):
        """Get confidence score"""
        return self._confidence

    def get_scan(self):
        """Getter for laser scans between src → tgt."""
        return self._scan

    def set_scan(self, scan):
        """Setter for replacing scan list."""
        self._scan = scan.copy() if scan else []
    
    def set_timestamp(self, timestamp):
        """Set timestamp for this edge"""
        self._timestamp = timestamp
    
    def get_timestamp(self):
        """Get timestamp of this edge"""
        return self._timestamp
    
    def get_covariance(self):
        """Get covariance matrix (inverse of information matrix)"""
        try:
            return np.linalg.inv(self.information)
        except np.linalg.LinAlgError:
            # Return high uncertainty if matrix is singular
            return np.diag([1.0, 1.0, 1.0])
    
    def update_information_matrix(self, new_information):
        """Update the information matrix (useful after ICP refinement)"""
        self.information = new_information.copy()
    
    def compute_error(self, pose_from: Pose3, pose_to: Pose3):
        """
        Compute pose error for this edge given actual poses
        Error = predicted_pose^-1 ⊕ actual_pose
        """
        # Predicted pose_to based on measurement
        predicted_pose_to = pose_from.compose(self.measurement)
        
        # Error between predicted and actual
        error_pose = predicted_pose_to.inverse().compose(pose_to)
        return error_pose.as_vector()
    
    def compute_chi2_error(self, pose_from: Pose3, pose_to: Pose3):
        """Compute chi-squared error for this edge"""
        error = self.compute_error(pose_from, pose_to)
        return error.T @ self.information @ error

    def __repr__(self):
        type_str = "LC" if self.is_loop_closure() else "OD"
        conf_str = f", conf={self._confidence:.2f}" if self.is_loop_closure() else ""
        return (f"Edge[{type_str}](from={self.from_idx}, to={self.to_idx}, "
                f"meas={self.measurement}, scans={len(self._scan)}{conf_str})")


class PoseGraph:
    """
    Simple pose graph container for managing nodes and edges
    """
    def __init__(self):
        self.nodes = []  # List[Pose3]
        self.edges = []  # List[Edge]
        self.node_scans = []  # List[List] - scans for each node
        self.node_timestamps = []  # List[float] - timestamps for each node
    
    def add_node(self, pose: Pose3, scan=None, timestamp=None):
        """Add a new node to the graph"""
        node_id = len(self.nodes)
        self.nodes.append(pose.copy())
        self.node_scans.append(scan.copy() if scan else [])
        self.node_timestamps.append(timestamp)
        return node_id
    
    def add_edge(self, edge: Edge):
        """Add an edge to the graph"""
        if edge.from_idx < len(self.nodes) and edge.to_idx < len(self.nodes):
            self.edges.append(edge)
            return True
        return False
    
    def get_node(self, idx):
        """Get node by index"""
        if 0 <= idx < len(self.nodes):
            return self.nodes[idx]
        return None
    
    def get_node_scan(self, idx):
        """Get scan for node by index"""
        if 0 <= idx < len(self.node_scans):
            return self.node_scans[idx]
        return []
    
    def update_node(self, idx, new_pose: Pose3):
        """Update node pose (used during optimization)"""
        if 0 <= idx < len(self.nodes):
            self.nodes[idx] = new_pose.copy()
    
    def get_odometry_edges(self):
        """Get all odometry edges"""
        return [edge for edge in self.edges if edge.is_odometry()]
    
    def get_loop_closure_edges(self):
        """Get all loop closure edges"""
        return [edge for edge in self.edges if edge.is_loop_closure()]
    
    def find_nearby_nodes(self, pose: Pose3, distance_threshold, min_separation=0):
        """Find nodes within distance threshold, excluding recent ones"""
        nearby = []
        for i, node_pose in enumerate(self.nodes):
            if len(self.nodes) - i > min_separation:  # Exclude recent nodes
                if pose.distance_to(node_pose) <= distance_threshold:
                    nearby.append(i)
        return nearby
    
    def compute_total_error(self):
        """Compute total chi-squared error of the graph"""
        total_error = 0.0
        for edge in self.edges:
            if edge.from_idx < len(self.nodes) and edge.to_idx < len(self.nodes):
                pose_from = self.nodes[edge.from_idx]
                pose_to = self.nodes[edge.to_idx]
                total_error += edge.compute_chi2_error(pose_from, pose_to)
        return total_error
    
    def get_statistics(self):
        """Get graph statistics"""
        num_nodes = len(self.nodes)
        num_odom_edges = len(self.get_odometry_edges())
        num_loop_edges = len(self.get_loop_closure_edges())
        total_error = self.compute_total_error()
        
        return {
            'nodes': num_nodes,
            'odometry_edges': num_odom_edges,
            'loop_closure_edges': num_loop_edges,
            'total_edges': len(self.edges),
            'total_chi2_error': total_error
        }
    
    def __repr__(self):
        stats = self.get_statistics()
        return (f"PoseGraph(nodes={stats['nodes']}, "
                f"odom_edges={stats['odometry_edges']}, "
                f"loop_edges={stats['loop_closure_edges']}, "
                f"error={stats['total_chi2_error']:.2f})")