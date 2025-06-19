#!/usr/bin/env python3
import numpy as np

class Pose3:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def as_vector(self):
        return np.array([self.x, self.y, self.theta])

    def __sub__(self, other):
        return Pose3(self.x - other.x, self.y - other.y, self.theta - other.theta)

    def __repr__(self):
        return f"Pose3: (x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}) "

    def compose(self, delta):
        """Pose composition: x ⊕ u"""
        dx, dy, dtheta = delta.x, delta.y, delta.theta
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)

        x_new = self.x + dx * cos_theta - dy * sin_theta
        y_new = self.y + dx * sin_theta + dy * cos_theta
        theta_new = self.theta + dtheta
        return Pose3(x_new, y_new, self._normalize_angle(theta_new))

    @staticmethod
    def _normalize_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))


class Edge:
    def __init__(self, from_idx, to_idx, measurement: Pose3, information: np.ndarray):
        self.from_idx = from_idx
        self.to_idx = to_idx
        self.measurement = measurement
        self.information = information  # 3x3 numpy matrix
        self._scan = []  

    def get_scan(self):
        """Getter for laser scans between src → tgt."""
        return self._scan

    def set_scan(self, scan):
        """Setter for replacing scan list."""
        self._scan = scan

    # def add_scan(self, scan):
    #     """Append a single scan to the list."""
    #     self._scan.append(scan)

    def __repr__(self):
        return (f"Edge(from={self.from_idx}, to={self.to_idx}, meas={self.measurement}, "
                f"scans={len(self._scan)})")

