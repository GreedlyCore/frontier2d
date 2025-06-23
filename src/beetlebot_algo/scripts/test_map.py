#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point

import numpy as np

class TestMapAndMarkersNode(Node):
    def __init__(self):
        super().__init__('test_map_and_markers_node')

        self.map_pub = self.create_publisher(OccupancyGrid, '/test_map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/test_markers', 10)

        # Timer for publishing at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_map_and_markers)

        # Map parameters
        self.width = 10
        self.height = 10
        self.resolution = 1.0  # 1 meter per cell (arbitrary)
        self.origin_x = -5.0
        self.origin_y = -5.0

        # Prepare static numpy map (20x20 zeros)
        self.grid = np.ones((self.height, self.width), dtype=np.int8)  # free space = 0
        
        self.grid[0, 0] = 100       # occupied
        self.grid[1, 1] = 100       # occupied
        self.grid[9, 9] = 100       # occupied
        self.grid[4, 3] = 100       # occupied
        self.grid[1, 2] = 100       # occupied
        # self.grid[0, 1] = 100     # occupied

        # Define 5 points in grid coordinates (row, col) to place markers
        # Change these as you want
        self.marker_points = [
            (-10, -10),
            (-5, -5),
            # (2, 3),
            # (5, 15),
            (5, 5),
            (9, 9),
            (10, 10),
            # (17, 7),
            # (19, 19)
        ]

    # def occupancy_grid_to_numpy(self, point):
        # pass  
    
    def grid_to_world_coordinates(self, grid_row, grid_col):
        """
        Convert grid coordinates to world coordinates
        
        ROS2 occupancy grid data is stored row-major order
        Grid origin (0,0) in map frame corresponds to map_info.origin
        Grid coordinates: (0,0) is top-left of numpy array
        World coordinates: map_info.origin is bottom-left of the grid
        
        Convert from numpy array coordinates to world coordinates
        Add 0.5 to center the point in the cell
        
        """
        
        world_x = self.origin_x + (grid_col + 0.5) * self.resolution
        world_y = self.origin_y + (self.height - grid_row - 0.5) * self.resolution
        
        # self.get_logger().info(f'mid of the map: ({map_info.origin.position.x, map_info.origin.position.y})')
        return world_x, world_y
    
    def publish_map_and_markers(self):
        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flatten numpy map row-major and convert to list
        # OccupancyGrid expects int8 with values [0..100] or -1 for unknown
        msg.data = [int(x) for x in self.grid.flatten()]

        self.map_pub.publish(msg)

        # Create MarkerArray for 5 points
        marker_array = MarkerArray()
        
        row, col = 0.0 , 0.0
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'test_points'
        marker.id = 101
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.origin_x + (col + 0.5) * self.resolution
        marker.pose.position.y = self.origin_y + (self.height - row - 0.5) * self.resolution
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker.lifetime.sec = 0  # forever

        marker_array.markers.append(marker)
        
        row, col = 1 , 2
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'test_points'
        marker.id = 104
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.origin_x + (col + 0.5) * self.resolution
        marker.pose.position.y = self.origin_y + (self.height - row - 0.5) * self.resolution
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.5, a=1.0)
        marker.lifetime.sec = 0  # forever

        marker_array.markers.append(marker)
        
        # world_x, world_y = self.grid_to_world_coordinates(centroid[0], centroid[1], map_info)        
        
        row, col = 0.0 , 0.0
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'test_points'
        marker.id = 100
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0 # ??
        marker.pose.position.y = 0.0 # ??
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        marker.lifetime.sec = 0  # forever

        marker_array.markers.append(marker)
        
        
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'test_points'
        marker.id = 102
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0 # ??
        marker.pose.position.y = 1.0 # ??
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        marker.lifetime.sec = 0  # forever

        marker_array.markers.append(marker)
        
        for i, (row, col) in enumerate(self.marker_points):
            marker = Marker()
            marker.header = msg.header
            marker.ns = 'test_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.origin_x + (col + 0.5) * self.resolution
            marker.pose.position.y = self.origin_y + (self.height - row - 0.5) * self.resolution
            marker.pose.position.z = 0.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker.lifetime.sec = 0  # forever

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TestMapAndMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
