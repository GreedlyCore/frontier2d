#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from scipy import ndimage
import cv2

# ROS2 message types
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')
        
        self.declare_parameter('update_rate', 0.5)  # in Hz
        self.declare_parameter('min_frontier_size', 10)  # minimum frontier length size
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/frontier_markers')
        self.declare_parameter('frontier_frame_id', 'map')
        
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.frontier_frame_id = self.get_parameter('frontier_frame_id').get_parameter_value().string_value
        
        self.get_logger().info(f'Frontier exploration node started')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Min frontier size: {self.min_frontier_size} cells')
        self.get_logger().info(f'Map topic: {self.map_topic}')
        self.get_logger().info(f'Marker topic: {self.marker_topic}')
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )
        
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )
        
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_frontiers)
        
        # State variables
        self.current_map = None
        self.last_markers_count = 0
        
        self.get_logger().info('Waiting for map data...')
    
    def map_callback(self, msg):
        """Callback for receiving occupancy grid map"""
        self.current_map = msg
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}')
    
    def occupancy_grid_to_numpy(self, occupancy_grid):
        """Convert ROS OccupancyGrid to numpy array"""
        # OccupancyGrid data: 0-100 (0=free, 100=occupied, -1=unknown)
        # Convert to our format: 254=free, 0=occupied, 205=unknown
        data = np.array(occupancy_grid.data, dtype=np.int32)
        data = data.reshape((occupancy_grid.info.height, occupancy_grid.info.width))
        
        # Convert values
        # -1 (unknown) -> 205
        # 0-49 (free) -> 254  
        # 50-100 (occupied) -> 0
        converted = np.zeros_like(data, dtype=np.uint8)
        converted[data == -1] = 205  # unknown
        converted[(data >= 0) & (data < 50)] = 254  # free
        converted[data >= 50] = 0  # occupied
        
        return converted
    
    def find_frontiers(self, grid_map):
        """
        Find frontier cells in the occupancy grid map
        """
        # Create binary masks for different cell types
        free_cells = (grid_map == 254)
        unknown_cells = (grid_map == 205)
        
        # Define 8-connectivity kernel for neighborhood searching
        kernel = np.array([[1, 1, 1],
                          [1, 1, 1],
                          [1, 1, 1]], dtype=np.uint8)
        
        # Find unknown cells adjacent to free cells
        unknown_dilated = ndimage.binary_dilation(unknown_cells, kernel)
        
        # Frontier cells are free cells that are adjacent to unknown cells
        frontier_cells = free_cells & unknown_dilated
        
        return frontier_cells
    
    def find_frontier_centroids(self, frontier_cells, min_size=10):
        """Find centroids of connected frontier regions that meet minimum size threshold"""
        # Label connected components of frontier cells
        labeled_frontiers, num_features = ndimage.label(frontier_cells)
        
        centroids = []
        filtered_frontiers = np.zeros_like(frontier_cells, dtype=bool)
        frontier_regions = []
        
        for i in range(1, num_features + 1):
            # Find all cells belonging to this frontier region
            region_mask = (labeled_frontiers == i)
            region_size = np.sum(region_mask)
            
            # Only process regions that meet minimum size threshold
            if region_size >= min_size:
                region_coords = np.where(region_mask)
                
                # Calculate centroid
                centroid_row = np.mean(region_coords[0])
                centroid_col = np.mean(region_coords[1])
                
                centroids.append((int(round(centroid_row)), int(round(centroid_col))))
                
                # Add this region to filtered frontiers
                filtered_frontiers |= region_mask
                
                # Store frontier points for visualization
                frontier_points = []
                for row, col in zip(region_coords[0], region_coords[1]):
                    frontier_points.append((row, col))
                frontier_regions.append(frontier_points)
        
        return centroids, filtered_frontiers, frontier_regions
    
    def grid_to_world_coordinates(self, grid_row, grid_col, map_info):
        """Convert grid coordinates to world coordinates"""
        # ROS occupancy grid origin is at bottom-left, but numpy array origin is top-left
        # So we need to flip the row coordinate
        world_x = map_info.origin.position.x + grid_col * map_info.resolution
        world_y = map_info.origin.position.y + (map_info.height - 1 - grid_row) * map_info.resolution
        return world_x, world_y
    
    def create_frontier_markers(self, centroids, frontier_regions, map_info):
        """Create visualization markers for frontiers and centroids"""
        markers = MarkerArray()
        blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker_id = 0
        
        for i, (centroid, frontier_points) in enumerate(zip(centroids, frontier_regions)):
            # Marker for frontier points (blue)
            frontier_marker = Marker()
            frontier_marker.header.frame_id = self.frontier_frame_id
            frontier_marker.header.stamp = self.get_clock().now().to_msg()
            frontier_marker.ns = "frontiers"
            frontier_marker.id = marker_id
            frontier_marker.type = Marker.POINTS
            frontier_marker.action = Marker.ADD
            frontier_marker.scale.x = 0.05  # point size
            frontier_marker.scale.y = 0.05
            frontier_marker.scale.z = 0.05
            frontier_marker.color = blue
            frontier_marker.lifetime.sec = 0  # persistent
            
            # Convert frontier points to world coordinates
            for row, col in frontier_points:
                world_x, world_y = self.grid_to_world_coordinates(row, col, map_info)
                point = Point()
                point.x = world_x
                point.y = world_y
                point.z = 0.0
                frontier_marker.points.append(point)
            
            markers.markers.append(frontier_marker)
            marker_id += 1
            
            # Marker for centroid (green sphere)
            centroid_marker = Marker()
            centroid_marker.header.frame_id = self.frontier_frame_id
            centroid_marker.header.stamp = self.get_clock().now().to_msg()
            centroid_marker.ns = "frontiers"
            centroid_marker.id = marker_id
            centroid_marker.type = Marker.SPHERE
            centroid_marker.action = Marker.ADD
            centroid_marker.scale.x = 0.3  # sphere size
            centroid_marker.scale.y = 0.3
            centroid_marker.scale.z = 0.3
            centroid_marker.color = green
            centroid_marker.lifetime.sec = 0  # persistent
            
            # Convert centroid to world coordinates
            world_x, world_y = self.grid_to_world_coordinates(centroid[0], centroid[1], map_info)
            centroid_marker.pose.position.x = world_x
            centroid_marker.pose.position.y = world_y
            centroid_marker.pose.position.z = 0.1  # slightly above ground
            centroid_marker.pose.orientation.w = 1.0
            
            markers.markers.append(centroid_marker)
            marker_id += 1
        
        # Delete old markers if we have fewer markers now
        current_markers_count = len(markers.markers)
        if current_markers_count < self.last_markers_count:
            for i in range(current_markers_count, self.last_markers_count):
                delete_marker = Marker()
                delete_marker.header.frame_id = self.frontier_frame_id
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.ns = "frontiers"
                delete_marker.id = i
                delete_marker.action = Marker.DELETE
                markers.markers.append(delete_marker)
        
        self.last_markers_count = current_markers_count
        return markers
    
    def update_frontiers(self):
        """Periodic update of frontier detection and visualization"""
        if self.current_map is None:
            return
        
        try:
            # Convert occupancy grid to numpy array
            grid_map = self.occupancy_grid_to_numpy(self.current_map)
            
            # Find frontiers
            frontiers = self.find_frontiers(grid_map)
            frontier_count = np.sum(frontiers)
            
            if frontier_count == 0:
                self.get_logger().debug('No frontiers found')
                # Publish empty marker array to clear previous markers
                empty_markers = MarkerArray()
                self.marker_publisher.publish(empty_markers)
                return
            
            # Find frontier centroids with size filtering
            centroids, filtered_frontiers, frontier_regions = self.find_frontier_centroids(
                frontiers, self.min_frontier_size)
            
            filtered_frontier_count = np.sum(filtered_frontiers)
            
            self.get_logger().debug(
                f'Found {frontier_count} total frontier cells, '
                f'{filtered_frontier_count} after filtering, '
                f'{len(centroids)} regions'
            )
            
            if len(centroids) == 0:
                self.get_logger().debug('No frontier regions meet minimum size threshold')
                # Publish empty marker array
                empty_markers = MarkerArray()
                self.marker_publisher.publish(empty_markers)
                return
            
            # Create and publish visualization markers
            markers = self.create_frontier_markers(centroids, frontier_regions, self.current_map.info)
            self.marker_publisher.publish(markers)
            
            self.get_logger().info(
                f'Published {len(centroids)} frontier regions with {filtered_frontier_count} cells'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in frontier update: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FrontierExplorationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()