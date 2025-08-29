#!/usr/bin/env python3

import numpy as np
import threading
import cv2
from queue import Queue
from typing import List, Tuple
import rclpy
from geometry_msgs.msg import Point
import numpy as np

from scipy import ndimage
from geometry_msgs.msg import Point

# from tools import nearest_cell, nhood4, nhood8

# ROS2 Costmap values (different from PGM!)
FREE_SPACE = 0
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255

class Frontier:
    """
    Represents a frontier structure
    """
    def __init__(self):
        self.centroid = Point()
        self.initial = Point()
        self.middle = Point()
        self.points = []
        self.size = 0
        self.min_distance = float('inf')
        self.cost = 0.0

class FrontierSearch:
    def __init__(self, costmap, potential_scale, gain_scale, min_frontier_size):
        """
        Initialize frontier search
        
        Args:
            costmap: nav2_costmap_2d::Costmap2D* equivalent
            potential_scale: double
            gain_scale: double
            min_frontier_size: double
        """
        self.costmap = costmap
        self.potential_scale = potential_scale
        self.gain_scale = gain_scale
        self.min_frontier_size = min_frontier_size
        
        # Thread safety for dynamic map updates
        self.map_lock = threading.Lock()
        
        # These will be set during search
        self.map = None
        self.size_x = 0
        self.size_y = 0
        
        self.num_features = 0

    def search_from(self, position: Point) -> List[Frontier]:
        """
        Search for frontiers from given position using dilate-based approach
        Thread-safe version for dynamic map updates
        
        Args:
            position: geometry_msgs::msg::Point
            
        Returns:
            List of Frontier objects
        """
        frontier_list = []
        
        # Thread-safe map access
        with self.map_lock:
            # Sanity check that robot is inside costmap bounds before searching
            mx, my = self.costmap.worldToMap(position.x, position.y)
            if mx is None or my is None:
                rclpy.logging.get_logger("FrontierSearch").error(
                    "Robot out of costmap bounds, cannot search for frontiers")
                return frontier_list
            
            self.map = self.costmap.getMap().copy()  # ← Important: copy()
            self.size_x = self.costmap.getSizeInCellsX()
            self.size_y = self.costmap.getSizeInCellsY()
            # Store robot position in map coordinates
            robot_pos = (mx, my)
        
        # Convert 1D map to 2D grid for easier processing
        grid_map = np.array(self.map).reshape(self.size_y, self.size_x)
        
        # Find frontier cells using dilate-based approach
        frontier_cells = self.find_frontiers_dilate(grid_map)
        
        # Find connected frontier regions and their centroids
        centroids, filtered_frontiers, num_features = self.find_frontier_regions(
            frontier_cells, 
            min_size=max(1, int(self.min_frontier_size / self.costmap.getResolution()))
        )
        
        self.num_features = num_features
        
        # Convert frontier regions to Frontier objects
        frontier_list = self.build_frontiers_from_regions(
            filtered_frontiers, centroids, robot_pos
        )
        
        # Set costs of frontiers
        for frontier in frontier_list:
            frontier.cost = self.frontier_cost(frontier)
        
        # Sort frontiers by cost
        frontier_list.sort(key=lambda f: f.cost)
        
        return frontier_list, frontier_cells
    
    def find_frontiers_dilate(self, grid_map: np.ndarray) -> np.ndarray:
        """
        Find frontier cells using morphological dilation
        
        Args:
            grid_map: 2D numpy array representing the occupancy grid
            
        Returns:
            Binary array where True indicates frontier cells
        """
        # Create binary masks for different cell types (ROS costmap values)
        free_cells = (grid_map == FREE_SPACE)  # 0 in ROS costmaps
        unknown_cells = (grid_map == NO_INFORMATION)  # 255 in ROS costmaps
        
        # Optional: Add some tolerance for near-free cells
        # This helps with real-world noise in costmaps
        near_free_threshold = 50  # Adjustable parameter
        free_cells = free_cells | (grid_map < near_free_threshold)
        
        # 8-connectivity kernel
        kernel8 = np.array([[1, 1, 1],
                            [1, 1, 1],
                            [1, 1, 1]], dtype=np.uint8)
        
        # Find unknown cells adjacent to free cells
        unknown_dilated = ndimage.binary_dilation(unknown_cells, kernel8)
        
        # Frontier cells are free cells adjacent to unknown cells
        frontier_cells = free_cells & unknown_dilated
        
        # Additional filtering to remove noise and isolated pixels
        # frontier_cells = self.filter(frontier_cells)
        
        return frontier_cells

    def filter(self, frontier_cells: np.ndarray) -> np.ndarray:
        kernel_small = np.array([[1, 1, 1],
                                [1, 1, 1],
                                [1, 1, 1]], dtype=np.uint8)
        # Opening operation: erosion followed by dilation
        # This removes small noise while preserving larger structures
        frontier_uint8 = (frontier_cells.astype(np.uint8)) * 255
        cleaned = cv2.morphologyEx(frontier_uint8, cv2.MORPH_OPEN, kernel_small)
        return cleaned.astype(bool)
    
    def find_frontier_regions(self, frontier_cells: np.ndarray, min_size: int = 1):
        """
        Find connected frontier regions that meet minimum size threshold
        Improved version with better region validation
        
        Args:
            frontier_cells: binary array where True indicates frontier cells
            min_size: minimum number of cells required for a frontier region
            
        Returns:
            Tuple of (centroids, filtered_frontiers)
        """
        # Label connected components of frontier cells
        labeled_frontiers, num_features = ndimage.label(frontier_cells)
        
        centroids = []
        filtered_frontiers = np.zeros_like(frontier_cells, dtype=bool)
        
        # More conservative minimum size to avoid tiny frontiers
        effective_min_size = max(min_size, 5)
        
        for i in range(1, num_features + 1):
            region_mask = (labeled_frontiers == i)
            region_size = np.sum(region_mask)
            
            if region_size >= effective_min_size:
                region_coords = np.where(region_mask)
                
                # Calculate centroid
                centroid_row = np.mean(region_coords[0])
                centroid_col = np.mean(region_coords[1])
                
                # Store as (x, y) in map coordinates
                centroids.append((int(round(centroid_col)), int(round(centroid_row))))
                filtered_frontiers |= region_mask
            
        return centroids, filtered_frontiers, num_features
    
    def build_frontiers_from_regions(self, frontier_mask: np.ndarray, 
                                centroids: List[tuple], 
                                robot_pos: tuple) -> List[Frontier]:
        """
        Build Frontier objects from frontier regions
        Fixed coordinate handling and thread safety
        
        Args:
            frontier_mask: binary array indicating frontier cells
            centroids: list of (col, row) centroid coordinates in map frame
            robot_pos: (x, y) robot position in map coordinates
            
        Returns:
            List of Frontier objects
        """
        frontiers = []
        
        if not centroids:
            return frontiers
        
        # Label connected components again to separate regions
        labeled_regions, _ = ndimage.label(frontier_mask)
        
        # Get robot position in world coordinates for distance calculations
        robot_wx, robot_wy = self.costmap.mapToWorld(robot_pos[0], robot_pos[1])
        
        for centroid_col, centroid_row in centroids:
            # Bounds checking
            if (centroid_row < 0 or centroid_row >= self.size_y or 
                centroid_col < 0 or centroid_col >= self.size_x):
                continue
                
            # Find the region label at this centroid
            region_label = labeled_regions[centroid_row, centroid_col]
            if region_label == 0:
                continue
            
            # Get all cells in this region
            region_mask = (labeled_regions == region_label)
            region_coords = np.where(region_mask)
            
            if len(region_coords[0]) == 0:
                continue
            
            # Create Frontier object
            frontier = Frontier()
            frontier.size = len(region_coords[0])
            frontier.points = []
            
            # Convert centroid to world coordinates
            centroid_wx, centroid_wy = self.costmap.mapToWorld(centroid_col, centroid_row)
            if centroid_wx is None or centroid_wy is None:
                continue
                
            frontier.centroid.x = centroid_wx
            frontier.centroid.y = centroid_wy
            
            # Find initial contact point (first cell found)
            initial_row, initial_col = region_coords[0][0], region_coords[1][0]
            initial_wx, initial_wy = self.costmap.mapToWorld(initial_col, initial_row)
            if initial_wx is None or initial_wy is None:
                continue
                
            frontier.initial.x = initial_wx
            frontier.initial.y = initial_wy
            
            # Process all points in the frontier
            min_distance = float('inf')
            closest_wx, closest_wy = initial_wx, initial_wy
            
            for row, col in zip(region_coords[0], region_coords[1]):
                # Bounds checking
                if row < 0 or row >= self.size_y or col < 0 or col >= self.size_x:
                    continue
                    
                # Convert to world coordinates
                wx, wy = self.costmap.mapToWorld(col, row)
                if wx is None or wy is None:
                    continue
                    
                # Add to points list
                point = Point()
                point.x = wx
                point.y = wy
                frontier.points.append(point)
                
                # Check if this is the closest point to robot
                distance = np.sqrt((robot_wx - wx) ** 2 + (robot_wy - wy) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    closest_wx, closest_wy = wx, wy
            
            frontier.min_distance = min_distance
            frontier.middle.x = closest_wx
            frontier.middle.y = closest_wy
            
            # Only add frontier if it has valid points
            if frontier.points:
                frontiers.append(frontier)
        
        return frontiers
    
    def frontier_cost(self, frontier: Frontier) -> float:
        """
        Calculate cost of frontier
        
        potential*min_dist*resolution - gain*size*resolution
        
        Args:
            frontier: Frontier object
            
        Returns:
            float - frontier cost
        """
        return self.costmap.getResolution() * ( (self.potential_scale * frontier.min_distance) - (self.gain_scale * frontier.size) )