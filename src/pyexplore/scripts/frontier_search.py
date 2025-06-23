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

from tools import nearest_cell, nhood4, nhood8

# Defined in nav2_costmap_2d
# https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp
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
        
        # These will be set during search
        self.map = None
        self.size_x = 0
        self.size_y = 0

    def search_from(self, position: Point) -> List[Frontier]:
        """
        Search for frontiers from given position using dilate-based approach
        
        Args:
            position: geometry_msgs::msg::Point
            
        Returns:
            List of Frontier objects
        """
        frontier_list = []
        
        # Sanity check that robot is inside costmap bounds before searching
        mx, my = self.costmap.worldToMap(position.x, position.y)
        if mx is None or my is None:
            rclpy.logging.get_logger("FrontierSearch").error(
                "Robot out of costmap bounds, cannot search for frontiers")
            return frontier_list
        
        self.map = self.costmap.getMap()
        self.size_x = self.costmap.getSizeInCellsX()
        self.size_y = self.costmap.getSizeInCellsY()
        
        # Convert 1D map to 2D grid for easier processing
        y = self.costmap.getSizeInCellsY()
        x = self.costmap.getSizeInCellsX()
        grid_map = np.array(self.map).reshape(y, x)
        
        
        # Find frontier cells using dilate-based approach
        frontier_cells = self.find_frontiers_dilate(grid_map)
    
        # import matplotlib.pyplot as plt
        # plt.imshow(frontier_cells, cmap='gray', interpolation='nearest')
        # plt.axis('off')  # optional, removes axis ticks
        # plt.show()
        
        # Find connected frontier regions and their centroids
        centroids, filtered_frontiers = self.find_frontier_regions(
            frontier_cells, 
            min_size=max(1, int(self.min_frontier_size / self.costmap.getResolution()))
        )
        
        # Convert frontier regions to Frontier objects
        robot_pos = (mx, my)
        frontier_list = self.build_frontiers_from_regions(
            filtered_frontiers, centroids, robot_pos
        )
        
        # Set costs of frontiers
        for frontier in frontier_list:
            frontier.cost = self.frontier_cost(frontier)
        
        # Sort frontiers by cost
        frontier_list.sort(key=lambda f: f.cost)
        
        return frontier_list
    
    def find_frontiers_dilate(self, grid_map: np.ndarray) -> np.ndarray:
        """
        Find frontier cells using morphological dilation
        
        Args:
            grid_map: 2D numpy array representing the occupancy grid
            
        Returns:
            Binary array where True indicates frontier cells
        """
        # Create binary masks for different cell types
        # free_cells = (grid_map > 60)
        # unknown_cells = (grid_map == -1)
        free_cells = (grid_map == FREE_SPACE)
        unknown_cells = (grid_map == NO_INFORMATION)
        

        # kernel4 = np.array([[0, 1, 0],
        #                     [1, 1, 1],
        #                     [0, 1, 0]], dtype=np.uint8)
        kernel8 = np.array([[1, 1, 1],
                            [1, 1, 1],
                            [1, 1, 1]], dtype=np.uint8)
        
        # Find unknown cells adjacent to free cells
        # Dilate unknown cells to find their neighbors
        unknown_dilated = ndimage.binary_dilation(unknown_cells, kernel8)
        frontier_cells = free_cells & unknown_dilated
        return frontier_cells

        # TODO: essential ?
        # frontier_uint8 = (frontier_cells.astype(np.uint8)) * 255
        # kernel = np.ones((9, 9), np.uint8)
        # # cv2.MORPH_OPEN || cv2.MORPH_CLOSE
        # # closed = cv2.morphologyEx(frontier_uint8, cv2.MORPH_OPEN, kernel)
        
        # return closed 
    
 
    
    def find_frontier_regions(self, frontier_cells: np.ndarray, min_size: int = 1):
        """
        Find connected frontier regions that meet minimum size threshold
        
        Args:
            frontier_cells: binary array where True indicates frontier cells
            min_size: minimum number of cells required for a frontier region
            
        Returns:
            Tuple of (centroids, filtered_frontiers)
        """
        # Define 8-connectivity for connected component labeling
        kernel_8 = np.array([[1, 1, 1],
                            [1, 1, 1],
                            [1, 1, 1]], dtype=np.uint8)
        
        # Label connected components of frontier cells
        labeled_frontiers, num_features = ndimage.label(frontier_cells)
        
        centroids = []
        filtered_frontiers = np.zeros_like(frontier_cells, dtype=bool)
        
        for i in range(1, num_features + 1):
            # Find all cells belonging to this frontier region
            region_mask = (labeled_frontiers == i)
            region_size = np.sum(region_mask)
            
            # Only process regions that meet minimum size threshold
            # print(region_size)
            # if region_size >= 20: #min_size:
            region_coords = np.where(region_mask)            
            centroid_row = np.mean(region_coords[0])
            centroid_col = np.mean(region_coords[1])
            centroids.append((int(round(centroid_row)), int(round(centroid_col))))
            filtered_frontiers |= region_mask
        
        return centroids, filtered_frontiers
    
    def build_frontiers_from_regions(self, frontier_mask: np.ndarray, 
                                   centroids: List[tuple], 
                                   robot_pos: tuple) -> List[Frontier]:
        """
        Build Frontier objects from frontier regions
        
        Args:
            frontier_mask: binary array indicating frontier cells
            centroids: list of (row, col) centroid coordinates
            robot_pos: (x, y) robot position in map coordinates
            
        Returns:
            List of Frontier objects
        """
        frontiers = []
        
        # Define 8-connectivity kernel for region growing
        kernel_8 = np.array([[1, 1, 1],
                            [1, 1, 1],
                            [1, 1, 1]], dtype=np.uint8)
        
        # Label connected components again to separate regions
        labeled_regions, _ = ndimage.label(frontier_mask, structure=kernel_8)
        
        # Get robot position in world coordinates for distance calculations
        robot_wx, robot_wy = self.costmap.mapToWorld(robot_pos[0], robot_pos[1])
        
        for i, (centroid_row, centroid_col) in enumerate(centroids):
            # Find the region label at this centroid
            region_label = labeled_regions[centroid_row, centroid_col]
            if region_label == 0:
                continue
            
            # Get all cells in this region
            region_mask = (labeled_regions == region_label)
            region_coords = np.where(region_mask)
            
            # Create Frontier object
            frontier = Frontier()
            frontier.size = len(region_coords[0])
            frontier.points = []
            
            # Convert centroid to world coordinates
            centroid_wx, centroid_wy = self.costmap.mapToWorld(centroid_col, centroid_row)
            frontier.centroid.x = centroid_wx
            frontier.centroid.y = centroid_wy
            
            # Find initial contact point (first cell found)
            initial_row, initial_col = region_coords[0][0], region_coords[1][0]
            initial_wx, initial_wy = self.costmap.mapToWorld(initial_col, initial_row)
            frontier.initial.x = initial_wx
            frontier.initial.y = initial_wy
            
            # Process all points in the frontier
            min_distance = float('inf')
            closest_wx, closest_wy = initial_wx, initial_wy
            
            for row, col in zip(region_coords[0], region_coords[1]):
                # Convert to world coordinates
                wx, wy = self.costmap.mapToWorld(col, row)
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
            
            frontiers.append(frontier)
        
        return frontiers
    
    def frontier_cost(self, frontier: Frontier) -> float:
        """
        Calculate cost of frontier
        
        Args:
            frontier: Frontier object
            
        Returns:
            float - frontier cost
        """
        return (self.potential_scale * frontier.min_distance * 
                self.costmap.getResolution()) - \
               (self.gain_scale * frontier.size * self.costmap.getResolution())
    
    def nearest_cell(self, pos: int, target_value: int) -> int:
        """
        Find nearest cell with target value 
        
        Args:
            pos: int - starting position index
            target_value: int - target cell value to find
            
        Returns:
            int - index of nearest cell, or None if not found
        """
        return nearest_cell(pos, target_value, self.costmap)
        
    
    def nhood4(self, idx: int) -> List[int]:
        """
        Get 4-connected neighborhood indices
        This would need to be implemented based on costmap_tools
        
        Args:
            idx: int - center cell index
            
        Returns:
            List[int] - list of neighbor indices
        """
        return nhood4(idx, self.costmap)
    
    def nhood8(self, idx: int) -> List[int]:
        """
        Get 8-connected neighborhood indices
        This would need to be implemented based on costmap_tools
        
        Args:
            idx: int - center cell index
            
        Returns:
            List[int] - list of neighbor indices
        """
        return nhood8(idx, self.costmap)
