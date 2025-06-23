#!/usr/bin/env python3

from typing import List, Tuple, Optional
from queue import Queue
import rclpy
from geometry_msgs.msg import PointStamped, PolygonStamped

def nhood4(idx: int, costmap) -> List[int]:
    """
    Determine 4-connected neighbourhood of an input cell, checking for map edges
    
    Args:
        idx: input cell index
        costmap: Reference to map data
        
    Returns:
        neighbour cell indexes
    """
    # get 4-connected neighbourhood indexes, check for edge of map
    out = []
    
    size_x = costmap.getSizeInCellsX()
    size_y = costmap.getSizeInCellsY()
    
    if idx > size_x * size_y - 1:
        rclpy.logging.get_logger("FrontierExploration").warn(
            "Evaluating nhood for offmap point")
        return out
    
    if idx % size_x > 0:
        out.append(idx - 1)
    if idx % size_x < size_x - 1:
        out.append(idx + 1)
    if idx >= size_x:
        out.append(idx - size_x)
    if idx < size_x * (size_y - 1):
        out.append(idx + size_x)
    
    return out


def nhood8(idx: int, costmap) -> List[int]:
    """
    Determine 8-connected neighbourhood of an input cell, checking for map edges
    
    Args:
        idx: input cell index
        costmap: Reference to map data
        
    Returns:
        neighbour cell indexes
    """
    # get 8-connected neighbourhood indexes, check for edge of map
    out = nhood4(idx, costmap)
    
    size_x = costmap.getSizeInCellsX()
    size_y = costmap.getSizeInCellsY()
    
    if idx > size_x * size_y - 1:
        return out
    
    if idx % size_x > 0 and idx >= size_x:
        out.append(idx - 1 - size_x)
    if idx % size_x > 0 and idx < size_x * (size_y - 1):
        out.append(idx - 1 + size_x)
    if idx % size_x < size_x - 1 and idx >= size_x:
        out.append(idx + 1 - size_x)
    if idx % size_x < size_x - 1 and idx < size_x * (size_y - 1):
        out.append(idx + 1 + size_x)
    
    return out


def nearest_cell(start: int, val: int, costmap) -> Tuple[bool, Optional[int]]:
    """
    Find nearest cell of a specified value
    
    Args:
        start: Index initial cell to search from
        val: Specified value to search for
        costmap: Reference to map data
        
    Returns:
        Tuple of (success: bool, result: Optional[int])
        - success: True if a cell with the requested value was found
        - result: Index of located cell (None if not found)
    """
    map_data = costmap.getMap()
    size_x = costmap.getSizeInCellsX()
    size_y = costmap.getSizeInCellsY()
    
    if start >= size_x * size_y:
        return False, None
    
    # initialize breadth first search
    bfs = Queue()
    visited_flag = [False] * (size_x * size_y)
    
    # push initial cell
    bfs.put(start)
    visited_flag[start] = True
    
    # search for neighbouring cell matching value
    while not bfs.empty():
        idx = bfs.get()
        
        # return if cell of correct value is found
        if map_data[idx] == val:
            return True, idx
        
        # iterate over all adjacent unvisited cells
        for nbr in nhood8(idx, costmap):
            if not visited_flag[nbr]:
                bfs.put(nbr)
                visited_flag[nbr] = True
    
    return False, None