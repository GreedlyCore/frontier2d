#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
import numpy as np

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformException, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

# from nav2_simple_commander.costmap_2d import PyCostmap2D
# updated a bit that dependency:
from costmap_2d import PyCostmap2D

def init_translation_table():
    """Initialize translation table to speed things up"""
    cost_translation_table = [0] * 256
    
    # lineary mapped from [0..100] to [0..255]
    for i in range(256):
        cost_translation_table[i] = int(1 + (251 * (i - 1)) / 97)
    
    # special values:
    cost_translation_table[0] = 0      # NO obstacle
    cost_translation_table[99] = 253   # INSCRIBED obstacle
    cost_translation_table[100] = 254  # LETHAL obstacle
    cost_translation_table[255] = 255  # UNKNOWN (equivalent to -1 in C++)
    
    return cost_translation_table

# static translation table to speed things up
cost_translation_table__ = init_translation_table()

class Costmap2DClient:
    def __init__(self, node: Node, tf: Buffer):
        self.tf_ = tf
        self.node_ = node
        self.costmap_received_ = False
        self.global_frame_ = ""
        self.robot_base_frame_ = ""
        self.transform_tolerance_ = 0.0
        self.costmap_ = None
        self.mutex_ = threading.Lock()
        
        # Declare parameters
        self.node_.declare_parameter('costmap_topic', 'map')
        self.node_.declare_parameter('costmap_updates_topic', 'map_updates')
        # self.node_.declare_parameter('costmap_topic', 'global_costmap/costmap')
        # self.node_.declare_parameter('costmap_updates_topic', 'global_costmap/costmap_updates')
        self.node_.declare_parameter('robot_base_frame', 'base_link')
        # transform tolerance is used for all tf transforms here
        self.node_.declare_parameter('transform_tolerance', 0.3)
        
        # Get parameters
        costmap_topic = self.node_.get_parameter('costmap_topic').get_parameter_value().string_value
        costmap_updates_topic = self.node_.get_parameter('costmap_updates_topic').get_parameter_value().string_value
        self.robot_base_frame_ = self.node_.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.transform_tolerance_ = self.node_.get_parameter('transform_tolerance').get_parameter_value().double_value
        
        # initialize costmap
        self.costmap_sub_ = self.node_.create_subscription(OccupancyGrid, costmap_topic, self._costmap_callback, 100)
    
        # Spin some until the callback gets called to replicate
        self.node_.get_logger().info(
            f"Waiting for costmap to become available, topic: {costmap_topic}"
        )
          
        # Create a future that will be set when costmap is received
        # https://robotics.stackexchange.com/questions/91278/ros2-correct-usage-of-spin-once-to-receive-multiple-callbacks-synchronized
        future = rclpy.task.Future()
        self.costmap_future = future
        try:
            rclpy.spin_until_future_complete(
                self.node_, 
                future, 
                timeout_sec=30.0
            )
            if not future.done():
                self.node_.get_logger().error(
                    f"Timeout waiting for costmap on topic: {costmap_topic}"
                )
                raise RuntimeError("Costmap not received within timeout")     
        except Exception as e:
            self.node_.get_logger().error(f"Error waiting for costmap: {e}")
            raise
        
        # subscribe to map updates
        self.costmap_updates_sub_ = self.node_.create_subscription(
            OccupancyGridUpdate,
            costmap_updates_topic,
            self._costmap_updates_callback,
            1000
        )
        
        # TF transform is necessary for get_robot_pose method
        last_error = self.node_.get_clock().now()
        tf_error = ""
        while rclpy.ok():
            try:
                # Check if transform is available
                if self.tf_.can_transform(
                    self.global_frame_, 
                    self.robot_base_frame_, 
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                ):
                    break
            except Exception as e:
                tf_error = str(e)
            
            rclpy.spin_once(self.node_, timeout_sec=0.1)
            
            current_time = self.node_.get_clock().now()
            if (current_time - last_error).nanoseconds > 5.0 * 1e9:  # 5 seconds
                self.node_.get_logger().warn(
                    f"Timed out waiting for transform from {self.robot_base_frame_} "
                    f"to {self.global_frame_} to become available before subscribing "
                    f"to costmap, tf error: {tf_error}"
                )
                last_error = current_time
            
            # The error string will accumulate and errors will typically be the same,
            # so the last will do for the warning above. 
            # Reset the string here to avoid accumulation.
            tf_error = ""
        self.node_.get_logger().info(f"Costmap2D client node is initted")
        
    def get_costmap(self):
        """Get the costmap object"""
        return self.costmap_
    def _costmap_callback(self, msg: OccupancyGrid):
        """Callback for full costmap updates"""
        if not self.costmap_:
            self.costmap_ = PyCostmap2D(msg)  # Using nav2's Python costmap wrapper
        self.costmap_received_ = True
        # Set the future to complete the wait
        if hasattr(self, 'costmap_future') and not self.costmap_future.done():
            self.costmap_future.set_result(True)
        self.update_full_map(msg)
    
    def _costmap_updates_callback(self, msg: OccupancyGridUpdate):
        """Callback for partial costmap updates"""
        self.update_partial_map(msg)                
    
    def update_full_map(self, msg: OccupancyGrid):
        """Update the full costmap"""
        self.global_frame_ = msg.header.frame_id
        
        size_in_cells_x = msg.info.width
        size_in_cells_y = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        self.node_.get_logger().debug(
            f"Received full new map, resizing to: {size_in_cells_x}, {size_in_cells_y}"
        )
        
        self.costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x, origin_y)
        
        # lock as we are accessing raw underlying map
        with self.mutex_:
            # fill map with data
            costmap_data = self.costmap_.getMap()
            costmap_size = self.costmap_.getSizeInCellsX() * self.costmap_.getSizeInCellsY()
            
            self.node_.get_logger().debug(f"full map update, {costmap_size} values")
            
            for i in range(min(costmap_size, len(msg.data))):
                cell_cost = int(msg.data[i]) & 0xFF  # Ensure unsigned char range
                if cell_cost < 0:
                    cell_cost = 255  # Handle -1 as 255
                costmap_data[i] = cost_translation_table__[cell_cost]
            
            self.node_.get_logger().debug(f"map updated, written {costmap_size} values")
    
    def update_partial_map(self, msg: OccupancyGridUpdate):
        """Update partial costmap"""
        self.node_.get_logger().debug("received partial map update")
        self.global_frame_ = msg.header.frame_id
        
        if msg.x < 0 or msg.y < 0:
            self.node_.get_logger().debug(
                f"negative coordinates, invalid update. x: {msg.x}, y: {msg.y}"
            )
            return
        
        x0 = msg.x
        y0 = msg.y
        xn = msg.width + x0
        yn = msg.height + y0
        
        # lock as we are accessing raw underlying map
        with self.mutex_:
            costmap_xn = self.costmap_.getSizeInCellsX()
            costmap_yn = self.costmap_.getSizeInCellsY()
            
            if xn > costmap_xn or x0 > costmap_xn or yn > costmap_yn or y0 > costmap_yn:
                self.node_.get_logger().warn(
                    f"received update doesn't fully fit into existing map, "
                    f"only part will be copied. received: [{x0}, {xn}], [{y0}, {yn}] "
                    f"map is: [0, {costmap_xn}], [0, {costmap_yn}]"
                )
            
            # update map with data
            costmap_data = self.costmap_.getMap()
            i = 0
            for y in range(y0, min(yn, costmap_yn)):
                for x in range(x0, min(xn, costmap_xn)):
                    idx = self.costmap_.getIndex(x, y)
                    cell_cost = int(msg.data[i]) & 0xFF  # Ensure unsigned char range
                    if cell_cost < 0:
                        cell_cost = 255  # Handle -1 as 255
                    costmap_data[idx] = cost_translation_table__[cell_cost]
                    i += 1
    
    def get_robot_pose(self) -> Pose:
        """Get the current robot pose"""
        robot_pose = PoseStamped()
        empty_pose = Pose()
        robot_pose.header.frame_id = self.robot_base_frame_
        robot_pose.header.stamp = self.node_.get_clock().now().to_msg()
        
        # get the global pose of the robot
        try:
            transform = self.tf_.lookup_transform(
                self.global_frame_,
                robot_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=3)
                # timeout=rclpy.duration.Duration(seconds=self.transform_tolerance_)
            )
            robot_pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(robot_pose, transform)
            return robot_pose_transformed.pose
            
        except LookupException as ex:
            self.node_.get_logger().error(
                f"No Transform available Error looking up robot pose: {str(ex)}",
                throttle_duration_sec=1.0
            )
            return empty_pose
        except ConnectivityException as ex:
            self.node_.get_logger().error(
                f"Connectivity Error looking up robot pose: {str(ex)}",
                throttle_duration_sec=1.0
            )
            return empty_pose
        except ExtrapolationException as ex:
            self.node_.get_logger().error(
                f"Extrapolation Error looking up robot pose: {str(ex)}",
                throttle_duration_sec=1.0
            )
            return empty_pose
        except TransformException as ex:
            self.node_.get_logger().error(
                f"Other error: {str(ex)}",
                throttle_duration_sec=1.0
            )
            return empty_pose