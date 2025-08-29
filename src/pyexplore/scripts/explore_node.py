#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import SaveMap

from frontier_search import FrontierSearch
from costmap_client import Costmap2DClient
import threading
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from datetime import datetime
import math
from enum import Enum


class ExplorationState(Enum):
    INITIALIZING = "initializing"
    INITIAL_MOVE = "initial_move" 
    EXPLORING = "exploring"
    RECOVERY = "recovery"
    FINISHED = "finished"


class RecoveryBehavior(Enum):
    ROTATE_IN_PLACE = "rotate_in_place"
    MOVE_BACK = "move_back"
    RANDOM_WALK = "random_walk"
    RETURN_TO_START = "return_to_start"

"""
 ## Main planning function
 ### Send new goal if:
 1. Global Timeout has finished
 2. Local timeout has finished - no progress (meters) (recovery behaviours ???)
 3. Previous goal has been finished/aborted
 4. If there is <5% of distance to current goal (configurable percent)
 ### Requirements for the new goal
 1. We're not pursuing the same goal 
 2. Min distance between new and old goals (distance check with some thresh)
 """
class Explore(Node):
    def __init__(self):
        super().__init__('explore_node')
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.costmap_client = Costmap2DClient(self, self.tf_buffer)
        
        # State management
        self.exploration_state = ExplorationState.INITIALIZING
        self.recovery_behavior = RecoveryBehavior.ROTATE_IN_PLACE
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        
        # Goal tracking
        self.prev_goal = Point()
        self.current_goal = Point()
        self.prev_distance = 0.0
        self.last_progress = self.get_clock().now()
        self.frontier_blacklist = []
        self.start_position = None
        
        # Visualization
        self.last_markers_count = 0
        
        # Declare parameters
        self.declare_parameter('planner_frequency', 0.5)
        self.declare_parameter('progress_timeout', 10.0)  # Local timeout for no movement
        self.declare_parameter('visualize', True)
        self.declare_parameter('potential_scale', 2.0)
        self.declare_parameter('gain_scale', 1e-3)
        self.declare_parameter('min_frontier_size', 0.5)
        self.declare_parameter('goal_interval', 40.0)  # Global timeout per goal
        self.declare_parameter('initial_help', True)
        self.declare_parameter('initial_move_duration', 10.0)
        self.declare_parameter('min_goal_distance', 2.0)
        self.declare_parameter('goal_reached_threshold', 0.5)
        self.declare_parameter('no_frontiers_timeout', 60.0)
        self.declare_parameter('recovery_rotation_speed', 0.5)
        self.declare_parameter('recovery_duration', 15.0)
        self.declare_parameter('close_goal_threshold_percent', 0.05)  # 5% of original distance
        
        # Get parameters
        self.planner_frequency = self.get_parameter('planner_frequency').get_parameter_value().double_value
        self.timeout = self.get_parameter('progress_timeout').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.initial_help = self.get_parameter('initial_help').get_parameter_value().bool_value
        self.potential_scale = self.get_parameter('potential_scale').get_parameter_value().double_value
        self.gain_scale = self.get_parameter('gain_scale').get_parameter_value().double_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().double_value
        self.goal_interval = self.get_parameter('goal_interval').get_parameter_value().double_value
        self.initial_move_duration = self.get_parameter('initial_move_duration').get_parameter_value().double_value
        self.min_goal_distance = self.get_parameter('min_goal_distance').get_parameter_value().double_value
        self.goal_reached_threshold = self.get_parameter('goal_reached_threshold').get_parameter_value().double_value
        self.no_frontiers_timeout = self.get_parameter('no_frontiers_timeout').get_parameter_value().double_value
        self.recovery_rotation_speed = self.get_parameter('recovery_rotation_speed').get_parameter_value().double_value
        self.recovery_duration = self.get_parameter('recovery_duration').get_parameter_value().double_value
        self.close_goal_threshold = self.get_parameter('close_goal_threshold_percent').get_parameter_value().double_value
        
        # Timing
        self.last_goal_sent_time = self.get_clock().now()
        self.initial_move_start_time = None
        self.no_frontiers_start_time = None
        self.recovery_start_time = None
        self.original_goal_distance = 0.0  # Store original distance to goal
        
        # Action clients and publishers
        self.nav2_base_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Service client for map saving
        self.pkg_share_dir = get_package_share_directory('pyexplore')
        self.client = self.create_client(SaveMap, '/map_saver/save_map')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map server service is not available, waiting again...')
            
        # Laser scan for initial helper
        self.laser_scan = None
        self.laser_sub = self.create_subscription(LaserScan, 'lidar', self.laser_callback, 10)
        
        self.search = FrontierSearch(self.costmap_client.get_costmap(),
                                    self.potential_scale, self.gain_scale,
                                    self.min_frontier_size)
        
        # Initialize visualization publisher if needed
        if self.visualize:
            self.marker_array_publisher = self.create_publisher(MarkerArray, 'frontiers', 10)
        
        self.goal_handle = None
        self.mutex_ = threading.Lock()
        self.goal_tolerance = 0.5
        
        # Map saving setup
        self.save_image_timer = self.create_timer(20.0, self.save_map)
        self.latest_frontier_cells = None
        self.image_counter = 0
        self.image_mutex = threading.Lock()
        
        # Wait for nav2
        self.get_logger().info("--- Waiting to connect to nav2 server ---")
        self.nav2_base_client.wait_for_server()
        self.get_logger().info("--- Connected to nav2 server ---")
        # Create main behavior tree timer
        timer_period = 1.0 / self.planner_frequency
        self.behavior_tree_timer = self.create_timer(timer_period, self.behavior_tree_tick)
        
        self.get_logger().info(f"PyExplore node initialized in {self.exploration_state.value} state")

    def laser_callback(self, msg):
        self.laser_scan = msg

    def behavior_tree_tick(self):
        try:
            if self.exploration_state == ExplorationState.INITIALIZING:
                self.handle_initializing_state()
            elif self.exploration_state == ExplorationState.INITIAL_MOVE:
                self.handle_initial_move_state()
            elif self.exploration_state == ExplorationState.EXPLORING:
                self.handle_exploring_state()
            elif self.exploration_state == ExplorationState.RECOVERY:
                self.get_logger().info(f"RECOVERY behaviour not enabled yet")
                #TODO: will enable and test later
                # self.handle_recovery_state()
            elif self.exploration_state == ExplorationState.FINISHED:
                self.handle_finished_state()
        except Exception as e:
            self.get_logger().error(f"Error in behavior tree: {str(e)}")
            #TODO: will enable and test later // maybe configure nav2 BT and GROOT
            # self.transition_to_recovery()

    def handle_initializing_state(self):
        """Initialize exploration - store start position and decide on initial move"""
        pose = self.costmap_client.get_robot_pose()
        if pose is None:
            self.get_logger().warn("Cannot get robot pose, waiting...")
            return
            
        self.start_position = pose.position
        # self.get_logger().info(f"Starting exploration from: ({pose.position.x:.2f}, {pose.position.y:.2f})")
        
        if self.initial_help:
            self.transition_to_initial_move()
        else:
            self.transition_to_exploring()

    def handle_initial_move_state(self):
        """Execute initial exploratory move to gather more map data"""
        current_time = self.get_clock().now()
        
        # Start initial move if not started
        if self.initial_move_start_time is None:
            self.initial_move_start_time = current_time
            initial_goal = self.calculate_initial_goal()
            if initial_goal:
                self.send_navigation_goal(initial_goal)
                self.get_logger().info("Sent initial exploratory goal")
            else:
                self.get_logger().warn("Could not calculate initial goal, moving to exploration")
                self.transition_to_exploring()
                return
        
        # Check if initial move duration exceeded or goal completed
        duration = (current_time - self.initial_move_start_time).nanoseconds / 1e9
        goal_finished = self.is_goal_finished()
        
        if duration >= self.initial_move_duration or goal_finished:
            if goal_finished:
                self.get_logger().info("Initial move completed")
            else:
                self.get_logger().info("Initial move duration exceeded, canceling goal")
                self.cancel_current_goal()
            self.transition_to_exploring()

    def handle_exploring_state(self):
        """Main exploration logic"""
        frontiers = None
        with self.mutex_:
            pose = self.costmap_client.get_robot_pose()
            if pose is None:
                return
            frontiers, frontier_cells = self.search.search_from(pose.position)
        
        # Store frontier cells for visualization
        with self.image_mutex:
            self.latest_frontier_cells = frontier_cells
        
        # Visualize frontiers
        if self.visualize:
            self.visualize_frontiers(frontiers)
        
        # Check for no frontiers condition
        if not frontiers or len(frontiers) == 0:
            self.handle_no_frontiers()
            return
        
        # Reset no frontiers timer since we have frontiers
        self.no_frontiers_start_time = None
        
        # Find best frontier (not blacklisted)
        target_frontier = self.select_best_frontier(frontiers)
        if target_frontier is None:
            self.get_logger().warn("All frontiers blacklisted")
            self.transition_to_recovery()
            return
        
        # Decide if we should send a new goal
        if self.should_send_new_goal(target_frontier, pose.position):
            self.send_navigation_goal(target_frontier.centroid)
            self.update_goal_tracking(target_frontier)

    def handle_recovery_state(self):
        """Handle recovery behaviors when exploration fails"""
        current_time = self.get_clock().now()
        
        if self.recovery_start_time is None:
            self.recovery_start_time = current_time
            self.get_logger().info(f"Starting recovery behavior: {self.recovery_behavior.value}")
        
        duration = (current_time - self.recovery_start_time).nanoseconds / 1e9
        
        if duration >= self.recovery_duration:
            self.finish_recovery_behavior()
            return
        
        self.execute_recovery_behavior()

    def handle_finished_state(self):
        """Handle finished exploration state"""
        self.get_logger().info_throttle(10.0, "Exploration finished")
        # Could implement final behaviors here like returning to start

    def transition_to_initial_move(self):
        """Transition to initial move state"""
        self.exploration_state = ExplorationState.INITIAL_MOVE
        self.initial_move_start_time = None
        self.get_logger().info("Transitioning to initial move")

    def transition_to_exploring(self):
        """Transition to exploring state"""
        self.exploration_state = ExplorationState.EXPLORING
        self.get_logger().info("Transitioning to exploring")

    def transition_to_recovery(self):
        """Transition to recovery state"""
        self.exploration_state = ExplorationState.RECOVERY
        self.recovery_start_time = None
        self.recovery_attempts += 1
        self.get_logger().warn(f"Transitioning to recovery (attempt {self.recovery_attempts})")

    def transition_to_finished(self):
        """Transition to finished state"""
        self.exploration_state = ExplorationState.FINISHED
        self.cancel_current_goal()
        self.get_logger().info("Exploration finished")

    def calculate_initial_goal(self):
        """Calculate initial goal based on laser scan data"""
        if self.laser_scan is None:
            self.get_logger().warn("No laser scan data available")
            return None
        
        pose = self.costmap_client.get_robot_pose()
        if pose is None:
            return None
        
        # Find the direction with maximum free space
        ranges = np.array(self.laser_scan.ranges)
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0)]
        
        if len(valid_ranges) == 0:
            return None
        
        # Find angle with maximum range
        max_range_idx = np.argmax(ranges)
        if not np.isfinite(ranges[max_range_idx]):
            max_range_idx = np.argmax(valid_ranges)
        
        angle = self.laser_scan.angle_min + max_range_idx * self.laser_scan.angle_increment
        
        # Calculate goal position (move 70% of max range in that direction)
        max_range = min(ranges[max_range_idx], 5.0)  # Cap at 5 meters
        target_distance = max_range * 0.7
        
        goal = Point()
        goal.x = pose.position.x + target_distance * math.cos(angle)
        goal.y = pose.position.y + target_distance * math.sin(angle)
        goal.z = 0.0
        
        self.get_logger().info(f"Initial goal: ({goal.x:.2f}, {goal.y:.2f}) at angle {math.degrees(angle):.1f}°")
        return goal

    def handle_no_frontiers(self):
        """Handle case when no frontiers are detected"""
        current_time = self.get_clock().now()
        
        if self.no_frontiers_start_time is None:
            self.no_frontiers_start_time = current_time
            self.get_logger().warn("No frontiers detected, starting timeout")
        
        duration = (current_time - self.no_frontiers_start_time).nanoseconds / 1e9
        
        if duration >= self.no_frontiers_timeout:
            self.get_logger().info("No frontiers timeout exceeded, exploration finished")
            self.transition_to_finished()

    def select_best_frontier(self, frontiers):
        """Select the best frontier that's not blacklisted"""
        for frontier in frontiers:
            if not self.goal_on_blacklist(frontier.centroid):
                return frontier
        return None

    def should_send_new_goal(self, target_frontier, current_position):
        """
        Determine if we should send a new goal based on:
        1. Goal completion status (SUCCESS/ABORTED/CANCELED)
        2. Global goal timeout (e.g., 40 seconds per goal)
        3. Local timeout - no movement progress (e.g., 10 seconds without movement)
        4. Close to goal threshold (e.g., <5% of original distance remaining)
        """
        # If no active goal, we can send one
        if self.goal_handle is None:
            self.get_logger().info("TRUE: No active goal")
            return True
            
        current_time = self.get_clock().now()
        
        # 1. Check goal completion status first (highest priority)
        goal_is_finished = self.is_goal_finished()
        if goal_is_finished:
            status = self.goal_handle._status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("TRUE: Previous goal succeeded")
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info("TRUE: Previous goal aborted")
                self.frontier_blacklist.append(self.current_goal)
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("TRUE: Previous goal canceled")
            return True
        
        # Goal is still executing - check other timeout conditions
        time_since_last_goal = (current_time - self.last_goal_sent_time).nanoseconds / 1e9
        
        # 2. Check global timeout (robot should finish goal within this time)
        if time_since_last_goal >= self.goal_interval:
            self.get_logger().info(f"TRUE: Global timeout exceeded ({time_since_last_goal:.1f}s >= {self.goal_interval}s)")
            self.cancel_current_goal()
            return True
        
        # 3. Check local timeout - no movement progress  
        current_distance_to_goal = math.sqrt(
            (current_position.x - self.current_goal.x)**2 + 
            (current_position.y - self.current_goal.y)**2
        )
        
        # Update progress tracking only if we moved closer
        if current_distance_to_goal < self.prev_distance - 0.1:  # Small threshold to avoid noise
            self.last_progress = current_time
            self.prev_distance = current_distance_to_goal
            self.get_logger().debug(f"Made progress: {current_distance_to_goal:.2f}m to goal")
        
        # time_since_progress = (current_time - self.last_progress).nanoseconds / 1e9
        # if time_since_progress >= self.timeout:
        #     self.get_logger().info(f"TRUE: Local timeout - no progress for {time_since_progress:.1f}s")
        #     self.frontier_blacklist.append(self.current_goal)
        #     self.cancel_current_goal()
        #     return True
        
        # 4. Check if we're very close to goal (less than X% of original distance)
        if self.original_goal_distance > 0:
            remaining_percentage = current_distance_to_goal / self.original_goal_distance
            if remaining_percentage < self.close_goal_threshold:
                self.get_logger().info(f"TRUE: Close to goal ({remaining_percentage*100:.1f}% < {self.close_goal_threshold*100}%)")
                self.cancel_current_goal()  # Cancel since we're close enough
                return True
        
        # Don't send new goal - current goal is still valid and executing
        # self.get_logger().debug(f"FALSE: Continuing current goal (no progress: {time_since_progress:.1f}s, "
        #                        f"global time: {time_since_last_goal:.1f}/{self.goal_interval}s, "
        #                        f"distance: {current_distance_to_goal:.2f}m)")
        return False


    def update_goal_tracking(self, frontier):
        """Update goal tracking variables"""
        self.current_goal = frontier.centroid
        self.prev_goal = frontier.centroid
        
        # Calculate and store original distance for progress tracking
        pose = self.costmap_client.get_robot_pose()
        if pose:
            self.original_goal_distance = math.sqrt(
                (pose.position.x - frontier.centroid.x)**2 + 
                (pose.position.y - frontier.centroid.y)**2
            )
            self.prev_distance = self.original_goal_distance
        else:
            self.original_goal_distance = frontier.min_distance
            self.prev_distance = frontier.min_distance
            
        self.last_goal_sent_time = self.get_clock().now()
        self.last_progress = self.get_clock().now()  # Reset progress timer

    def execute_recovery_behavior(self):
        """Execute the current recovery behavior"""
        if self.recovery_behavior == RecoveryBehavior.ROTATE_IN_PLACE:
            twist = Twist()
            twist.angular.z = self.recovery_rotation_speed
            self.cmd_vel_pub.publish(twist)
        elif self.recovery_behavior == RecoveryBehavior.MOVE_BACK:
            # Move backwards slowly
            twist = Twist()
            twist.linear.x = -0.2
            self.cmd_vel_pub.publish(twist)
        elif self.recovery_behavior == RecoveryBehavior.RANDOM_WALK:
            # Implement random walk behavior
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = (np.random.random() - 0.5) * 1.0
            self.cmd_vel_pub.publish(twist)

    def finish_recovery_behavior(self):
        """Finish current recovery behavior and transition"""
        # Stop robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Clear some blacklisted goals if too many attempts
        if self.recovery_attempts >= self.max_recovery_attempts:
            self.get_logger().info("Max recovery attempts reached, clearing blacklist")
            self.frontier_blacklist.clear()
            self.recovery_attempts = 0
            
        # Try next recovery behavior
        self.cycle_recovery_behavior()
        self.transition_to_exploring()

    def cycle_recovery_behavior(self):
        """Cycle to next recovery behavior"""
        behaviors = list(RecoveryBehavior)
        current_idx = behaviors.index(self.recovery_behavior)
        self.recovery_behavior = behaviors[(current_idx + 1) % len(behaviors)]

    def send_navigation_goal(self, target_position):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position = target_position
        goal.pose.pose.orientation.w = 1.0
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        send_goal_future = self.nav2_base_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, target_position))
        self.get_logger().info(f"Sent goal: ({target_position.x:.2f}, {target_position.y:.2f})")

    def cancel_current_goal(self):
        """Cancel the current navigation goal"""
        if self.goal_handle and not self.is_goal_finished():
            self.goal_handle.cancel_goal_async()

    def is_goal_finished(self):
        """Check if the current goal is finished"""
        if self.goal_handle is None:
            return True
        
        status = self.goal_handle._status
        # https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html
        finished_states = [
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED, 
            GoalStatus.STATUS_CANCELED
        ]
        self.get_logger().info(f"\nCurrent goal status: {status}\n")
        return status in finished_states

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        pass  # Could track navigation progress here

    def goal_response_callback(self, future, target_position):
        """Handle goal response"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.reached_goal(future.result(), target_position))

    def reached_goal(self, result, frontier_goal):
        """Handle goal completion"""
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug("Goal succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().debug("Goal aborted, blacklisting")
            self.frontier_blacklist.append(frontier_goal)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().debug("Goal canceled")

    # ... (include all the other methods from your original code like visualize_frontiers, 
    # goal_on_blacklist, save_frontier_image, etc.)
    
    def visualize_frontiers(self, frontiers):
        """Visualize frontiers as markers"""
        if not self.visualize or not frontiers:
            return
            
        red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        
        current_time = self.get_clock().now().to_msg()
        markers_msg = MarkerArray()
        markers = markers_msg.markers
        
        min_cost = frontiers[0].cost if frontiers else 0.0
        id = 0
            
        for frontier in frontiers:
            # POINTS marker
            m_points = Marker()
            m_points.header.frame_id = 'map'
            m_points.header.stamp = current_time
            m_points.ns = "frontiers"
            m_points.id = id
            m_points.type = Marker.POINTS
            m_points.action = Marker.ADD
            m_points.pose.orientation.w = 1.0
            m_points.scale.x = 0.1
            m_points.scale.y = 0.1
            m_points.scale.z = 0.1
            m_points.frame_locked = True
            m_points.points = frontier.points
            m_points.color = red if self.goal_on_blacklist(frontier.centroid) else blue
            m_points.lifetime.sec = 0
            markers.append(m_points)
            id += 1
            
            # SPHERE marker  
            m_sphere = Marker()
            m_sphere.header.frame_id = 'map'
            m_sphere.header.stamp = current_time
            m_sphere.ns = "frontiers"
            m_sphere.id = id
            m_sphere.type = Marker.SPHERE
            m_sphere.action = Marker.ADD
            m_sphere.pose.position = frontier.initial
            m_sphere.pose.orientation.w = 1.0
            m_sphere.scale.x = m_sphere.scale.y = m_sphere.scale.z = min(abs(min_cost * 0.4 / frontier.cost), 0.5)
            m_sphere.color = green
            m_sphere.frame_locked = True
            m_sphere.lifetime.sec = 0
            markers.append(m_sphere)
            id += 1
        
        # Delete unused markers
        if id < self.last_markers_count:
            for i in range(id, self.last_markers_count):
                delete_marker = Marker()
                delete_marker.id = i
                delete_marker.action = Marker.DELETE
                markers.append(delete_marker)
        
        self.last_markers_count = id
        self.marker_array_publisher.publish(markers_msg)

    def goal_on_blacklist(self, goal):
        """Check if goal is on blacklist"""
        tolerance = 5
        resolution = 0.05  # typical costmap resolution
        
        for frontier_goal in self.frontier_blacklist:
            x_diff = abs(goal.x - frontier_goal.x)
            y_diff = abs(goal.y - frontier_goal.y)
            
            if (x_diff < tolerance * resolution and
                y_diff < tolerance * resolution):
                return True
        return False

    # ros2 run nav2_map_server map_saver_cli -f "map_name" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/namespace
    def save_map(self):
        """
        Save map for debug purposes
        using actions
        https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html
        """
        request = SaveMap.Request()
        # Format: MMDD_HHMMSS (e.g., 1229_143055 for Dec 29, 14:30:55)
        map_name = "map_" + datetime.now().strftime("%m%d_%H%M%S")
        request.map_topic = "/map" #"/global_costmap/costmap"
        # maps_dir = os.path.join(self.pkg_share_dir, 'maps')
        # swap from abs to relative
        maps_dir = '/home/sonieth2/ros/ros2/projects/practice_3d_year/frontier2d/maps'
        map_path = os.path.join(maps_dir, map_name)
        request.map_url = map_path + ".yaml"
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        future = self.client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            self.get_logger().info(f'Map saved successfully: {map_name}')
            return True
        else:
            self.get_logger().error('Service call failed')
            return False

    def start(self):
        """Start exploration"""
        if self.exploration_state == ExplorationState.FINISHED:
            self.exploration_state = ExplorationState.INITIALIZING
        self.get_logger().info("Exploration started")

    def stop(self):
        """Stop exploration"""
        self.cancel_current_goal()
        self.transition_to_finished()
        self.get_logger().info("Exploration stopped")


def main(args=None):
    rclpy.init(args=args)
    explore_node = Explore()
    
    try:
        rclpy.spin(explore_node) 
    except KeyboardInterrupt:
        explore_node.get_logger().info("Shutting down exploration node")
    finally:
        explore_node.stop()
        explore_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()