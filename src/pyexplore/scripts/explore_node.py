#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import tf2_ros
import threading

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from frontier_search import FrontierSearch
from costmap_client import Costmap2DClient

from action_msgs.msg import GoalStatus

class Explore(Node):
    def __init__(self):
        super().__init__('explore_node')
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.costmap_client = Costmap2DClient(self, self.tf_buffer)
        
        self.prev_distance = 0.0
        self.last_markers_count = 0
        
        # Declare parameters
        self.declare_parameter('planner_frequency', 1.0)
        self.declare_parameter('progress_timeout', 30.0)
        self.declare_parameter('visualize', True)
        self.declare_parameter('potential_scale', 1e-3)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('min_frontier_size', 0.5)
        
        self.declare_parameter('goal_interval', 20.0)
        
        # Get parameters
        self.planner_frequency = self.get_parameter('planner_frequency').get_parameter_value().double_value
        timeout = self.get_parameter('progress_timeout').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.potential_scale = self.get_parameter('potential_scale').get_parameter_value().double_value
        self.gain_scale = self.get_parameter('gain_scale').get_parameter_value().double_value
        min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().double_value
        
        self.goal_interval = self.get_parameter('goal_interval').get_parameter_value().double_value
        self.last_goal_sent_time = self.get_clock().now() #- Duration(seconds=self.goal_interval)
        
        self.progress_timeout = timeout
        
        self.nav2_base_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.search = FrontierSearch(self.costmap_client.get_costmap(),
                                    self.potential_scale, self.gain_scale,
                                    min_frontier_size)
        
        # Initialize visualization publisher if needed
        if self.visualize:
            self.marker_array_publisher = self.create_publisher(
                MarkerArray, 'frontiers', 10)
        
        # Initialize member variables
        self.prev_goal = Point()
        self.last_progress = self.get_clock().now()
        self.frontier_blacklist = []
        
        self.get_logger().info("Waiting to connect to nav2 server")
        self.nav2_base_client.wait_for_server()
        self.get_logger().info("Connected to nav2 server")
        
        # Create timer for exploration
        timer_period = 1.0 / self.planner_frequency  # in seconds
        self.exploring_timer = self.create_timer(timer_period, self.make_plan)
        
        self.goal_handle = None
        self.mutex_ = threading.Lock()
        
    
    def __del__(self):
        self.stop()
    
    def visualize_frontiers(self, frontiers):
        """
        Visualize frontiers as markers
        """
        red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        
        self.get_logger().debug(f"visualising {len(frontiers)} frontiers")
        
        current_time = self.get_clock().now().to_msg()

        markers_msg = MarkerArray()
        markers = markers_msg.markers
        
        # weighted frontiers are always sorted
        min_cost = 0.0 if not frontiers else frontiers[0].cost
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
        
        current_markers_count = len(markers)
        
        # Delete previous markers, which are now unused
        current_markers_count = len(markers)
        if current_markers_count < self.last_markers_count:
            for i in range(id, self.last_markers_count+1):
                delete_marker = Marker()
                delete_marker.id = i
                delete_marker.action = Marker.DELETE
                markers.append(delete_marker)
        
        self.last_markers_count = current_markers_count
        self.marker_array_publisher.publish(markers_msg)
    
    def make_plan(self):
        """
        Main planning function
        """
        with self.mutex_:
            # find frontiers
            pose = self.costmap_client.get_robot_pose()
            # get frontiers sorted according to cost
            frontiers = self.search.search_from(pose.position)
            
            self.get_logger().info(f'hm: {self.search.num_features} ')
        self.get_logger().debug(f"found {len(frontiers)} frontiers")
        for i, frontier in enumerate(frontiers):
            self.get_logger().debug(f"frontier {i} cost: {frontier.cost}")
        
        # TODO: temporary comment
        # if not frontiers:
        #     self.stop()
        #     return
        
        # publish frontiers as visualization markers
        if self.visualize:
            self.visualize_frontiers(frontiers)
        
        # find non blacklisted and most-cost frontier
        frontier = None
        for f in frontiers:
            if not self.goal_on_blacklist(f.centroid):
                frontier = f
                break
        
        if frontier is None:
            # self.stop()
            self.get_logger().info(f"No frontiers found (???), stop.")
            return
        
        target_position = frontier.centroid
        # time out if we are not making any progress
        same_goal = self.prev_goal == target_position

        
        self.prev_goal = target_position
        if not same_goal or (self.prev_distance > frontier.min_distance):
            # we have different goal or we made some progress
            self.last_progress = self.get_clock().now()
            self.prev_distance = frontier.min_distance
        
        # black list if we've made no progress for a long time
        if (self.get_clock().now() - self.last_progress).nanoseconds / 1e9 > self.progress_timeout:  
            self.frontier_blacklist.append(target_position)
            self.get_logger().debug("Adding current goal to black list, giving next plan")
            self.make_plan()
            return
        
        # Check if we should send a new goal
        time_since_last_goal = (self.get_clock().now() - self.last_goal_sent_time).nanoseconds / 1e9
        goal_is_finished = self.is_goal_finished()
        
        # Only send new goal if:
        # 1. We have a different goal AND (enough time has passed OR current goal is finished)
        # 2. OR if we're not pursuing the same goal
        should_send_goal = False
        
        if not same_goal:
            # Different goal - check timing or completion status
            if goal_is_finished or time_since_last_goal >= self.goal_interval:
                should_send_goal = True
            else:
                self.get_logger().debug(f"Waiting: {self.goal_interval - time_since_last_goal:.1f}s left before next goal (current goal still active)")
                return
        else:
            # Same goal - don't send unless current goal is finished and we want to retry
            if goal_is_finished and time_since_last_goal >= self.goal_interval:
                should_send_goal = True
            elif not goal_is_finished:
                # Same goal and still executing - do nothing
                return
            else:
                self.get_logger().debug(f"Same goal finished but waiting: {self.goal_interval - time_since_last_goal:.1f}s left")
                return
        
        if should_send_goal:
            self.get_logger().debug("Sending goal to move base nav2")
            self.last_goal_sent_time = self.get_clock().now()    
            
            # send goal to move_base if we have something new to pursue
            goal = NavigateToPose.Goal()
            goal.pose.pose.position = target_position
            goal.pose.pose.orientation.w = 1.0
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            
            # Send goal with result callback
            send_goal_future = self.nav2_base_client.send_goal_async(goal, 
                                                                    feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(
                lambda future: self.goal_response_callback(future, target_position))

    def is_goal_finished(self):
        """
        Check if the current goal is finished (succeeded, aborted, or canceled)
        """
        if self.goal_handle is None:
            return True  # No active goal || or unkwown error
        status = self.goal_handle._status            
        # Goal is finished if it's in any terminal state
        finished_states = [
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED, 
            GoalStatus.STATUS_CANCELED
            # TODO: more status like EXECUTING?
        ]
        return status in finished_states
            
        
    
    # nav2_msgs.action.NavigateToPose_Feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
           
    
    def goal_response_callback(self, future, target_position):
        """
        Handle goal response and set up result callback
        """
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        # self.get_logger().info(f'{goal_handle.get_status()}')
        
        # Get result
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.reached_goal(future.result(), target_position))
    
    
    def goal_on_blacklist(self, goal):
        """
        Check if goal is on blacklist
        """
        tolerance = 5
        # costmap2d = self.costmap_client.get_costmap()
        
        # check if a goal is on the blacklist for goals that we're pursuing
        for frontier_goal in self.frontier_blacklist:
            x_diff = abs(goal.x - frontier_goal.x)
            y_diff = abs(goal.y - frontier_goal.y)
            
            # if (x_diff < tolerance * costmap2d.get_resolution() and
            #     y_diff < tolerance * costmap2d.get_resolution()):
            #     return True
            
            # Placeholder resolution value - would need actual costmap
            resolution = 0.05  # typical costmap resolution
            if (x_diff < tolerance * resolution and
                y_diff < tolerance * resolution):
                return True
        
        return False
    
    def reached_goal(self, result, frontier_goal):
        """
        Handle goal completion result
        """
        status = result.status
        
        if status == GoalStatus.STATUS_EXECUTING:
            self.get_logger().debug("Goal EXECUTING")
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug("Goal was successful")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().debug("Goal was aborted")
            self.frontier_blacklist.append(frontier_goal)
            self.get_logger().debug("Adding current goal to black list")
            return
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().debug("Goal was canceled")
            return
        else:
            self.get_logger().warn(f"Unknown result code from nav2: {status}")
            return
                
        # find new goal immediately regardless of planning frequency.
        # execute via timer to prevent dead lock in nav2_base_client (this is
        # callback for sendGoal, which is called in makePlan). the timer must live
        # until callback is executed.
        # oneshot_ = relative_nh_.createTimer(
        #     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
        #     true);
        
        # TODO: Implement this with ros2 timers?
        # Because of the async nature of ros2 calls I think this is not needed.
        # self.make_plan()
    
    def start(self):
        """
        Start exploration
        """
        self.get_logger().info("Exploration started.")
    
    def stop(self):
        """
        Stop exploration
        """
        # TODO - check the API
        # self.nav2_base_client.cancel_all_goals()
        # self.nav2_base_client.cancel_goal_async()
        # if hasattr(self, 'exploring_timer'):
        #     self.exploring_timer.cancel()
        
        self.get_logger().info("Exploration stopped.")


def main(args=None):
    rclpy.init(args=args)
    
    # ROS1 code
    # if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    #                                    ros::console::levels::Debug)) {
    #   ros::console::notifyLoggerLevelsChanged();
    # }
    
    explore_node = Explore()
    rclpy.spin(explore_node)  # std::move(std::make_unique)?
    
    explore_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()