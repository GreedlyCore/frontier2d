#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from tf2_ros import Buffer, TransformListener

# from GridMapUtils import  
from utils import linear_mapping_of_values, log_odds_to_prob, prob_to_log_odds, euler_from_quaternion

FLOOR_SIZE_X = 45
FLOOR_SIZE_Y = 45
RESOLUTION = 0.1
WORLD_ORIGIN_X = -FLOOR_SIZE_X / 2.0
WORLD_ORIGIN_Y = -FLOOR_SIZE_Y / 2.0
MAP_SIZE_X = int(FLOOR_SIZE_X / RESOLUTION)
MAP_SIZE_Y = int(FLOOR_SIZE_Y / RESOLUTION)
OCC_PROB = 0.9
FREE_PROB = 0.5

class LidarMapping(Node):
    def __init__(self):
        super().__init__('lidar_mapping_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ranges = []
        self.angle_min = 0
        self.range_max = 0
        self.angle_increment = 2 * np.pi / 30
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0.0

        self.z_random = 0.05
        self.laser_z_hit = 0.95
        self.laser_sigma_hit = 0.2

        self.l0 = np.round(np.log(OCC_PROB / FREE_PROB), 3)
        self.OCC_L = np.round(1.5 * self.l0, 3)
        self.FREE_L = np.round(0.5 * self.l0, 3)

        self.declare_parameter('alpha', 0.1)
        self.declare_parameter('beta', 0.05)
        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value

        self.create_subscription(LaserScan, '/lidar', self.get_scan, 5)
        self.create_subscription(Odometry, '/odom', self.get_ground_truth_pose, 5)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 5)

        # self.from_frame = 'base_link'
        # self.to_frame = 'map'
        self.occupancy_grid_msg = OccupancyGrid()
        self.occupancy_grid_msg.header.frame_id = 'beetlebot/odom'
        self.occupancy_grid_msg.info = MapMetaData(
            width=MAP_SIZE_X,
            height=MAP_SIZE_Y,
            resolution=RESOLUTION,
            origin=Pose(position=Point(x=WORLD_ORIGIN_X, y=WORLD_ORIGIN_Y),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        )
        self.map = self.generate_map(0.0)
        self.angles = np.zeros(360)
        self.pub_map()
        self.get_logger().info('--- Lidar mapping initted ---')

    def generate_map(self, value):
        return np.full(shape=(MAP_SIZE_X, MAP_SIZE_Y), fill_value=value)

    def get_ground_truth_pose(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        ori_q = msg.pose.pose.orientation
        _, _, self.robot_theta = euler_from_quaternion(msg.pose.pose.orientation)
        

    def get_scan(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_max = msg.range_max
        self.angles = np.linspace(self.angle_min, self.angle_max, len(msg.ranges))

        for i in range(len(self.ranges)):
            r = self.ranges[i]
            if not np.isfinite(r) or r > self.range_max:
                continue  

            robot_pose = [self.robot_x, self.robot_y, self.robot_theta]
            # print(f"robot_pose: { robot_pose }")
            proj = [r * np.cos(robot_pose[2] + self.angles[i]),
                    r * np.sin(robot_pose[2] + self.angles[i])]
            laser_point_world = [robot_pose[0] + proj[0], robot_pose[1] + proj[1]]

            try:
                perceptual_range = self.get_perceptual_range(robot_pose, laser_point_world)
            except OverflowError:
                continue  # Подстраховка 

            for j, (ix, iy) in enumerate(perceptual_range):
                p = self.inverse_range_sensor_model(j, len(perceptual_range))
                self.mark_map(ix, iy, value=prob_to_log_odds(p))

        self.pub_map()


    def coords_to_2d_array(self, x, y):
        ix = int((x - WORLD_ORIGIN_X) / RESOLUTION)
        iy = int((y - WORLD_ORIGIN_Y) / RESOLUTION)
        return ix, iy

    def pub_map(self):
        map_prob = log_odds_to_prob(self.map)
        map_visual = linear_mapping_of_values(map_prob)
        self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_grid_msg.data = map_visual.astype(int).T.reshape(map_visual.size, order='C').tolist()
        self.map_publisher.publish(self.occupancy_grid_msg)

    def mark_map(self, ix, iy, value):
        try:
            self.map[ix, iy] += value
        except Exception as e:
            self.get_logger().error(f'Map marking error: {e}')

    def get_map_val(self, ix, iy):
        try:
            return self.map[ix, iy]
        except Exception as e:
            self.get_logger().error(f'Map value error: {e}')

    def get_perceptual_range(self, robot_pose, obs):
        x0, y0 = self.coords_to_2d_array(robot_pose[0], robot_pose[1])
        x1, y1 = self.coords_to_2d_array(obs[0], obs[1])
        dx = np.abs(x1 - x0)
        dy = -np.abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        perceptual_range = []
        while True:
            perceptual_range.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return perceptual_range


    def inverse_range_sensor_model(self, i, len_perceptual_range):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        """
        if i == (len_perceptual_range - 1): return OCC_PROB
        else: return FREE_PROB
    


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
