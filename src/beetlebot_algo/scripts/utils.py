#!/usr/bin/env python3
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def lidar_scan(msgScan):
    """
    Convert LaserScan msg to array
    """
    distances = np.array([])
    angles = np.array([])
    information = np.array([])

    for i in range(len(msgScan.ranges)):
        # angle calculation
        ang = i * msgScan.angle_increment

        # distance calculation
        if ( msgScan.ranges[i] > msgScan.range_max ):
            dist = msgScan.range_max + 5
        elif ( msgScan.ranges[i] < msgScan.range_min ):
            dist = msgScan.range_min
        else:
            dist = msgScan.ranges[i]

        # smaller the distance, bigger the information (measurement is more confident)
        inf = ((msgScan.range_max - dist) / msgScan.range_max) ** 2 

        distances = np.append(distances, dist)
        angles = np.append(angles, ang)
        information = np.append(information, inf)

    # distances in [m], angles in [radians], information [0-1]
    return ( distances, angles, information )


def lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom):
	"""
	Lidar measurements in X-Y plane
	"""
	distances_x = np.array([])
	distances_y = np.array([])

	for (dist, ang) in zip(distances, angles):
		distances_x = np.append(distances_x, x_odom + dist * np.cos(ang + theta_odom))
		distances_y = np.append(distances_y, y_odom + dist * np.sin(ang + theta_odom))

	return (distances_x, distances_y)


def transform_orientation(orientation_q):
    """
    Transform theta, yaw angle to [radians] from [quaternion orientation]
    """
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

def get_odom_orientation(msgOdom):
    """"
    Get theta from Odometry msg in [radians]
    """
    orientation_q = msgOdom.pose.pose.orientation
    theta = transform_orientation(orientation_q)
    return theta
    

def get_odom_position(msgOdom):
    """
    Get (x,y) coordinates from Odometry msg in [m]
    """
    x = msgOdom.pose.pose.position.x
    y = msgOdom.pose.pose.position.y
    return (x, y)


##################

# def log_odds(p):
# 	"""
# 	Log odds ratio of p(x):

# 		           p(x)
# 	 l(x) = log ----------
# 		          1 - p(x)
# 	"""
# 	return np.log(p / (1 - p))


# def retrieve_p(l):
# 	"""
# 	Retrieve p(x) from log odds ratio:

# 	 		         1
# 	 p(x) = 1 - -----------------
# 		          1 + exp(l(x))
# 	"""
# 	return 1 - 1 / (1 + np.exp(l))

def prob_to_log_odds(x):
     return np.log(x) - np.log(1 - x)


def log_odds_to_prob(x):
    x_bigger = np.array([x], dtype=np.float128)
    return 1 - (1 / (1 + np.exp(x_bigger)))


def linear_mapping_of_values(x, old_min=0, old_max=1, new_min=0, new_max=100):
    """
    https://stackoverflow.com/a/929107/1253729
    """
    old_range = old_max - old_min
    new_range = new_max - new_min
    return ((((x - old_min) * new_range) / old_range) + new_min).astype(int)

def generate_transformation_matrix(translation, theta):
    """
    Generate a 3x3 homogeneous transformation matrix from translation and rotation.
    """
    t_x, t_y = translation

    transformation_matrix = np.array([
        [np.cos(theta), -np.sin(theta), t_x],
        [np.sin(theta), np.cos(theta),  t_y],
        [     0,             0,           1]])
    return transformation_matrix

def transform_point(point, transformation_matrix):
    # need an numpy array !!!
    transformed_point_homogeneous = np.dot(transformation_matrix, point)
    return transformed_point_homogeneous[:2]

def lookup_transform_dummy(frame_from, frame_to):
    """
    Dummy function to simulate the transformation lookup.
    """
    # For simplicity, we assume some fixed transformations here.
    # In reality, you should retrieve the actual transformations from your system.
    if frame_from == "camera" and frame_to == "odom":
        translation = [1.0, 2.0]  # Example translation
        rotation = np.deg2rad(45)  # Example rotation in radians
        return generate_transformation_matrix(translation, rotation)
    
    if frame_from == "map" and frame_to == "odom":
        translation = [3.0, 4.0]  # Example translation
        rotation = np.deg2rad(30)  # Example rotation in radians
        return generate_transformation_matrix(translation, rotation)
    
    return np.eye(3)

def angle_diff(a, b):
    d = a - b
    while d > np.pi:
        d -= 2 * np.pi
    while d < -np.pi:
        d += 2 * np.pi
    return d

# def angle_diff(a, b):
#     a = np.arctan2(np.sin(a), np.cos(a))
#     b = np.arctan2(np.sin(b), np.cos(b))

#     d1 = a - b
#     d2 = 2 * np.pi - np.abs(d1)

#     if d1 > 0.0:
#         d2 *= -1.0
    
#     if np.abs(d1) < np.abs(d2):
#         return d1
#     else:
#         return d2