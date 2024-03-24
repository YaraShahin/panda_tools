import copy, math
import numpy as np
from geometry_msgs.msg import *
from tf.transformations import *

#Convert quaternion to euler
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians 

#Convert euler to quaternion
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]

def _approach(self, pose):
    """
    Move to a pose with a hover-distance above the requested pose and
    then move to the pose incrementally while monitoring the z force
    """
    print('approaching...')
    approach = copy.deepcopy(pose)
    approach.position.z = approach.position.z + self._hover_distance + self.robot.eps.z + self.robot.hand_EE_offset
    self.robot.move_to(approach)

    while approach.position.z >= pose.position.z:
        approach.position.z = approach.position.z - self.step_size
        self.robot.move_to(approach)

def get_pose(position, orientation=[1, 0, 0, 0]):
    grasp_pose = Pose()
    grasp_pose.position.x = position[0]
    grasp_pose.position.y = position[1]
    grasp_pose.position.z = position[2]
    grasp_pose.orientation.x = orientation[0]
    grasp_pose.orientation.y = orientation[1]
    grasp_pose.orientation.z = orientation[2]
    grasp_pose.orientation.w = orientation[3]

    return grasp_pose


def matrix_from_point_msg(point):
    """
    ## @brief Get a translation matrix from a geometry_msgs/Point
    ## @param point geometry_msgs/Point to turn into matrix
    """
    return translation_matrix((point.x, point.y, point.z))


def matrix_from_quaternion_msg(quaternion):
    """
    ## @brief Get a rotation matrix from a geometry_msgs/Quaternion
    ## @param quaternion geometry_msgs/Quaternion to turn into matrix
    """
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    return quaternion_matrix(q)


def matrix_from_pose_msg(pose):
    """
    ## @brief Get a transformation matrix from a geometry_msgs/Pose
    ## @param pose geometry_msgs/Pose to turn into matrix
    """
    t = matrix_from_point_msg(pose.position)
    r = matrix_from_quaternion_msg(pose.orientation)
    return concatenate_matrices(t, r)


def point_msg_from_matrix(transformation):
    """
    ## @brief Get a geometry_msgs/Point from a transformation matrix
    ## @param transformation The matrix to convert to a point
    """
    msg = Point()
    msg.x = transformation[0][3]
    msg.y = transformation[1][3]
    msg.z = transformation[2][3]
    return msg


def quaternion_msg_from_matrix(transformation):
    """
    ## @brief Get a geometry_msgs/Quaternion from a transformation matrix
    ## @param transformation The matrix to convert to a quaternion
    """
    q = quaternion_from_matrix(transformation)
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg


def pose_msg_from_matrix(transformation):
    """
    ## @brief Get a geometry_msgs/Pose from a transformation matrix
    ## @param transformation The matrix to convert to a pose
    """
    msg = Pose()
    msg.position = point_msg_from_matrix(transformation)
    msg.orientation = quaternion_msg_from_matrix(transformation)
    return msg


def translate_pose_msg(pose, x, y, z):
    """
    ## @brief Translate a geometry_msgs/Pose
    ## @param pose The pose to translate
    ## @param x The displacement in X coordinate axis
    ## @param y The displacement in Y coordinate axis
    ## @param z The displacement in Z coordinate axis
    """
    initial = matrix_from_pose_msg(pose)
    transform = translation_matrix((x,y,z))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))


def rotate_pose_msg_by_euler_angles(pose, r, p, y):
    """
    ## @brief Rotate a geometry_msgs/Pose
    ## @param pose The pose to rotate
    ## @param r The roll
    ## @param p The pitch
    ## @param y The yaw
    """
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))


def rotate_pose_msg_about_origin(pose, r, p, y):
    """
    ## @brief Rotate a geometry_msgs/Pose
    ## @param pose The pose to rotate
    ## @param r The roll
    ## @param p The pitch
    ## @param y The yaw
    """
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(transform, initial))
