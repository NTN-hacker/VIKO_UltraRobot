import tf
from geometry_msgs.msg import Pose
import numpy as np

# Assume target_point and target_normal are given
target_point = [0.5, 0.2, 0.3]  # Example target point
target_normal = [0.0, 0.0, 1.0]  # Example normal vector (pointing upwards)
# Define the z-axis of the end-effector frame (normal vector)
z_axis = target_normal

# Define an arbitrary vector perpendicular to z_axis as the x-axis
x_axis = np.cross(z_axis, [1, 0, 0])
if np.linalg.norm(x_axis) < 1e-6:  # If x_axis is too small, use a different vector
    x_axis = np.cross(z_axis, [0, 1, 0])
x_axis = x_axis / np.linalg.norm(x_axis)

# Calculate the y-axis as the cross product of z_axis and x_axis
y_axis = np.cross(z_axis, x_axis)

# Construct the rotation matrix from the axes
rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))


# Calculate the end-effector frame
# ... (same code as above to calculate rotation_matrix) ...

# Convert the rotation matrix to a quaternion
quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

# Create a Pose message with the desired position and orientation
target_pose = Pose()
target_pose.position.x = target_point[0]
target_pose.position.y = target_point[1]
target_pose.position.z = target_point[2]
target_pose.orientation.x = quaternion[0]
target_pose.orientation.y = quaternion[1]
target_pose.orientation.z = quaternion[2]
target_pose.orientation.w = quaternion[3]

# Move the robot's end-effector to the target pose
move_robot(target_pose)