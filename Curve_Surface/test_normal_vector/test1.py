# import transformations as tf

# from geometry_msgs.msg import Pose

import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import open3d as o3d

# Assume target_point and target_normal are given
# target_point = [0.5, 0.2, 0.3]  # Example target point
# target_normal = [0.0, 0.0, 1.0]  # Example normal vector (pointing upwards)
# # Define the z-axis of the end-effector frame (normal vector)
# z_axis = target_normal

# # Define an arbitrary vector perpendicular to z_axis as the x-axis
# x_axis = np.cross(z_axis, [1, 0, 0])
# if np.linalg.norm(x_axis) < 1e-6:  # If x_axis is too small, use a different vector
#     x_axis = np.cross(z_axis, [0, 1, 0])
# x_axis = x_axis / np.linalg.norm(x_axis)

# # Calculate the y-axis as the cross product of z_axis and x_axis
# y_axis = np.cross(z_axis, x_axis)

# # Construct the rotation matrix from the axes
# rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))


# Calculate the end-effector frame
# ... (same code as above to calculate rotation_matrix) ...

# Convert the rotation matrix to a quaternion
# quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

# # Create a Pose message with the desired position and orientation
# target_pose = Pose()
# target_pose.position.x = target_point[0]
# target_pose.position.y = target_point[1]
# target_pose.position.z = target_point[2]
# target_pose.orientation.x = quaternion[0]
# target_pose.orientation.y = quaternion[1]
# target_pose.orientation.z = quaternion[2]
# target_pose.orientation.w = quaternion[3]

# # Move the robot's end-effector to the target pose
# move_robot(target_pose)


def pcd_rotation(point_cloud, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    roll_T = np.array(
        [
            [1, 0, 0],
            [0, np.cos(np.deg2rad(roll_deg)), -np.sin(np.deg2rad(roll_deg))],
            [0, np.sin(np.deg2rad(roll_deg)), np.cos(np.deg2rad(roll_deg))],
        ]
    )

    pitch_T = np.array(
        [
            [np.cos(np.deg2rad(pitch_deg)), 0, np.sin(np.deg2rad(pitch_deg))],
            [0, 1, 0],
            [-np.sin(np.deg2rad(pitch_deg)), 0, np.cos(np.deg2rad(pitch_deg))],
        ]
    )

    yaw_T = np.array(
        [
            [np.cos(np.deg2rad(yaw_deg)), -np.sin(np.deg2rad(yaw_deg)), 0],
            [np.sin(np.deg2rad(yaw_deg)), np.cos(np.deg2rad(yaw_deg)), 0],
            [0, 0, 1],
        ]
    )

    np_point_cloud = point_cloud.reshape((-1, 3))
    t_pcd = np.matmul(np_point_cloud, np.matmul(np.matmul(yaw_T, pitch_T), roll_T))
    print(np.matmul(np.matmul(yaw_T, pitch_T), roll_T))
    return t_pcd


def calculate_point_normalvector(p1, p2, p3, reference_point):
    # p1 기준점(reference point)
    # p1, p2, p3 data type [x, y, z]
    vector21 = p2 - p1
    vector31 = p3 - p1
    out_vector = np.cross(vector21, vector31)
    # cross product
    vector_size = np.linalg.norm(out_vector)
    # L2 norm
    normal_vector = out_vector / vector_size
    # unitize
    # Ensure the normal vector points in the direction of the reference point
    if np.dot(normal_vector, reference_point - p1) < 0:
        normal_vector = -normal_vector
    # print(f"normal_vector: {normal_vector}" + "\n")
    return normal_vector


def pcd_show1(point_clouds=[]):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for point_cloud in point_clouds:
        if isinstance(point_cloud, np.ndarray):
            ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2])

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Point Cloud Visualization")
    plt.show()


def pcd_show2(point_clouds=[]):
    show_list = []
    for point_cloud in point_clouds:
        if type(point_cloud).__module__ == np.__name__:
            np_point_cloud = np.array(point_cloud)
            np_point_cloud = np_point_cloud.reshape((-1, 3))
            o3d_point_cloud = o3d.geometry.PointCloud()
            o3d_point_cloud.points = o3d.utility.Vector3dVector(
                np.asarray(np_point_cloud)
            )
            show_list.append(o3d_point_cloud)
        else:
            show_list.append(point_cloud)
    o3d.visualization.draw_geometries(show_list, point_show_normal=True)


def simple_normal_vector(source):
    # normal vector를 계산하기 위한 최소 포인트 수

    near_sample_num = 3

    # 가까운 거리 search을 효율적으로하기 위하여
    neigh = NearestNeighbors(n_neighbors=near_sample_num)
    neigh.fit(source)

    normal_vector_list = []
    for _, src_point in enumerate(source):
        distances, indices = neigh.kneighbors(
            src_point.reshape(-1, 3), return_distance=True
        )

        # flatten
        distances = distances.ravel()
        indices = indices.ravel()

        p1, p2, p3 = source[indices]
        print(f"p1: {p1}, p2: {p2}, p3: {p3}")
        nv = calculate_point_normalvector(p1, p2, p3, src_point)

        normal_vector_list.append(nv)

    return np.array(normal_vector_list)


def plot_point_cloud_with_normals(source, normals):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plot point cloud
    ax.scatter(source[:, 0], source[:, 1], source[:, 2], c="b", label="Point Cloud")
    ax.scatter(0, 0, 0, c="m", label="x")
    ax.scatter(0, 0, 0, c="g", label="y")

    # Plot normals
    for i in range(len(source)):
        ax.quiver(
            source[i, 0],
            source[i, 1],
            source[i, 2],
            normals[i, 0],
            normals[i, 1],
            normals[i, 2],
            length=0.3,
            color="r",
        )

        axes_length = 0.3  # Length of axes

        ax.quiver(
            0,
            0,
            0,
            x_axis[i, 0],
            x_axis[i, 1],
            x_axis[i, 2],
            length=axes_length,
            color="m",
        )
        ax.quiver(
            0,
            0,
            0,
            y_axis[i, 0],
            y_axis[i, 1],
            y_axis[i, 2],
            length=axes_length,
            color="g",
        )
        # ax.quiver(0, 0, 0, z_axis[i, 0], z_axis[i, 1], z_axis[i, 2], length=axes_length, color='b', label='Z Axis')

    ax.grid(False)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Point Cloud with Normal Vectors")
    ax.legend()
    plt.show()


def determine_axes(normal_vector):
    z_axis = normal_vector
    x_axis = np.cross(z_axis, [1, 0, 0])
    X_axis = []
    Y_axis = []

    for i in range(len(normal_vector)):
        x_axis = np.cross(z_axis[i], [1, 0, 0])
        if (
            np.linalg.norm(x_axis) < 1e-6
        ):  # If x_axis is too small, use a different vector
            x_axis = np.cross(z_axis[i], [0, 1, 0])
        x_axis = x_axis / np.linalg.norm(x_axis)
        X_axis.append(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        Y_axis.append(y_axis)

    return np.array(X_axis), np.array(Y_axis), np.array(z_axis)


if __name__ == "__main__":
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    theta_sample_num = 50
    theta = np.linspace(0.0, 2 * np.pi, theta_sample_num)
    r = 1.0
    x = theta
    y = r * np.sin(theta)
    z = np.zeros_like(x)

    # Generate the target point cloud representing a sinusoidal curve surface
    target = np.stack([x, y, z], axis=-1)

    source = pcd_rotation(target, 90, 0, 0)

    pcd_show1([source])

    source_nv = simple_normal_vector(source)
    print(f"type(source_nv): {source_nv}")

    x_axis, y_axis, z_axis = determine_axes(source_nv)
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(np.asarray(source))
    source_pcd.normals = o3d.utility.Vector3dVector(np.asarray(source_nv))
    pcd_show2([source_pcd, coord])

    plot_point_cloud_with_normals(source, source_nv)
