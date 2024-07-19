import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors
import open3d as o3d


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
    print(f"normal_vector: {normal_vector}" + "\n")
    return normal_vector


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


def pcd_show(point_clouds=[]):
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


if __name__ == "__main__":
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # x = np.linspace(-1, 1, 10)
    # y = np.linspace(-1, 1, 10)
    # # x = np.random.uniform(1, 10, 10)
    # # y = np.random.uniform(1, 10, 10)
    # X, Y = np.meshgrid(x, y)
    # Z = -(X**2) + - Y**2  # Phương trình parabol
    # # Z = - (X**2) + - Y**2  # Phương trình parabol

    theta_sample_num = 50
    theta = np.linspace(0.0, 2 * np.pi, theta_sample_num)
    r = 1.0
    X = theta
    Y = r * np.sin(theta)
    Z = np.zeros_like(X)

    # target = np.stack([X.flatten(), Y.flatten(), Z.flatten()], axis=-1)
    target = np.stack([X, Y, Z], axis=-1)
    source = pcd_rotation(target, 45, 0, 0)

    print(f"target: {target}")

    # pcd_show([target, coord])
    # Hiển thị dưới dạng điểm point clouds
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    # ax.plot_surface(X, Y, Z, rstride=5, cstride=5, cmap="viridis", color="b")
    # ax.plot_surface(X, Y, Z, cmap="viridis", color="b")

    # ax.scatter(X, Y, Z, color="b")

    ax.scatter(source[:, 0], source[:, 1], source[:, 2])
    ax.grid(False)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Points cloud on Surface")
    # plt.show()

    source_nv = simple_normal_vector(source)
    print(f"source_nv: {source_nv}")

    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(np.asarray(source))
    source_pcd.normals = o3d.utility.Vector3dVector(np.asarray(source_nv))
    pcd_show([source_pcd, coord])

    for point, normal in zip(target, source_nv):
        ax.quiver(
            point[0],
            point[1],
            point[2],
            normal[0],
            normal[1],
            normal[2],
            length=0.2,
            color="purple",
        )

    plt.show()
