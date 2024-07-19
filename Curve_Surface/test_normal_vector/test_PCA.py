import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def mean_normal_vector(point_clouds):
    normal_vector_list = []
    pass


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


def calculate_fitting_normalvector(source):
    # 평균점 구하기
    cp_source = np.mean(source, axis=0).reshape((1, 3))

    # centroid화
    X = source - cp_source

    # 공분산행렬 계산
    D = np.dot(X.T, X)

    # 특이값분해
    U, S, V_T = np.linalg.svd(D.T)

    # 근사행렬 == 회전행렬 계산
    R = np.dot(V_T, U)

    # reflection case <- SVD 문제
    if np.linalg.det(R) < 0:
        V_T[2, :] *= -1
        # R = np.dot(V_T,U)

    # normal vector 추출
    return V_T.T[:3, 2]


def estimation_normal_vector(source, radius=0.1, near_sample_num=15):
    # radius : 지정된 반경 범위
    # near_num : 근접점 갯수

    # normal vector를 계산하기 위한 최소 포인트 수
    point_num = 2

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

        # 지정된 반경 범위 보다 작은 index만 추출
        cond = np.where(distances < radius)

        # 지정된 반경내 매칭점 추출
        indices = indices[cond]
        distances = distances[cond]

        # 지정된 반경내 매칭점 갯수가 2개 이상일 때(자기자신포함)
        if len(indices) >= point_num + 1:

            # 조건들을 만족하는 가까운 점(point)들 추출
            near_points = source[indices]

            # 분산과 평면 fitting 이용한 normal vector 계산
            mean_normal_vector = calculate_fitting_normalvector(near_points)
            normal_vector_list.append(
                mean_normal_vector / np.linalg.norm(mean_normal_vector)
            )

        else:
            normal_vector_list.append(np.zeros((3)))

    return np.array(normal_vector_list)


def calculate_angle_between_vectors(b):

    a = [0, 0, 1]
    angle_2_vector = []
    for i in range(len(b)):
        # Calculate the dot product
        dot_product = np.dot(a, b[i])

        # Calculate the magnitudes of the vectors
        magnitude_a = np.linalg.norm(a)
        magnitude_b = np.linalg.norm(b[i])

        # Calculate the cosine of the angle
        cos_theta = dot_product / (magnitude_a * magnitude_b)

        # Calculate the angle in radians
        angle_radians = np.arccos(cos_theta)

        # Convert the angle to degrees
        angle_degrees = np.degrees(angle_radians)
        angle_2_vector.append(angle_degrees)

    print(f"angle:{angle_2_vector}")

    return angle_2_vector


if __name__ == "__main__":

    coord = o3d.geometry.TriangleMesh.create_coordinate_frame()

    # Generate the target point cloud representing a sinusoidal curve surface
    x = np.linspace(-40, 40, 20)
    y = np.linspace(-40, 40, 20)
    X, Y = np.meshgrid(x, y)
    Z = -(1 / 70) * (X**2) + -(1 / 70) * (Y**2)  # Phương trình parabol

    # X = X + 450
    # Y= Y -50
    # Z = Z - 60
    # print(f'X, Y, Z: {X, Y, Z}')
    np_curve_surface = np.stack([X, Y, Z], axis=-1)
    print(f"np_curve_surface: {np_curve_surface[5]}")
    source = np_curve_surface.reshape((-1, 3))
    pcd_show([source, coord])

    # source_cur = np_curve_surface.reshape((-1, 3)) + np.random.uniform(
    #     -0.05, 0.05, (1600, 3)
    # )
    # pcd_show([source_cur, coord])

    # source = np_curve_surface.reshape((-1, 3)) + np.random.uniform(
    #     -0.05, 0.05, (400, 3)
    # )
    # source = np_curve_surface.reshape((-1, 3))
    source_nv = estimation_normal_vector(source, 25, 60)
    print(f"source_nv:{source_nv[50:60]}")

    calculate_angle_between_vectors(source_nv[50:60])
    print(f"len(nr):{len(source_nv)}")

    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(np.asarray(source))
    source_pcd.normals = o3d.utility.Vector3dVector(np.asarray(source_nv))
    pcd_show([source_pcd, coord])
    pcd_show([coord])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(source[:, 0], source[:, 1], source[:, 2])
    ax.grid(False)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Points cloud on Surface")
    for point, normal in zip(source, source_nv):
        ax.quiver(
            point[0],
            point[1],
            point[2],
            normal[0],
            normal[1],
            normal[2],
            length=4,
            color="purple",
        )

    plt.show()


"""
    # 2. Noise data on plane
    plate_X = np.arange(0, 1.0, 0.1)
    plate_Y = np.arange(0, 1.0, 0.1)
    plate_X, plate_Y = np.meshgrid(plate_X, plate_Y)
    plate_Z = np.zeros_like(plate_X)
    np_plate = np.stack([plate_X, plate_Y, plate_Z], axis=-1)
    source = np_plate.reshape((-1, 3)) + np.random.uniform(-0.05, 0.05, (100, 3))

    # Define new plane coordinates in the XZ-plane
    plate_X_perpendicular = np.arange(0.0, 1.0, 0.1)
    plate_Z_perpendicular = np.arange(0.0, 1.0, 0.1)
    plate_X_perpendicular, plate_Z_perpendicular = np.meshgrid(
        plate_X_perpendicular, plate_Z_perpendicular
    )
    plate_Y_perpendicular = np.zeros_like(plate_X_perpendicular)

    # Stack the coordinates to form the new plane
    np_plate_perpendicular = np.stack(
        [plate_X_perpendicular, plate_Y_perpendicular, plate_Z_perpendicular], axis=-1
    )
    combined_plate = np.concatenate([np_plate, np_plate_perpendicular], axis=0)
    # Add noise to the new plane
    source_perpendicular = np_plate_perpendicular.reshape((-1, 3)) + np.random.uniform(
        -0.05, 0.05, (100, 3)
    )
    # pcd_show([source, source_perpendicular, coord])

    # np_plate = np.stack([plate_X, plate_Y, plate_Z], axis=-1)
    source = combined_plate.reshape((-1, 3)) + np.random.uniform(-0.05, 0.05, (200, 3))
    source_nv = estimation_normal_vector(source, 0.5, 100)

    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(np.asarray(source))
    source_pcd.normals = o3d.utility.Vector3dVector(np.asarray(source_nv))
    # pcd_show([source_pcd, coord])
"""
