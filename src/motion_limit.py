import sys
import threading

# sys.path.append("E:\\Project\\Robot-6DOF\\VIKO_UltraRobot")
sys.path.append("D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot")
from layout import app_robot as app
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
from src import robotic_modify as RM
import numpy as np
from sklearn.neighbors import NearestNeighbors
from library import robot_lib as rl



class TestThread:

    def __init__(self):
        self.rob_mod = RM.VisionRobot()
        self.rob_mod.pickRobot()
        self.rob_mod.fixedRef()
        self.rob_mod.attRobot()
        self.robot_module = rl.RobotModule()

    def point(self):

        x = np.linspace(-40, 40, 10)
        y = np.linspace(-40, 40, 10)
        X, Y = np.meshgrid(x, y)
        Z = (-1 / 120) * (X**2) + (-1 / 120) * (Y**2)  # Phương trình parabol

        X = X + 500
        Y= Y - 50
        Z = Z - 150
        # print(f'X, Y, Z: {X, Y, Z}')
        np_curve_surface = np.stack([X, Y, Z], axis=-1)
        points = np_curve_surface[5]
        print(f'point:{points}')

        source = np_curve_surface.reshape((-1, 3))

        return points, source

    def calculate_angle_between_vectors(self, b):
   
        a = [0, 0, 1]
        oy_vecto = [1, 0, 0]
        angle_2_vector = []
        angle_2_oy_vector = []
        for i in range (len(b)):
            # Calculate the dot product
            dot_product = np.dot(a, b[i])
            dot_dev = np.dot(oy_vecto, b[i])
            
            # Calculate the magnitudes of the vectors
            magnitude_a = np.linalg.norm(a)
            magnitude_b = np.linalg.norm(b[i])
            magnitude_c = np.linalg.norm(oy_vecto)
            
            # Calculate the cosine of the angle
            cos_theta = dot_product / (magnitude_a * magnitude_b)
            cos_theta_02 = dot_dev / (magnitude_c * magnitude_b)

            # Calculate the angle in radians
            angle_radians = np.arccos(cos_theta)
            angle_radians_02 = np.arccos(cos_theta_02)

            angle_degrees_02 = np.degrees(angle_radians_02)
            angle_2_oy_vector.append(angle_degrees_02)

            if angle_degrees_02 <= 90:
            
            # Convert the angle to degrees
                angle_degrees = np.degrees(angle_radians) 
                angle_2_vector.append(angle_degrees)

            elif angle_degrees_02 > 90:
                angle_degrees = np.degrees(angle_radians) - 90
                angle_2_vector.append(angle_degrees)

            # else:
            #     print("check angle")
  

        print(f'angle:{angle_2_vector}')
        print(f'angle_oy:{angle_2_oy_vector}')

        
        return angle_2_vector
    @staticmethod
    def sleep_seconds(seconds):
        print(f"Sleeping for {seconds} seconds...")
        time.sleep(seconds)
        print("Awake now!")
    
    # def move(self, check_target, angle_2_vector):
    def move(self):

        # Default limit space in the Cartesian space
        DEFAULT_XYZ_MIN = np.array([199, -480, -300])
        DEFAULT_XYZ_MAX = np.array([727, 480, 305])

        rot = np.array([np.radians(-180), np.radians(0), np.radians(0)])

        i = j = 0
        # print(f'target:{(check_target)}')
        # print(f'target:{(check_target[0][0])}')    
        # for i in range (len(check_target)):
            
        #     for j in range(3):
        #         target_1_within_range = all(DEFAULT_XYZ_MIN[j] <= check_target[i][j] <= DEFAULT_XYZ_MAX[j] for j in range(3))


        #         if target_1_within_range: 
        #             rot = [rot[0], np.radians(angle_2_vector[i]), rot[2]]
        #             target2base_none_mat1 = np.concatenate((check_target[i], rot), axis=0)
        #             target2base_mat1 = TxyzRxyz_2_Pose(target2base_none_mat1)  

        #         else:
        #             print("Target 1 is outside the default range.")


            # self.rob_mod.runMoveJ(target2base_mat1, None)
            # print(f"target_laser_mat:{target2base_mat1}")
        # self.robot_module.cameraPosLeft(self.rob_mod.rf_laser2base)
        # self.sleep_seconds(20)
        self.robot_module.cameraPosRight(self.rob_mod.rf_laser2base)
        self.sleep_seconds(20)
        # self.rob_mod.attRobot()



    def calculate_fitting_normalvector(self, source):
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


    def estimation_normal_vector(self, source, radius=0.1, near_sample_num=15):
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
                mean_normal_vector = self.calculate_fitting_normalvector(near_points)
                normal_vector_list.append(
                    mean_normal_vector / np.linalg.norm(mean_normal_vector)
                )

            else:
                normal_vector_list.append(np.zeros((3)))
            
        print(f'nor:{normal_vector_list[50:60]}')

        return np.array(normal_vector_list)



if __name__ == "__main__":
    # app.main()
    test = TestThread()
    test.move()
    # points, source = test.point()
    # source_nv = test.estimation_normal_vector(source, 30, 60)
    # angle_2_vector = test.calculate_angle_between_vectors(source_nv[50:60])
    # test.move(points, angle_2_vector)
