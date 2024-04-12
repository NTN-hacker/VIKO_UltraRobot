from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import numpy as np
import time
import create_ref as cr
import sys
sys.path.append('D:\Quan\\roboDK\Vision-Machine-collab-Nhan\VIKO_UltraRobot')
from config import config as CFG
from datetime import datetime
import pandas as pd
# import keyboard
import rot_pos as rp
import lib.vision as vis#temp

def getCoordinates(flag: bool):
    obj = vis.VisionModule()
    obj._start_()
    obj._getImage_()
    obj.load_model()
    # obj.save_image()
    coordinate = obj._getCoordinate_()
    obj._end_()
    return coordinate

def drawPolygon(target_rf, n_sides, R):

    for i in range(n_sides + 1):
        ang = i * 2 * pi / n_sides  # angle: 0, 60, 120, ...
        # target_matrix = TxyzRxyz_2_Pose(pos_i)
        # -----------------------------
        # Movement relative to the reference frame
        # Create a copy of the target
        target_draw_1 = Mat(target_rf)
        print(f"target_i: {target_draw_1}")
        pos_draw_i = target_draw_1.Pos()
        pos_draw_i[0] = pos_draw_i[0] + R * cos(ang)
        pos_draw_i[1] = pos_draw_i[1] + R * sin(ang)
        target_draw_1.setPos(pos_draw_i)
        print("Moving to target %i: angle %.1f" % (i, ang * 180 / pi))
        # print(str(Pose_2_TxyzRxyz(target_i)))
        robot.setSpeed(0)
        robot.MoveL(target_draw_1)
    return


def intialTarget(x, y, z):
    # target_pos_initial = np.array([200, 200, 200, 1])
    target_pos_initial = np.array([x, y, z, 1])

    target_pose = np.array(
        [
            [1, 0, 0, target_pos_initial[0]],
            [0, 1, 0, target_pos_initial[1]],
            [0, 0, 1, target_pos_initial[2]],
            [0, 0, 0, 1],
        ]
    )
    return target_pose

""" Invertible matrix
def changeJoint(rf_camera2base, rf_surface2base):

    rf_base2camera = np.linalg.inv(rf_camera2base)
    print("ref_base2camera:", rf_base2camera, "\n")
    print("ref_surface2base:", rf_surface2base, "\n")
    rf_surface2camera = np.dot(rf_base2camera, rf_surface2base)
    print("ref_surface2camera:", rf_surface2camera, "\n")

    rot, pos = rp.rotPos(rf_surface2camera)
    print("rot:", rot, "\n", "pos:", pos, "\n")

    return
"""
    

def convertCoordinates(focal_length, res_height, res_width, pixel_x, pixel_y, distance):                    

    sensor_height = res_height * CFG.PIXEL_SIZE / 1000 #cfg
    sensor_width = res_width * CFG.PIXEL_SIZE / 1000 #cfg
    sensor_size = np.array([sensor_height, sensor_width])
    print("sensor_size:", sensor_size, "\n")

    hFOV = (distance * sensor_size[0]) / focal_length
    vFOV = (distance * sensor_size[1]) / focal_length
    print("hFOV:", hFOV, "\n", "vFOV:", vFOV, "\n")

    # spatial_x = []
    # spatial_y = []
    # for i in range (len(pixel_x)):
    #     spatial_x = pixel_x[i] * hFOV / res_width
    #     spatial_y = pixel_y[i] * vFOV / res_height
    #     print("spatial_x:", spatial_x, "\n", "spatial_y:", spatial_y, "\n")

    spatial_x = pixel_x * hFOV / res_width
    spatial_y = pixel_y * vFOV / res_height
    print("spatial_x:", spatial_x, "\n", "spatial_y:", spatial_y, "\n")

    return spatial_x, spatial_y, hFOV, vFOV


def sleep_seconds(seconds):
    print(f"Sleeping for {seconds} seconds...")
    time.sleep(seconds)
    print("Awake now!")


def createPoint(x_st, y_st, z_st):

    x_axis = x_st
    y_axis = y_st
    z_axis = z_st

    target_points = [x_axis, y_axis, z_axis]

    target_pose = intialTarget(target_points[0], target_points[1], target_points[2])

    rf_target2camera = np.dot(rf_surface2camera, target_pose)

    rf_target2base = np.dot(rf_camera2base, rf_target2camera)
    # print("target_ref_base:", rf_target2base, "\n")
    rot_target, pos_target = rp.rotPos(rf_target2base)

    # print("rot_target:", rot_target, "\n", "pos_target:", pos_target, "\n")

    target_none_matrix = np.concatenate((pos_target, rot_target), axis=0)
    print("target_none_matrix:", target_none_matrix, "\n")

    # convert to pose
    target_matrix = TxyzRxyz_2_Pose(target_none_matrix)

    speeds = CFG.SPEEDS #cfg - TEST
    robot.setSpeed(speeds[1])
    robot.MoveL(target_matrix)

    return 


""" forward kinematics
def transformation_matrix(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Các góc xoay của các khớp (radian)
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = np.radians(-90)
theta6 = np.radians(90)

# Tạo ma trận biến đổi cho từng khớp
T1 = transformation_matrix(0, 0, 0, theta1)
T2 = transformation_matrix(0, 0, 0, theta2)
T3 = transformation_matrix(0, 0, 0, theta3)
T4 = transformation_matrix(0, 0, 0, theta4)
T5 = transformation_matrix(0, 0, 0, theta5)
T6 = transformation_matrix(0, 0, 0, theta6)

# Tích các ma trận biến đổi
T_final = np.dot(np.dot(np.dot(np.dot(np.dot(T1, T2), T3), T4), T5), T6)

# Trục hệ tọa độ của mỗi khớp
axes = {
    'Khớp 1': T1[:3, :3],
    'Khớp 2': T2[:3, :3],
    'Khớp 3': T3[:3, :3],
    'Khớp 4': T4[:3, :3],
    'Khớp 5': T5[:3, :3],
    'Khớp 6': T6[:3, :3]
}

for k, v in axes.items():
    print(k + ':')
    print(v)
    print()

# Vị trí của khâu tác động cuối cùng (cột 4 của ma trận biến đổi)
position = T_final[:3, 3]
# Hướng của khâu tác động cuối cùng (các phần tử của cột 1, 2, 3 của ba cột đầu tiên)
orientation = T_final[:3, :3]

print("Vị trí của khâu tác động cuối cùng:", position)
print("Hướng của khâu tác động cuối cùng:", orientation)

# Các góc xoay của các khớp (radian)
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = np.radians(-90)
theta6 = np.radians(90)
"""
RDK = Robolink()

# robot = RDK.AddFile("E:\\Install-software\\RoboDK\\Library\\Motoman-GP8.robot")
robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("No robot selected or available")


RUN_ON_ROBOT = True

if RDK.RunMode() != RUNMODE_SIMULATE:
    RUN_ON_ROBOT = False

if RUN_ON_ROBOT:
    
    # Connect to the robot using default IP
    success = robot.Connect()  # Try to connect once
    #success robot.ConnectSafe() # Try to connect multiple times
    status, status_msg = robot.ConnectedState()
    print(status)
    print(status_msg)
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        print(status_msg)
        raise Exception("Failed to connect: " + status_msg)

    # This will set to run the API programs on the robot and the simulator (online programming)
    RDK.setRunMode(RUNMODE_RUN_ROBOT)

robot.ConnectedState()
print("ConnectedState:", robot.ConnectedState(), "\n")


# reference frame from flange to base
pos_flange2base = [380, 0, 305]  # Translation vector [Tx, Ty, Tz]
rot_flange2base = [
    np.radians(-180),
    np.radians(0),
    np.radians(0),
]  # Rotation angles [Rx, Ry, Rz] in radians

# Create the transformation matrix for the new rf flange to base
rf_flage2base = cr.createRef(pos_flange2base, rot_flange2base)
print("rf_flage2base:", rf_flage2base, "\n")


# reference frame camera to flange
pos_camera2flange = [0, -30, 200]  # Translation vector [Tx, Ty, Tz] ## sai so
rot_camera2flange = [
    np.radians(0),
    np.radians(0),
    np.radians(0),
]  # Rotation angles [Rx, Ry, Rz] in radians

# Create the transformation matrix for the new rf camera to flange
rf_camera2flange = cr.createRef(pos_camera2flange, rot_camera2flange)
print("ref_camera2flange:", rf_camera2flange, "\n")

rf_camera2flange_non_matrix = np.concatenate((pos_camera2flange, rot_camera2flange))
rf_camera2flange_matrix = TxyzRxyz_2_Pose(rf_camera2flange_non_matrix)
print("ref_camera2flange_test:", rf_camera2flange_matrix, "\n")

# reference frame camera to base
rf_camera2base = np.dot(rf_flage2base, rf_camera2flange)
print("ref_camera2base:", rf_camera2base, "\n")

rot_camera2base, pos_camera2base = rp.rotPos(rf_camera2base)
# print(
#     "rot_camera2base:", rot_camera2base, "\n", "pos_camera2base:", pos_camera2base, "\n"
# )
rot_camera2base_non_matrix = np.concatenate((pos_camera2base, rot_camera2base))
print("rot_camera2base_non_matrix:", rot_camera2base_non_matrix, "\n")
rf_camera2base_matrix = TxyzRxyz_2_Pose(rot_camera2base_non_matrix)

robot.setPoseFrame(robot.PoseFrame())
# print(f"robot.PoseFrame():{robot.PoseFrame()}")
robot.setPoseTool(rf_camera2flange_matrix)
print(f"robot.PoseTool():{robot.PoseTool()}")
robot.setRounding(5)  # Set the rounding parameter
robot.setSpeed(10, 10)  # Set linear speed in mm/s
robot.setSpeedJoints(10)

print("rf_camera2base_matrix:", rf_camera2base_matrix)
robot.MoveJ(rf_camera2base_matrix)

sleep_seconds(10)

coordinate_pixel = getCoordinates(True)

x_target, y_target, h, w = convertCoordinates(CFG.FOCAL_LENGTH, CFG.RESOLUTION_X, CFG.RESOLUTION_Y, coordinate_pixel[-1],\
                                                                                                    coordinate_pixel[0], CFG.DISTANCE_2OBJECT) #cfg

# pos_img2sur = [-(w / 2), -(h / 2), 0] # sai so
# rot_img2sur = [0, 0, 0]

# rf_img2sur = cr.createRef(pos_img2sur, rot_img2sur)
# print("rf_img2sur:", rf_img2sur, "\n")

# reference frame surface to base
pos_surface2camera = [-(h / 2), -(w / 2), 480]  # Translation vector [Tx, Ty, Tz]
rot_surface2camera = [0, 0, 0]  # Rotation angles [Rx, Ry, Rz] in radians

# Create the transformation matrix for the new rf surface to base
rf_surface2camera = cr.createRef(pos_surface2camera, rot_surface2camera)
print("rf_surface2camera:", rf_surface2camera, "\n")

createPoint(x_target, y_target, 0)
#TEST FLOW
x_target_02, y_target_02, h, w = convertCoordinates(CFG.FOCAL_LENGTH, CFG.RESOLUTION_X, CFG.RESOLUTION_Y, list(coordinate_pixel)[-1] + 800,\
                                                                                                    list(coordinate_pixel)[0], CFG.DISTANCE_2OBJECT)
createPoint(x_target_02, y_target_02, 0)

# robot.MoveJ(fixed_target_matrix)

robot.MoveJ(rf_camera2base_matrix)
current_joint_values = robot.Joints()
print("current_joint_values:", current_joint_values, "\n")

# robot.setJoints([0, 0, 0, 0, -90, 0])

def export_csv(data: dict):
    """
    Export to excel or csv
    """
    time = str(datetime.now())
    data = {'id':  time, 
            'Current Joint Values': current_joint_values,
            #### add if need
            }

    dataframe = pd.DataFrame(data)
    dataframe.to_csv(f'Test_{time[:10]}.csv')
