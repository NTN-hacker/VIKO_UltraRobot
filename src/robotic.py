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
    

# def convertCoordinates(focal_length, res_height, res_width, pixel_x, pixel_y, distance):                    
def convertCoordinates(res_height, res_width, pixel_x, pixel_y, distance, focal_length):

    sensor_height = res_height * 3.5 / 1000 #cfg
    sensor_width = res_width * 3.5 / 1000 #cfg
    sensor_size = np.array([sensor_height, sensor_width])
    print("sensor_size:", sensor_size, "\n")

    xpixel_center = res_width / 2
    ypixel_center = res_height / 2

    xpixel_2center = pixel_x - xpixel_center
    ypixel_2center = pixel_y - ypixel_center
    print("xpixel2center:", xpixel_2center, "\n")

    x_2_camera = xpixel_2center * sensor_size[1] / (2 * xpixel_center)
    y_2_camera = ypixel_2center * sensor_size[0] / (2 * ypixel_center)

    print("x2camera:", x_2_camera, "\n")
    print("y2camera:", y_2_camera, "\n")


    x_real = x_2_camera * distance / focal_length
    y_real = y_2_camera * distance / focal_length
    print("x_real:", x_real, "\n")
    print("y_real:", y_real, "\n")
    # return spatial_x, spatial_y, hFOV, vFOV
    return x_real, y_real


def sleep_seconds(seconds):
    print(f"Sleeping for {seconds} seconds...")
    time.sleep(seconds)
    print("Awake now!")


def createPoint(x_st, y_st, z_st, count):

    target = [x_st, y_st, z_st]

    target2camera = intialTarget(target[0], target[1], target[2])
    target2_newcamera = np.dot(rf_camera2camera, target2camera)
    # print(f'target2_newcamera:{target2_newcamera}')
    # rf_target2camera = np.dot(rf_surface2camera, target_pose)

    rf_target2base = np.dot(rf_camera2base, target2_newcamera)
    # print("target_ref_base:", rf_target2base, "\n")
    rot_target, pos_target = rp.rotPos(rf_target2base)

    # print("rot_target:", rot_target, "\n", "pos_target:", pos_target, "\n")

    target_none_matrix = np.concatenate((pos_target, rot_target), axis=0)
    print(f"target_none_matrix {count}:{target_none_matrix}", "\n")

    if abs(target_none_matrix[2]) > 300:
        print('out of limit')
        target_none_matrix[2] = -330
        # robot.Disconnect()

    # convert to pose
    target_matrix = TxyzRxyz_2_Pose(target_none_matrix)

    speeds = CFG.SPEEDS #cfg - TEST
    robot.setSpeed(speeds[1])

    try:
        # robot.MoveL(target_matrix)
        get_joint = robot.Joints()
        pass
    except Exception as e:
        print('Error:', e)
        get_joint = robot.Joints()
    return target_none_matrix, get_joint

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
    # success = robot.Connect()  # Try to connect once
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
pos_camera2flange = [-15.5, -55.5, 150]  # Translation vector [Tx, Ty, Tz] ## sai so
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

pos_camera2camera = [0, 0, 0]
rot_camera2camera = [
    np.radians(0),
    np.radians(0),
    np.radians(90),
]
rf_camera2camera = cr.createRef(pos_camera2camera, rot_camera2camera)

pos_setFrame= [0, 0, 0]  # Translation vector [Tx, Ty, Tz] ## sai so
rot_setFramee = [
    np.radians(0),
    np.radians(0),
    np.radians(0),
]  # Rotation angles [Rx, Ry, Rz] in radians
rf_setFrame = np.concatenate((pos_setFrame, rot_setFramee))
setFrame = TxyzRxyz_2_Pose(rf_setFrame)

robot.setPoseFrame(setFrame)
# print(f"robot.PoseFrame():{robot.PoseFrame()}")
robot.setPoseTool(rf_camera2flange_matrix)
# print(f"robot.PoseTool():{robot.PoseTool()}")
robot.setRounding(5)  # Set the rounding parameter
robot.setSpeed(10, 10)  # Set linear speed in mm/s
robot.setSpeedJoints(10)

print("rf_camera2base_matrix:", rf_camera2base_matrix)
robot.MoveJ(rf_camera2base_matrix)

# sleep_seconds(5)

# coordinate_pixel = getCoordinates(True)
coordinate_pixel = [899, 67, 883, 1737]

x_target, y_target = convertCoordinates(CFG.RESOLUTION_Y, CFG.RESOLUTION_X, coordinate_pixel[0],\
                                                                                                    coordinate_pixel[1], CFG.DISTANCE_2OBJECT, CFG.FOCAL_LENGTH) #cfg


target01, joint1 = createPoint(x_target, y_target, CFG.DISTANCE_2OBJECT, None) #fix

coordinate_pixel = [920, 184, 896, 1733]
#TEST FLOW
x_target_02, y_target_02 = convertCoordinates(CFG.RESOLUTION_Y, CFG.RESOLUTION_X, coordinate_pixel[0] + coordinate_pixel[2],\
                                                                                                    coordinate_pixel[1] + coordinate_pixel[3], CFG.DISTANCE_2OBJECT, CFG.FOCAL_LENGTH)


target02, joint2 = createPoint(x_target_02, y_target_02, CFG.DISTANCE_2OBJECT, None)

robot.MoveJ(rf_camera2base_matrix)

current_joint_values = robot.Joints()
# target01 = np.array2string(target01)
target02 = np.array2string(target02)
limit = robot.JointLimits()

# print(f'limit:{limit}')


def export_csv(data: dict):
    """
    Export to excel or csv
    """
    time = str(datetime.now())
    # dataframe = pd.DataFrame(data)
    # dataframe.to_csv(f'Test_{time[:10]}.csv')
    filename = f'Test_{time[:10]}.csv'

    try:
        # Read the existing CSV file into a DataFrame
        existing_dataframe = pd.read_csv(filename)
    except FileNotFoundError:
        # If the file doesn't exist, create a new DataFrame
        existing_dataframe = pd.DataFrame()

    # Create a DataFrame from the new data
    new_dataframe = pd.DataFrame(data)

    # Concatenate the existing and new dataframes
    updated_dataframe = pd.concat([existing_dataframe, new_dataframe], ignore_index=True)

    # Save the updated DataFrame to the CSV file
    updated_dataframe.to_csv(filename, index=False)

data_export = {'id': str(datetime.now()), 
    # 'target01': target01,
    # 'joint1': joint1,
    'target02': target02,
    'joint2': joint2,
    'limit_lower': limit[0],
    'limit_upper': limit[1]

    #### add if need
    }

export_csv(data_export)

