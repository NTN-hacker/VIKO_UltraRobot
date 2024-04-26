from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import numpy as np
import time
import create_ref as cr
import sys

sys.path.append("D:\Quan\\roboDK\Vision-Machine-collab-Nhan\VIKO_UltraRobot")
from config import config as CFG
from datetime import datetime
import pandas as pd

# import keyboard
import rot_pos as rp
import lib.vision as vis  # temp
import dis_ob_cam as dis


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


def sleep_seconds(seconds):
    print(f"Sleeping for {seconds} seconds...")
    time.sleep(seconds)
    print("Awake now!")


def createPoint(arr_target, count, rot_camera2base):

    target2_newcamera = np.dot(rf_camera2camera, arr_target)
    # print(f'target2_newcamera:{target2_newcamera}')

    rf_target2base = np.dot(rf_camera2base, target2_newcamera)
    # print("target_ref_base:", rf_target2base, "\n")
    rot_target, pos_target = rp.rotPos(rf_target2base)

    # print("rot_target:", rot_target, "\n", "pos_target:", pos_target, "\n")

    # target_none_matrix = np.concatenate((pos_target, rot_target), axis=0)

    pos_laser_to_object = [pos_target[0] - 450, pos_target[1], pos_target[2]]
    target_laser_none_mat = np.concatenate(
        (pos_laser_to_object, rot_camera2base), axis=0
    )

    # print(f"target_laser_none_matrix {count}:{target_laser_none_mat}", "\n")

    # convert to pose
    target_laser_mat = TxyzRxyz_2_Pose(target_laser_none_mat)

    return target_laser_mat, pos_laser_to_object


def runRobot(target_laser_mat, pos_laser_to_object):
    speeds = CFG.SPEEDS  # cfg - TEST
    robot.setSpeed(speeds[1])

    while True:
        print(f"Target position: {pos_laser_to_object}")
        choice = input(
            "Do you want to continue? (yes/no): "
        ).lower()  # Convert input to lowercase to handle variations

        if choice == "y":
            try:
                robot.MoveL(target_laser_mat)
                get_joint = robot.Joints()
                print(f"get_joint:{get_joint}")
                break
            except Exception as e:
                print("Error:", e)
                get_joint = robot.Joints()
                print(f"get_joint:{get_joint}")
                break
        elif choice == "n":
            get_joint = robot.Joints()
            print(f"get_joint:{get_joint}")
            break
        else:
            print("Invalid input. Please enter 'y' or 'n'.")


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
    # success robot.ConnectSafe() # Try to connect multiple times
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
camera2base_non_matrix = np.concatenate((pos_camera2base, rot_camera2base))
print("rot_camera2base_non_matrix:", camera2base_non_matrix, "\n")
rf_camera2base_matrix = TxyzRxyz_2_Pose(camera2base_non_matrix)

pos_camera2camera = [0, 0, 0]
rot_camera2camera = [
    np.radians(0),
    np.radians(0),
    np.radians(90),
]
rf_camera2camera = cr.createRef(pos_camera2camera, rot_camera2camera)

pos_setFrame = [0, 0, 0]  # Translation vector [Tx, Ty, Tz] ## sai so
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

dis.cameraPosLeft(camera2base_non_matrix)

sleep_seconds(10)

dis.cameraPosRight(camera2base_non_matrix)

# coordinate_pixel = getCoordinates(True)

dis_pixel = []

x_left_dis, y_left_dis = dis.convertCoordinates(
    CFG.RESOLUTION_X, CFG.RESOLUTION_Y, dis_pixel[0], dis_pixel[1]
)  # x_ at left position
x_right_dis, y_right_dis = dis.convertCoordinates(
    CFG.RESOLUTION_X,
    CFG.RESOLUTION_Y,
    dis_pixel[0] + dis_pixel[2],
    dis_pixel[1] + dis_pixel[3],
)  # x_ at right position
dis_cameraToObject = dis.distanceCameraToObject(
    x_left_dis, x_right_dis, CFG.FOCAL_LENGTH
)  # distance from camera to object

robot.MoveJ(rf_camera2base_matrix)  # move home position

coordinate_pixel = []
x_to_camera_01, y_to_camera_01 = dis.convertCoordinates(
    CFG.RESOLUTION_X, CFG.RESOLUTION_Y, coordinate_pixel[0], coordinate_pixel[1]
)  # cfg

x_to_camera_02, y_to_camera_02 = dis.convertCoordinates(
    CFG.RESOLUTION_X,
    CFG.RESOLUTION_Y,
    coordinate_pixel[0] + coordinate_pixel[2],
    coordinate_pixel[1] + coordinate_pixel[3],
)  # cfg

# TEST FLOW
z_laser_to_camera = dis_cameraToObject - CFG.DISTANCE_LASERtoOBJECT
conv_real_target01, conv_real_target02 = dis.realTarget(
    x_to_camera_01,
    y_to_camera_01,
    x_to_camera_02,
    y_to_camera_02,
    CFG.DISTANCE_O1O2,
    CFG.FOCAL_LENGTH,
    z_laser_to_camera,
)

target01, pos_laser = createPoint(conv_real_target01, 1, rot_camera2base)  # fix
target02, pos_laser = createPoint(conv_real_target01, 2, rot_camera2base)

runRobot(target01, pos_laser)
runRobot(target02, pos_laser)

robot.MoveJ(rf_camera2base_matrix)


current_joint_values = robot.Joints()
target01 = np.array2string(target01)
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
    filename = f"Test_{time[:10]}.csv"

    try:
        # Read the existing CSV file into a DataFrame
        existing_dataframe = pd.read_csv(filename)
    except FileNotFoundError:
        # If the file doesn't exist, create a new DataFrame
        existing_dataframe = pd.DataFrame()

    # Create a DataFrame from the new data
    new_dataframe = pd.DataFrame(data)

    # Concatenate the existing and new dataframes
    updated_dataframe = pd.concat(
        [existing_dataframe, new_dataframe], ignore_index=True
    )

    # Save the updated DataFrame to the CSV file
    updated_dataframe.to_csv(filename, index=False)


data_export = {
    "id": str(datetime.now()),
    "target01": target01,
    "target02": target02,
    "limit_lower": limit[0],
    "limit_upper": limit[1],
    #### add if need
}

export_csv(data_export)
