from datetime import datetime
import pandas as pd
import numpy as np
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import sys
import os
import time
import math


sys.path.append("D:\Quan\\roboDK\Vision-Machine-collab-Nhan\VIKO_UltraRobot") # config path
from config import config as CFG

########################################################

def rotPos(trans_matrix):
        roll = np.arctan2(trans_matrix[2, 1], trans_matrix[2, 2])
        pitch = np.arctan2(
            -trans_matrix[2, 0],
            np.sqrt(trans_matrix[2, 1] ** 2 + trans_matrix[2, 2] ** 2),
        )
        yaw = np.arctan2(trans_matrix[1, 0], trans_matrix[0, 0])

        rot = np.array([roll, pitch, yaw])
        pos = trans_matrix[:3, 3]

        return pos, rot

def rotPosRef(x, y, z, roll, pitch, yaw)->list:
        rotRef = np.array([np.radians(roll), np.radians(pitch), np.radians(yaw)])
        posRef = np.array([x, y, z])
        return posRef, rotRef
    
def createRef(translation, rotation):
    # Translation matrix
    translation_matrix = np.array(
        [
            [1, 0, 0, translation[0]],
            [0, 1, 0, translation[1]],
            [0, 0, 1, translation[2]],
            [0, 0, 0, 1],
        ]
    )

    # Rotation matrices (assuming XYZ Euler angles)
    rotation_x = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(rotation[0]), -np.sin(rotation[0]), 0],
            [0, np.sin(rotation[0]), np.cos(rotation[0]), 0],
            [0, 0, 0, 1],
        ]
    )

    rotation_y = np.array(
        [
            [np.cos(rotation[1]), 0, np.sin(rotation[1]), 0],
            [0, 1, 0, 0],
            [-np.sin(rotation[1]), 0, np.cos(rotation[1]), 0],
            [0, 0, 0, 1],
        ]
    )

    rotation_z = np.array(
        [
            [np.cos(rotation[2]), -np.sin(rotation[2]), 0, 0],
            [np.sin(rotation[2]), np.cos(rotation[2]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    # Combine translation and rotation
    transformation_matrix = np.dot(
        translation_matrix, np.dot(rotation_x, np.dot(rotation_y, rotation_z))
    )

    return transformation_matrix

def rotLaser(coordinate_pixel):
        pixel_1 = coordinate_pixel[0]
        pixel_2 = coordinate_pixel[1]

        angle = math.atan2(pixel_1[1] - pixel_2[1], pixel_1[0] - pixel_2[0])
        theta_laser = math.degrees(angle)
        print(f"theta_laser:{theta_laser}")
        # if degree_angle > 90:
            # theta_laser = 180 - degree_angle
        # else:
        #     theta_laser = degree_angle
        
        # print(f"theta_laser:{theta_laser}")

        return theta_laser

def export_csv(data: dict):
    """
    Export to excel or csv
    """
    time = str(datetime.now())
    filename = f"data/Robotic/Test_{time[:10]}.csv"

    directory = os.path.dirname(filename)
    if not os.path.exists(directory):
        os.mkdir(directory)

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

def distanceCameraToObject(x1_dis, x2_dis, focal_length, baseLine):     
    pixel_focalLength = focal_length * 1000 / CFG.PIXEL_SIZE
    dis_cameraToObject = baseLine * pixel_focalLength / (abs(x1_dis - x2_dis))
    return dis_cameraToObject, pixel_focalLength

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

def convertCoordinates(
        res_width,
        res_height,
        pixel_x,
        pixel_y,
        pixel_focalLength,
        dis_cameraToObject,
        theta,
    ):
        xpixel_to_center = pixel_x - res_width / 2
        ypixel_to_center = pixel_y - res_height / 2

        pixel_XY = np.array([[xpixel_to_center], [ypixel_to_center], [1]])

        rot_trans_XY = np.array(
            [
                [np.cos(np.radians(theta)), -np.sin(np.radians(theta)), 0],
                [np.sin(np.radians(theta)), np.cos(np.radians(theta)), 0],
                [0, 0, 1],
            ]
        )

        new_XY = np.dot(rot_trans_XY, pixel_XY)

        x1_real = new_XY[0][0] * dis_cameraToObject / pixel_focalLength
        y1_real = new_XY[1][0] * dis_cameraToObject / pixel_focalLength

        return x1_real, y1_real


def sleep_seconds(seconds):
    print(f"Sleeping for {seconds} seconds...")
    time.sleep(seconds)
    print("Awake now!")
########################################################


class RobotModule:
    def __init__(self):
        self.robot = Robolink().ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)

    def move(self, matcamera2base, direction):
        pos, rot = rotPos(matcamera2base)
        rot = [rot[0], rot[1], rot[2]]

        if direction == "left":
            pos = [pos[0], pos[1] + CFG.VERTICAL_BASELINE / 2, pos[2]]
        elif direction == "right":
            pos = [pos[0], pos[1] - CFG.VERTICAL_BASELINE / 2, pos[2]]
        elif direction == "forward":
            pos = [pos[0] + CFG.HORIZONTAL_BASELINE / 2, pos[1], pos[2]]
        elif direction == "back":
            pos = [pos[0] - CFG.HORIZONTAL_BASELINE / 2, pos[1], pos[2]]
        else:
            raise ValueError("Invalid direction. Use 'left', 'right', 'forward', or 'back'.")

        camera2base_nonmat = np.concatenate((pos, rot))
        camera2base_pose = TxyzRxyz_2_Pose(camera2base_nonmat)
        self.robot.MoveJ(camera2base_pose)

    def moveLeft(self, matcamera2base):
        self.move(matcamera2base, "left")

    def moveRight(self, matcamera2base):
        self.move(matcamera2base, "right")

    def moveForward(self, matcamera2base):
        self.move(matcamera2base, "forward")

    def moveBack(self, matcamera2base):
        self.move(matcamera2base, "back")
    
    def connectRobot(self)-> None:
        """
        Connect Robot and PC
        """
        if not self.robot.Valid():
            raise Exception("Invalid robot selected")

        RUN_ON_ROBOT = True
        if self.robot.RunMode() != RUNMODE_SIMULATE: #CHANGE
            RUN_ON_ROBOT = False

        self.robot.setRunMode(RUNMODE_RUN_ROBOT) #CHANGE
        if RUN_ON_ROBOT:
            # Connect to the robot using default IP
            self.robot.Connect("192.168.10.111")  # Try to connect once
            self.robot.ConnectSafe("192.168.10.111")  # Try to connect multiple times
            self.status, status_msg = self.robot.ConnectedState()
            print("self.status", self.status)
            if self.status != ROBOTCOM_READY:
                # Stop if the connection did not succeed
                print(status_msg)
                raise Exception("Failed to connect: " + status_msg)

            # This will set to run the API programs on the robot and the simulator (online programming)
            self.robot.setRunMode(RUNMODE_RUN_ROBOT)
            # self.RDK.CloseRoboDK()
        
        print("ConnectedState:", self.robot.ConnectedState(), "\n")

    def disConnectRobot(self) -> None:
        """
        Disconnect PC RObot
        """
        self.status, _ = self.robot.ConnectedState()
        print(f"status:{self.status}")
        if self.status == ROBOTCOM_READY:
            self.robot.Disconnect()
    
    def fixedRef(self):
        """
        Fixed reference
        """

        def print_reference(name, ref):
            print(f"{name}:\n{ref}\n")

        # Reference frame flange to base
        pos_flange2base, rot_flange2base = rotPosRef(380, 0, 405, 180, 0, 0)
        rf_flange2base = createRef(pos_flange2base, rot_flange2base)
        print_reference("rf_flange2base", rf_flange2base)

        # Reference frame camera to flange
        pos_camera2flange, rot_camera2flange = rotPosRef(59, 0, 190, 0, 0, 0)
        rf_camera2flange = createRef(pos_camera2flange, rot_camera2flange)
        print_reference("rf_camera2flange", rf_camera2flange)

        # Reference frame camera to base
        rf_camera2base = np.dot(rf_flange2base, rf_camera2flange)
        print_reference("rf_camera2base", rf_camera2base)

        # Reference frame laser to camera
        pos_laser2camera, rot_laser2camera = rotPosRef(-54.43, 57, 0, 0, 0, 0)
        rf_laser2camera = createRef(pos_laser2camera, rot_laser2camera)

        # Reference frame laser to flange
        rf_laser2flange = np.dot(rf_camera2flange, rf_laser2camera)
        pos_laser2flange, rot_laser2flange = rotPos(rf_laser2flange)
        rf_laser2flange_non_matrix = np.concatenate((pos_laser2flange, rot_laser2flange))
        rf_laser2flange_matrix = TxyzRxyz_2_Pose(rf_laser2flange_non_matrix)
        print_reference("rf_laser2flange_matrix", rf_laser2flange_matrix)

        # Reference frame laser to base
        rf_laser2base = np.dot(rf_camera2base, rf_laser2camera)
        pos_laser2base, rot_laser2base = rotPos(rf_laser2base)
        rf_laser2base_non_matrix = np.concatenate((pos_laser2base, rot_laser2base))
        rf_laser2base_matrix = TxyzRxyz_2_Pose(rf_laser2base_non_matrix)
        print_reference("rf_laser2base_non_matrix", rf_laser2base_non_matrix)

        # Set the robot frame and tool pose
        pos_setFrame = [0, 0, 0]  # Translation vector [Tx, Ty, Tz]
        rot_setFrame = [np.radians(0), np.radians(0), np.radians(0)]  # Rotation angles [Rx, Ry, Rz] in radians
        rf_setFrame = np.concatenate((pos_setFrame, rot_setFrame))
        setFrame = TxyzRxyz_2_Pose(rf_setFrame)

        self.robot.setPoseFrame(setFrame)
        self.robot.setPoseTool(rf_laser2flange_matrix)

        return rf_laser2camera, rf_laser2base, rf_laser2base_matrix

    def createPoint(self, arr_target, count, theta_laser, rf_laser2camera, rf_laser2base):

        target2Camera = intialTarget(arr_target[0], arr_target[1], arr_target[2])

        rf_target2laser = np.dot(np.linalg.inv(rf_laser2camera), target2Camera)

        rf_target2base_bf = np.dot(rf_laser2base, rf_target2laser)
        print("target_ref_base:", rf_target2base_bf, "\n")

        rot_Laser = rotz(np.radians(theta_laser))

        rf_target2base_af = np.dot(rf_target2base_bf, rot_Laser)
        # print(f'rf_laser2base_af:{rf_target2base_af}')

        pos, rot = self.robot_module.rotPos(rf_target2base_af)
        target2base_none_mat = np.concatenate((pos, rot), axis=0)
        # print(f'pos, rot:{pos}, {rot}')

        target2base_mat = TxyzRxyz_2_Pose(target2base_none_mat)
        # print(f"target_laser_mat:{target2base_mat}")

        return target2base_mat, pos

