from datetime import datetime
import pandas as pd
import numpy as np
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import sys


sys.path.append("E:\Quan\AutoRoboticInspection-V1\VIKO_UltraRobot") # config path
from config import config as CFG


class RobotModule:
    def __init__(self):
        self.RDK = Robolink()
        self.robot = self.RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)

    @staticmethod
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

    @staticmethod
    def rotPosRef(x, y, z, roll, pitch, yaw)->list:
        "The rotation degrees is in degree"
        rotRef = np.array([np.radians(roll), np.radians(pitch), np.radians(yaw)])
        posRef = np.array([x, y, z])
        return posRef, rotRef

    @staticmethod
    def createRef(translation, rotation):
        "The rotation degrees is in radian"
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

    @staticmethod
    def rotLaser(coordinate_pixel):
        pixel_1 = coordinate_pixel[0]
        pixel_2 = coordinate_pixel[1]

        angle = math.atan2(pixel_1[1] - pixel_2[1], pixel_1[0] - pixel_2[0])
        theta_laser = math.degrees(angle)
        print(f"theta_laser_before:{theta_laser}")
        theta_laser = 90 + theta_laser    ### theta_laser always negative      

        return theta_laser

    @staticmethod
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

    def convertCoordinates(
        self,
        res_width,
        res_height,
        pixel_x,
        pixel_y,
        pixel_focalLength,
        dis_cameraToObject,
        theta,
    ):
        "theta is in degrees"
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

    @staticmethod
    def distanceCameraToObject(x1_dis, x2_dis, focal_length, baseLine):
        pixel_focalLength = focal_length * 1000 / CFG.PIXEL_SIZE
        dis_cameraToObject = baseLine * pixel_focalLength / (abs(x1_dis - x2_dis))
        return dis_cameraToObject, pixel_focalLength

    @staticmethod
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



def rotx(rx: float) -> 'Mat':
    r"""Returns a rotation matrix around the X axis (radians)
    """
    ct = math.cos(np.radians(rx))
    st = math.sin(np.radians(rx))
    return Mat([
        [1, 0, 0, 0],
        [0, ct, -st, 0],
        [0, st, ct, 0],
        [0, 0, 0, 1],
    ])


def roty(ry: float) -> 'Mat':
    r"""Returns a rotation matrix around the Y axis (radians)
    """
    ct = math.cos(np.radians(ry))
    st = math.sin(np.radians(ry))
    return Mat([
        [ct, 0, st, 0],
        [0, 1, 0, 0],
        [-st, 0, ct, 0],
        [0, 0, 0, 1],
    ])


def rotz(rz: float) -> 'Mat':
    r"""Returns a rotation matrix around the Z axis (radians)
    """
    ct = math.cos(np.radians(rz))
    st = math.sin(np.radians(rz))
    return Mat([
        [ct, -st, 0, 0],
        [st, ct, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])