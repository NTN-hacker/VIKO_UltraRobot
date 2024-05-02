from datetime import datetime 
import pandas as pd
import numpy as np
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import sys
import os.path as osp
import os

sys.path.append("D:\Quan\\roboDK\Vision-Machine-collab-Nhan\VIKO_UltraRobot")
from config import config as CFG
RDK = Robolink()
robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)


def rotPos(trans_matrix):
    """
    .....
    """
    roll = np.arctan2(trans_matrix[2, 1], trans_matrix[2, 2])
    pitch = np.arctan2(
        -trans_matrix[2, 0], np.sqrt(trans_matrix[2, 1] ** 2 + trans_matrix[2, 2] ** 2)
    )
    yaw = np.arctan2(trans_matrix[1, 0], trans_matrix[0, 0])

    rot = np.array([roll, pitch, yaw])
    pos = trans_matrix[:3, 3]

    return rot, pos


def createRef(translation, rotation):
    """
    Create a transformation matrix for a new reference frame.

    Args:
    - translation: A 3-element list or array representing the translation [Tx, Ty, Tz].
    - rotation: A 3-element list or array representing the rotation angles [Rx, Ry, Rz] in radians.

    Returns:
    - transformation_matrix: A 4x4 transformation matrix representing the new reference frame.
    """
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




def export_csv(data: dict):
    """
    Export to excel or csv
    """
    time = str(datetime.now())
    filename = f"data/Robotic/Test_{time[:10]}.csv"

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





def cameraPosLeft(matcamera2base):
    
    ## move the left position to capture
    camera2base_posLeft = matcamera2base
    rot, pos = rotPos(camera2base_posLeft)
    print(f'rot, pos of left:{rot}, {pos}')
    rot = [rot[0], rot[1], rot[2]]
    print(f'rot:{rot}')
    pos = [pos[0], pos[1] + (CFG.HORIZONTAL_BASELINE)/2, pos[2]]
    # pos = [pos[0] - CFG.FORWARD_BASELINE/2, pos[1], pos[2]]
    print(f'newpos:{pos}')

    camera2base_left_nonmat = np.concatenate((pos, rot))
    camera2base_left = TxyzRxyz_2_Pose(camera2base_left_nonmat)
    robot.MoveJ(camera2base_left)
    print(f'camera2base_posLeft_mat:{camera2base_left}')


def cameraPosRight(matcamera2base):
    
    ## move the right position to capture
    camera2base_posRight = matcamera2base
    rot, pos = rotPos(camera2base_posRight)
    # pos = [pos[0] + CFG.FORWARD_BASELINE/2, pos[1], pos[2]]
    rot = [rot[0], rot[1], rot[2]]
    print(f'rot:{rot}')
    pos = [pos[0], pos[1] - (CFG.HORIZONTAL_BASELINE)/2, pos[2]]
    print(f'newpos:{pos}')

    camera2base_right_nonmat = np.concatenate((pos, rot))
    camera2base_right = TxyzRxyz_2_Pose(camera2base_right_nonmat)
    robot.MoveJ(camera2base_right)
    print(f'camera2base_posLeft_mat:{camera2base_right}')


def convertCoordinates(res_width, res_height, pixel_x, pixel_y, pixel_focalLength, dis_cameraToObject, theta):                    

    xpixel_to_center = pixel_x - res_width / 2
    ypixel_to_center = pixel_y - res_height / 2

    pixel_XY = np.array([[xpixel_to_center], [ypixel_to_center], [1]])

    rot_trans_XY = np.array([[np.cos(np.radians(theta)), -np.sin(np.radians(theta)), 0],\
                             [np.sin(np.radians(theta)), np.cos(np.radians(theta)), 0],\
                               [0, 0, 1]] )

    new_XY = np.dot(rot_trans_XY, pixel_XY)

    x1_real = new_XY[0][0]* dis_cameraToObject / pixel_focalLength
    y1_real = new_XY[1][0]* dis_cameraToObject / pixel_focalLength

    print("xtocamera:", x1_real, "\n")
    print("ytocamera:", y1_real, "\n")

    return x1_real, y1_real


def distanceCameraToObject(x1_dis, x2_dis, focal_length, baseLine):
    
    pixel_focalLength = focal_length * 1000 / CFG.PIXEL_SIZE 
    dis_cameraToObject = baseLine * pixel_focalLength / (abs(x1_dis - x2_dis))
    print(f'dis_cameraToObject:{dis_cameraToObject}')
    return dis_cameraToObject, pixel_focalLength




##### POINT ####
# import numpy as np
# from robotic import x_target, x_target_02, y_target, y_target_02, createPoint
# from config import config as CFG
# def trajectory(x, y):
#     ## create the y = ax + b
#     a, b = np.polyfit(x, y, 1)
#     return a, b

# x = [x_target, x_target_02]
# y = [y_target, y_target_02]
# a, b = trajectory(x, y)
# print(f'a:{a}, b:{b}')

# def points_tra(a, b, x_target, x_target_02) -> float:
#     i = 0
#     step = int(abs(x_target_02 - x_target)/10)
#     new_x = np.zeros(10)
#     new_y = np.zeros(10)
#     for i in range(10):
#         if i == 0:
#             new_x[i] = x_target + step
#             new_y[i] = a * new_x[i] + b
#         else:
#             new_x[i] = new_x[i-1] + step
#             new_y[i] = a * new_x[i] + b
        
#         target, joint = createPoint(new_x[i], new_y[i], CFG.DISTANCE_2OBJECT, i)
        
#         if i == 8:
#             print(f'joint:{joint}')

#     print(f'newx:{new_x}, newy:{new_y}')
    

#     return new_x, new_y

# new_x, new_y = points_tra(a, b, x_target, x_target_02)
#####

