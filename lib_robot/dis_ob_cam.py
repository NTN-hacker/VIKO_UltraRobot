import numpy as np
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import lib_robot.rot_pos as rp
import sys

sys.path.append("D:\Quan\\roboDK\Vision-Machine-collab-Nhan\VIKO_UltraRobot")
from config import config as CFG
RDK = Robolink()
robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)

# def cameraForward(matcamera2base):
    
#     ## move the left position to capture
#     camera2base = matcamera2base
#     rot, pos = rp.rotPos(camera2base)
#     print(f'rot, pos of left:{rot}, {pos}')
#     pos = [pos[0] + CFG.FORWARD, pos[1], pos[2]]
#     print(f'newpos:{pos}')

#     camera2base_Forward_nonmat = np.concatenate((pos, rot))
#     camera2base_Forward  = TxyzRxyz_2_Pose(camera2base_Forward_nonmat)
#     robot.MoveJ(camera2base_Forward)
#     print(f'camera2base_posLeft_mat:{camera2base_Forward_nonmat}')


def cameraPosLeft(matcamera2base):
    
    ## move the left position to capture
    camera2base_posLeft = matcamera2base
    rot, pos = rp.rotPos(camera2base_posLeft)

    rot = [rot[0], rot[1], rot[2]]
    pos = [pos[0], pos[1] + (CFG.HORIZONTAL_BASELINE)/2, pos[2]]

    camera2base_left_nonmat = np.concatenate((pos, rot))
    camera2base_left = TxyzRxyz_2_Pose(camera2base_left_nonmat)
    robot.MoveJ(camera2base_left)


def cameraPosRight(matcamera2base):
    
    ## move the right position to capture
    camera2base_posRight = matcamera2base
    rot, pos = rp.rotPos(camera2base_posRight)

    rot = [rot[0], rot[1], rot[2]]
    pos = [pos[0], pos[1] - (CFG.HORIZONTAL_BASELINE)/2, pos[2]]

    camera2base_right_nonmat = np.concatenate((pos, rot))
    camera2base_right = TxyzRxyz_2_Pose(camera2base_right_nonmat)
    robot.MoveJ(camera2base_right)


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


# dis_pixel:(269, 733, 271, 202)
# small 2
# (2050.949134397749, 924.6976815317794)
# (1930.690973910225, 1437.2208718198021)

# Big 
# (2050.03096639686, 1439.937362030905)
# (1931.5211436809227, 922.0008008970045)

# pixel_x1 = 2215
# pixel_y1 = 2209
# pixel_x2 = 1702
# pixel_y2 = 2209


# o1o2 = 60
# res_width = 2048
# res_height = 2448
# focal_length = 16
# x1, y1 = convertCoordinates(res_width, res_height, pixel_x1, pixel_y1, 3.45)
# x2, y2 = convertCoordinates(res_width, res_height, pixel_x2, pixel_y2, 3.45)

# dis = distanceCameraToObject(x1, x2, focal_length)
# print(f'target1:{target1}, target2:{target2}')