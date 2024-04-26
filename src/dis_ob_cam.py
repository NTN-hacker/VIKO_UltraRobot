import numpy as np
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import rot_pos as rp


RDK = Robolink()
# robot = RDK.AddFile("E:\\Install-software\\RoboDK\\Library\\Motoman-GP8.robot")
robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)

def cameraPosLeft(matcamera2base):
    
    ## move the left position to capture
    camera2base_posLeft = matcamera2base.copy()
    camera2base_posLeft = camera2base_posLeft[1] + 30
    rot, pos = rp.rotPos(camera2base_posLeft)
    camera2base_left_nonmat = np.concatenate(pos, rot)
    camera2base_left = TxyzRxyz_2_Pose(camera2base_left_nonmat)
    robot.MoveJ(camera2base_left)
    print(f'camera2base_posLeft_mat:{camera2base_left}')

def cameraPosRight(matcamera2base):
    
    ## move the right position to capture
    camera2base_posLeft = matcamera2base.copy() 
    camera2base_posLeft = camera2base_posLeft[1] - 30
    rot, pos = rp.rotPos(camera2base_posLeft)
    camera2base_right_nonmat = np.concatenate(pos, rot)
    camera2base_right = TxyzRxyz_2_Pose(camera2base_right_nonmat)
    robot.MoveJ(camera2base_right)
    print(f'camera2base_posLeft_mat:{camera2base_right}')


def convertCoordinates(res_width, res_height, pixel_x, pixel_y):                    

    sensor_height = res_height * 3.45 / 1000 #cfg
    sensor_width = res_width * 3.45 / 1000 #cfg
    sensor_size = np.array([sensor_height, sensor_width])
    print("sensor_size:", sensor_size, "\n")

    xpixel_center = res_width / 2
    ypixel_center = res_height / 2
    xpixel_2center = pixel_x - xpixel_center
    ypixel_2center = pixel_y - ypixel_center
    # print("xpixel2center:", xpixel_2center, "\n")

    x_to_camera = xpixel_2center * sensor_size[0] / (2 * xpixel_center)
    y_to_camera = ypixel_2center * sensor_size[1] / (2 * ypixel_center)

    print("xtocamera:", x_to_camera, "\n")
    print("ytocamera:", y_to_camera, "\n")

    return x_to_camera, y_to_camera

def realTarget(x1, y1, x2, y2, o1o2, focal_length):
    
    dis_cameraToObject = o1o2 * focal_length / (abs(x1) + abs(x2))
    print("distance:", dis_cameraToObject, "\n")

    x1_real = x1 * dis_cameraToObject / focal_length
    y1_real = y1 * dis_cameraToObject / focal_length
    target1 = np.array([x1_real, y1_real, dis_cameraToObject])

    x2_real = x2 * dis_cameraToObject / focal_length
    y2_real = y2* dis_cameraToObject / focal_length
    target2 = np.array([x2_real, y2_real, dis_cameraToObject ])

    return target1, target2, dis_cameraToObject

# small 2
# (2050.949134397749, 924.6976815317794)
# (1930.690973910225, 1437.2208718198021)

# Big 
# (2050.03096639686, 1439.937362030905)
# (1931.5211436809227, 922.0008008970045)

# pixel_x1 = 924.6976815317794
# pixel_y1 = 2050.949134397749

# pixel_x2 = 1439.937362030905
# pixel_y2 = 2050.03096639686


# o1o2 = 60
# res_width = 2448
# res_height = 2048
# focal_length = 16
# x1, y1 = convertCoordinates(res_height, res_width, pixel_x1, pixel_y1)
# x2, y2 = convertCoordinates(res_height, res_width, pixel_x2, pixel_y2)

# target1, target2 = height(x1, y1, x2, y2, o1o2, focal_length)
# print(f'target1:{target1}, target2:{target2}')