import numpy as np
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import sys
import math

sys.path.append("D:\Quan\\roboDK\Vision-Machine-collab-Nhan\VIKO_UltraRobot")
from config import config as CFG
from library import robot_lib as rl
RDK = Robolink()
robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)
import cv2


def cameraPosLeft(matcamera2base):
    
    ## move the left position to capture
    camera2base_posLeft = matcamera2base
    rot, pos = rl.rotPos(camera2base_posLeft)
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
    rot, pos = rl.rotPos(camera2base_posRight)
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

def get_point(img):
    x, y = 0, 0
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow('gray', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('gray', 300, 700)

    cv2.imshow('gray', img_gray)
    # apply binary thresholding
    ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY_INV)

    # cv2.imshow('gray', thresh)

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, 
                                        method=cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        # draw contours on the original image
        image_copy = img.copy()
        area = cv2.contourArea(contour)
        print(area)
        if 10 < area < 10000:
            cv2.drawContours(image_copy, contours=contour, contourIdx=-1, 
                            color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = float(M['m10'] / M['m00'])
                cy = float(M['m01'] / M['m00'])
                x, y, = cx, cy
                print(f"Coordinates of the center: ({cx}, {cy})")
                
            else:
                print("Could not find the center coordinates.")

    image_copy = cv2.putText(image_copy, ".", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), thickness = 2, lineType=cv2.LINE_AA)
    # cv2.resize(image_copy, (480, 640))
    # cv2.imshow("Image with Contours", image_copy)
    cv2.waitKey(0)  # Wait indefinitely until a key is pressed
    cv2.destroyAllWindows()  # Close all OpenCV windows
    return x, y

image = cv2.imread("D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot\\test1.png")
x, y = get_point(image)



# dis_pixel:(269, 733, 271, 202)
# small 2
# (2050.949134397749, 924.6976815317794)
# (1930.690973910225, 1437.2208718198021)

# Big 
# (2050.03096639686, 1439.937362030905)
# (1931.5211436809227, 922.0008008970045)

# pixel_x1 = 751
# pixel_y1 = 216
# pixel_x2 = 753.97
# pixel_y2 = 238.02

# pixel_focalLength = 16 * 1000 / CFG.PIXEL_SIZE 
# o1o2 = 65
# res_width = 2048
# res_height = 2448
# focal_length = 16
# x1, y1 = convertCoordinates(res_width, res_height, pixel_x1, pixel_y1, pixel_focalLength, 558.45, -90)
# x2, y2 = convertCoordinates(res_width, res_height, pixel_x2, pixel_y2, pixel_focalLength, 563.45, -90)
# print (x1,y1, x2,y2)
# dis = distanceCameraToObject(x1, x2, focal_length)
# print(f'target1:{target1}, target2:{target2}')