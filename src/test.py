import numpy as np
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import sys
import math

sys.path.append("E:\Project\Robot-6DOF\VIKO_UltraRobot")
from config import config as CFG
from library import robot_lib as rl

# RDK = Robolink()
# robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)
import cv2


def cameraPosLeft(matcamera2base):

    ## move the left position to capture
    camera2base_posLeft = matcamera2base
    rot, pos = rl.rotPos(camera2base_posLeft)
    print(f"rot, pos of left:{rot}, {pos}")
    rot = [rot[0], rot[1], rot[2]]
    print(f"rot:{rot}")
    pos = [pos[0], pos[1] + (CFG.HORIZONTAL_BASELINE) / 2, pos[2]]
    # pos = [pos[0] - CFG.FORWARD_BASELINE/2, pos[1], pos[2]]
    print(f"newpos:{pos}")

    camera2base_left_nonmat = np.concatenate((pos, rot))
    camera2base_left = TxyzRxyz_2_Pose(camera2base_left_nonmat)
    robot.MoveJ(camera2base_left)
    print(f"camera2base_posLeft_mat:{camera2base_left}")


def cameraPosRight(matcamera2base):

    ## move the right position to capture
    camera2base_posRight = matcamera2base
    rot, pos = rl.rotPos(camera2base_posRight)
    # pos = [pos[0] + CFG.FORWARD_BASELINE/2, pos[1], pos[2]]
    rot = [rot[0], rot[1], rot[2]]
    print(f"rot:{rot}")
    pos = [pos[0], pos[1] - (CFG.HORIZONTAL_BASELINE) / 2, pos[2]]
    print(f"newpos:{pos}")

    camera2base_right_nonmat = np.concatenate((pos, rot))
    camera2base_right = TxyzRxyz_2_Pose(camera2base_right_nonmat)
    robot.MoveJ(camera2base_right)
    print(f"camera2base_posLeft_mat:{camera2base_right}")


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

    print("xtocamera:", x1_real, "\n")
    print("ytocamera:", y1_real, "\n")

    return x1_real, y1_real


def distanceCameraToObject(x1_dis, x2_dis, focal_length, baseLine):

    pixel_focalLength = focal_length * 1000 / CFG.PIXEL_SIZE
    dis_cameraToObject = baseLine * pixel_focalLength / (abs(x1_dis - x2_dis))
    print(f"dis_cameraToObject:{dis_cameraToObject}")
    return dis_cameraToObject, pixel_focalLength


def get_point(img):
    x, y = 0, 0
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.namedWindow('gray', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('gray', 300, 700)

    # cv2.imshow('gray', img_gray)
    # apply binary thresholding
    ret, thresh = cv2.threshold(img_gray, 220, 255, cv2.THRESH_BINARY_INV)

    # cv2.imshow('gray', thresh)

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    contours, hierarchy = cv2.findContours(
        image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE
    )
    for contour in contours:
        # draw contours on the original image
        image_copy = img.copy()
        area = cv2.contourArea(contour)
        print(area)
        if 10 < area < 10000:
            cv2.drawContours(
                image_copy,
                contours=contour,
                contourIdx=-1,
                color=(0, 255, 0),
                thickness=2,
                lineType=cv2.LINE_AA,
            )
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = float(M["m10"] / M["m00"])
                cy = float(M["m01"] / M["m00"])
                (
                    x,
                    y,
                ) = (
                    cx,
                    cy,
                )
                print(f"Coordinates of the center: ({cx}, {cy})")

            else:
                print("Could not find the center coordinates.")

    image_copy = cv2.putText(
        image_copy,
        ".",
        (int(x), int(y)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 255),
        thickness=2,
        lineType=cv2.LINE_AA,
    )
    cv2.namedWindow("gray", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("gray", 300, 700)
    # cv2.resize(image_copy, (480, 640))
    cv2.imshow("gray", image_copy)
    cv2.waitKey(0)  # Wait indefinitely until a key is pressed
    cv2.destroyAllWindows()  # Close all OpenCV windows
    return x, y


def FoV(dis_cameraToObject):
    horizontal_FoV = (dis_cameraToObject * CFG.PIXEL_SIZE * CFG.RESOLUTION_X) / (
        CFG.FOCAL_LENGTH * 1000
    )
    vertical_FoV = (dis_cameraToObject * CFG.PIXEL_SIZE * CFG.RESOLUTION_Y) / (
        CFG.FOCAL_LENGTH * 1000
    )

    return [horizontal_FoV, vertical_FoV]


# image = cv2.imread("D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot\\test2.png")
# x, y = get_point(image)


# dis_pixel:(269, 733, 271, 202)
# small 2
# (2050.949134397749, 924.6976815317794)
# (1930.690973910225, 1437.2208718198021)

# Big
# (2050.03096639686, 1439.937362030905)
# (1931.5211436809227, 922.0008008970045)

p1 = [858.5115955011445, 1202.8767880043406]
p2 = [331.5910552470038, 673.9383554899064]

pixel_x1 = p1[1]
pixel_y1 = p1[0]
pixel_x2 = p2[1]
pixel_y2 = p2[0]

focal_length = 16
# x1, y1 = convertCoordinates(res_width, res_height, pixel_x1, pixel_y1, pixel_focalLength, 558.45, -90)
# x2, y2 = convertCoordinates(res_width, res_height, pixel_x2, pixel_y2, pixel_focalLength, 563.45, -90)
# print (x1,y1, x2,y2)
# dis = distanceCameraToObject(pixel_y1, pixel_y2, focal_length, 65)
# print(dis)
# print(f'target1:{target1}, target2:{target2}')

# A = [1105, 614]
# B = [1908, 713]

# theta1 = np.arctan2(A[1] - B[1], A[0] - B[0])
# theta2 = np.arctan2(B[1] - A[1], B[0] - A[0])
# print(f"theta1: {np.degrees(theta1)}, theta2: {np.degrees(theta2)}")

fov_600 = FoV(600)
print(f"fov_600:{fov_600} \n")


def createPoint(self, target_01, target_02, theta_laser, backOx, alpha):

    orgTarget01ToCamera = self.robot_module.intialTarget(
        target_01[0], target_01[1], target_01[2]
    )
    orgTarget012oCamera = self.robot_module.intialTarget(
        target_02[0], target_02[1], target_02[2]
    )

    orgTarget01ToLaser = np.dot(
        np.linalg.inv(self.rf_laser2camera), orgTarget01ToCamera
    )
    orgTarget02ToLaser = np.dot(
        np.linalg.inv(self.rf_laser2camera), orgTarget012oCamera
    )

    targetToOrgTarget = rotz(theta_laser)

    target01toLaser = np.dot(orgTarget01ToLaser, targetToOrgTarget)
    target02toLaser = np.dot(orgTarget02ToLaser, targetToOrgTarget)

    target01ToRf = np.dot(self.rf_laser2rf, target01toLaser)
    target02ToRf = np.dot(self.rf_laser2rf, target02toLaser)

    pos1toRf, rot1toRf = self.robot_module.rotPos(target01ToRf)
    pos2toRf, rot2toRf = self.robot_module.rotPos(target02ToRf)
    pos2ToPos1_rf = pos2toRf - pos1toRf
    pos_target02_oy_rf = sqrt(pos2ToPos1[0] ** 2 + pos2ToPos1[1] ** 2)
    print(f'pos_target02_oy_rf:{pos_target02_oy_rf}')

    rfToBase = self.robot_module.createRef([0, 0, 0], [np.radians(180), 0, 0])
    target01ToBase = np.dot(rfToBase, target01ToRf)
    target02ToBase = np.dot(rfToBase, target02ToRf)

    pos1toBase, _ = self.robot_module.rotPos(target01ToBase)
    pos2toBase, _ = self.robot_module.rotPos(target02ToBase)
    pos2ToPos1 = pos2toBase - pos1toBase
    pos_target02_oy = sqrt(pos2ToPos1[0] ** 2 + pos2ToPos1[1] ** 2)
    print(f'pos_target02_oy:{pos_target02_oy}')

    if theta_laser > 0:
        newPos2 = np.array([0, pos_target02_oy, 0])
    elif theta_laser <= 0:
        newPos2 = np.array([0, -pos_target02_oy, 0])
    else:
        print(f"check the angle of laser:{theta_laser}")

    newRef = target01ToBase
    posRef, rotRef = self.robot_module.rotPos(newRef)
    newRef_nonMat = np.concatenate((posRef, rotRef), axis=0)
    setRef = TxyzRxyz_2_Pose(newRef_nonMat)
    self.robot_module.setPoseFrame(setRef)

    target01toRf = np.dot(target01ToBase, np.linalg.inv(target01ToBase))
    target02toRf = self.robot_module.createRef(newPos2, rot2toRf)

    if alpha != 0:
        newT1toRf = np.dot(np.dot(target01toRf, roty(np.radians(alpha))), rotx(np.radians(0)))
        newT2toRf = np.dot(np.dot(target02toRf, roty(np.radians(alpha))), rotx(np.radians(0)))
    else:
        newT1toRf = np.dot(np.dot(target01toRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx
        newT2toRf = np.dot(np.dot(target02toRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx

    Pos1, Rot1 = self.robot_module.rotPos(newT1toRf)
    Pos2, Rot2 = self.robot_module.rotPos(newT2toRf)

    Pos1[0] += backOx
    Pos2[0] += backOx
    if alpha != 0:
        Pos1[1] += 0
        Pos2[1] += 0
    else:
        # pass
        print(f"Pos1 and Pos2 changed:{Pos1}, {Pos2}")

    new01 = self.robot_module.createRef(Pos1, Rot1)
    new01toBase = np.dot(newRef, new01)
    newPos01toBase, newRot01toBase = self.robot_module.rotPos(new01toBase)
    print(f"newPos01toBase:{newPos01toBase}, \n newRot01toBase:{newRot01toBase}")

    new02 = self.robot_module.createRef(Pos2, Rot2)
    new02toBase = np.dot(newRef, new02)
    newPos02toBase, newRot02toBase = self.robot_module.rotPos(new02toBase)
    print(f"newPos02toBase:{newPos02toBase}, \n newRot02toBase:{newRot02toBase}")

    target01_none_mat = np.concatenate((Pos1, Rot1), axis=0)
    target02_none_mat = np.concatenate((Pos2, Rot2), axis=0)
    target01 = TxyzRxyz_2_Pose(target01_none_mat)
    target02 = TxyzRxyz_2_Pose(target02_none_mat)

    return target01, target02, pos_target02_oy
