

from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import sys

sys.path.append("E:\Quan\AutoRoboticInspection-v1\VIKO_UltraRobot") # config path
import numpy as np
import cv2
from src import robotic_modify as ROB
from library import robot_lib_modify as RL
from config import config as CFG
from open_camera import onCameraGrabbed



def get_point(img):
    img = cv2.imread(img)
    x, y = 0, 0
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img_gray, 220, 255, cv2.THRESH_BINARY_INV)

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
    # cv2.namedWindow('gray', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('gray', 300, 700)
    # #cv2.resize(image_copy, (480, 640))
    # cv2.imshow("gray", image_copy)
    # cv2.waitKey(0)  # Wait indefinitely until a key is pressed
    # cv2.destroyAllWindows()  # Close all OpenCV windows
    return [x, y]


def main():
    robot = Robot()
    robot.cameraPosLeft()
    path_img1 = onCameraGrabbed()

    robot.cameraPosRight()
    path_img2 = onCameraGrabbed()
    robot.rob_mod.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
    p1 = get_point(path_img1)
    p2 = get_point(path_img2)

    print(p1, p2)
    

    

class Robot:
    def __init__(self):
        self.rob_mod = ROB.VisionRobot()
        self.rob_lib = RL.RobotModule()
        self.rob_mod.connectRobot()
        self.laser2rf = self.rob_mod.fixedRef()
        self.rob_mod.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])


    def cameraPosLeft(self):
        # move the left position to capture
        org_camera = self.laser2rf
        pos, rot = self.rob_lib.rotPos(org_camera)
        rot = [rot[0], rot[1], rot[2]]
        pos = [pos[0], pos[1] + CFG.HORIZONTAL_BASELINE, pos[2]]
        camera2base_left_nonmat = np.concatenate((pos, rot))
        camera2base_left = TxyzRxyz_2_Pose(camera2base_left_nonmat)

        print(f'camera2base_left:{camera2base_left}')
        self.rob_mod.robot.MoveJ(camera2base_left)

    def cameraPosRight(self):
        # move the right position to capture
        org_camera = self.laser2rf
        pos, rot = self.rob_lib.rotPos(org_camera)
        rot = [rot[0], rot[1], rot[2]]
        pos = [pos[0], pos[1] - CFG.HORIZONTAL_BASELINE, pos[2]]
        camera2base_right_nonmat = np.concatenate((pos, rot))
        camera2base_right = TxyzRxyz_2_Pose(camera2base_right_nonmat)

        print(f'camera2base_right:{camera2base_right}')
        self.rob_mod.robot.MoveJ(camera2base_right)

    def distanceCameraToObject(x1_dis, x2_dis, focal_length, baseLine):
        pixel_focalLength = focal_length * 1000 / CFG.PIXEL_SIZE
        dis_cameraToObject = baseLine * pixel_focalLength / (abs(x1_dis - x2_dis))
        return dis_cameraToObject, pixel_focalLength
    
    
if __name__ == "__main__":
  main()
