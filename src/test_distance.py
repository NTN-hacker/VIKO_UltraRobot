

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


def main():

    robot = Robot()
    robot.cameraPosLeft()
    path_img1 = onCameraGrabbed()

    robot.cameraPosRight()
    path_img2 = onCameraGrabbed()
    robot.rob_mod.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

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
        self.rob_mod.robot.MoveL(camera2base_left)

    def cameraPosRight(self):
        # move the right position to capture
        org_camera = self.laser2rf
        pos, rot = self.rob_lib.rotPos(org_camera)
        rot = [rot[0], rot[1], rot[2]]
        pos = [pos[0], pos[1] - CFG.HORIZONTAL_BASELINE, pos[2]]
        camera2base_right_nonmat = np.concatenate((pos, rot))
        camera2base_right = TxyzRxyz_2_Pose(camera2base_right_nonmat)

        print(f'camera2base_right:{camera2base_right}')
        self.rob_mod.robot.MoveL(camera2base_right)
    
    
if __name__ == "__main__":
  main()
