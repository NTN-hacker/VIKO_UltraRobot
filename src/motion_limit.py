import sys
import threading

# sys.path.append("E:\\Project\\Robot-6DOF\\VIKO_UltraRobot")
sys.path.append("D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot")
from layout import app_robot as app
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
from src import robotic_modify as RM
import numpy as np


class TestThread:

    def __init__(self):
        self.rob_mod = RM.VisionRobot()
        self.rob_mod.pickRobot()
        self.rob_mod.fixedRef()
        self.rob_mod.attRobot()

    def limitMotion(self):
        # Default limit space in the Cartesian space
        DEFAULT_XYZ_MIN = np.array([199, -480, -300])
        DEFAULT_XYZ_MAX = np.array([727, 480, 305])

        print(f'X, Y, Z in range: (199, 727), (-480, 480), (-300, 305)')
        x1 = float(input("Enter X Coordinate: "))
        y1 = float(input("Enter Y Coordinate: "))
        z1 = float(input("Enter Z Coordinate: "))
        XYZ_1 = np.array([x1, y1, z1])
        print(f"Target created: ({x1}, {y1}, {z1})")

        x2 = float(input("Enter X Coordinate: "))
        y2 = float(input("Enter Y Coordinate: "))
        z2 = float(input("Enter Z Coordinate: "))
        XYZ_2 = np.array([x2, y2, z2])
        print(f"Target created: ({x2}, {y2}, {z2})")

        rot = np.array([np.radians(-180), np.radians(0), np.radians(0)])

        target_1_within_range = all(DEFAULT_XYZ_MIN[i] <= XYZ_1[i] <= DEFAULT_XYZ_MAX[i] for i in range(3))
        target_2_within_range = all(DEFAULT_XYZ_MIN[i] <= XYZ_2[i] <= DEFAULT_XYZ_MAX[i] for i in range(3))

        if target_1_within_range:
            
            print("Target 1 is within the default range.")
            target2base_none_mat1 = np.concatenate((XYZ_1, rot), axis=0)
            target2base_mat1 = TxyzRxyz_2_Pose(target2base_none_mat1)
            print(f"target_laser_mat:{target2base_mat1}")
            self.rob_mod.runMoveJ(target2base_mat1, None)

        else:
            print("Target 1 is outside the default range.")

        if target_2_within_range:

            print("Target 2 is within the default range.")
            target2base_none_mat2 = np.concatenate((XYZ_2, rot), axis=0)
            target2base_mat2 = TxyzRxyz_2_Pose(target2base_none_mat2)
            print(f"target_laser_mat:{target2base_mat2}")
            self.rob_mod.runMoveL(target2base_mat2, None)

        else:
            print("Target 2 is outside the default range.")


if __name__ == "__main__":
    # app.main()
    test = TestThread()
    test.limitMotion()