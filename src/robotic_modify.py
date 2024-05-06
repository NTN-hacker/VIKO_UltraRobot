import sys

# sys.path.append(
#     "D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot"
# )  # config path
sys.path.append(
    "E:\\Project\\Robot-6DOF\\VIKO_UltraRobot"
)  # config path
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import numpy as np
import time
from layout import app_robot as app

from library import vision as vis
from library import robot_lib as rob
from datetime import datetime
from config import config as CFG


## class for vision robot
class VisionRobot:
    def __init__(self, RDK=None, robot=None):
        if RDK is None:
            RDK = Robolink()
        if robot is None:
            self.robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)
        if not robot.Valid():
            raise Exception("Invalid robot selected")

        self.RDK = RDK

        if RDK.RunMode() != RUNMODE_SIMULATE:
            RUN_ON_ROBOT = False

        if RUN_ON_ROBOT:
            # Connect to the robot using default IP
            robot.Connect()  # Try to connect once
            robot.ConnectSafe()  # Try to connect multiple times
            status, status_msg = robot.ConnectedState()
            print(status)
            print(status_msg)
            if status != ROBOTCOM_READY:
                # Stop if the connection did not succeed
                print(status_msg)
                raise Exception("Failed to connect: " + status_msg)

            # This will set to run the API programs on the robot and the simulator (online programming)
            RDK.setRunMode(RUNMODE_RUN_ROBOT)

        robot.ConnectedState()
        self.robot_module = rob.RobotModule()
        self.vision = vis.VisionModule()
        print("ConnectedState:", robot.ConnectedState(), "\n")

    def fixedRef(self):

        # reference frame flange to base
        pos_flange2base, rot_flange2base = self.robot_module.rotPosRef(
            380, 0, 305, -180, 0, 0
        )
        rf_flage2base = self.robot_module.createRef(pos_flange2base, rot_flange2base)
        print("rf_flage2base:", rf_flage2base, "\n")

        # reference frame camera to flange
        pos_camera2flange, rot_camera2flange = self.robot_module.rotPosRef(
            0, -30, 200, 0, 0, 0
        )
        rf_camera2flange = self.robot_module.createRef(
            pos_camera2flange, rot_camera2flange
        )
        print("ref_camera2flange:", rf_camera2flange, "\n")

        # reference frame camera to base
        rf_camera2base = np.dot(rf_flage2base, rf_camera2flange)
        print("ref_camera2base:", rf_camera2base, "\n")

        pos_camera2base, rot_camera2base = self.robot_module.rotPos(rf_camera2base)
        # print("rot_camera2base:", rot_camera2base, "\n", "pos_camera2base:", pos_camera2base, "\n")

        rf_camera2base_non_matrix = np.concatenate((pos_camera2base, rot_camera2base))
        print("rot_camera2base_non_matrix:", rf_camera2base_non_matrix, "\n")
        rf_camera2base_matrix = TxyzRxyz_2_Pose(rf_camera2base_non_matrix)

        rf_camera2flange_non_matrix = np.concatenate(
            (pos_camera2flange, rot_camera2flange)
        )

        rf_camera2flange_matrix = TxyzRxyz_2_Pose(rf_camera2flange_non_matrix)
        print("ref_camera2flange_test:", rf_camera2flange_matrix, "\n")

        self.robot.setPoseFrame(self.robot.PoseFrame())
        # print(f"robot.PoseFrame():{robot.PoseFrame()}")
        self.robot.setPoseTool(rf_camera2flange_matrix)

        return rf_camera2base_matrix, rf_camera2base

    def getCoordinates(self, flag: bool):

        self.vision._start_()
        self.vision._getImage_()
        self.vision.load_model()
        self.vision.save_image()
        coordinate = self.vision._getCoordinate_()
        # coordinate = self.vision._getCircle_()
        if flag == True:
            coordinate = self.vision._getCoordinateWeld_(
                self.vision.img_split, coordinate
            )
        self.vision._end_()
        return coordinate

    @staticmethod
    def sleep_seconds(seconds):
        print(f"Sleeping for {seconds} seconds...")
        time.sleep(seconds)
        print("Awake now!")

    def createPoint(self, arr_target, count, theta_laser, rf_camera2base):

        target_to_Camera = self.robot_module.intialTarget(
            arr_target[0], arr_target[1], arr_target[2]
        )

        rf_target2base = np.dot(rf_camera2base, target_to_Camera)
        print("target_ref_base:", rf_target2base, "\n")

        pos_target, rot_target = self.robot_module.rotPos(rf_target2base)
        print("rot_target:", rot_target, "\n", "pos_target:", pos_target, "\n")

        pos_target = [pos_target[0] - 51.44, pos_target[1] - 57, pos_target[2]]
        rot_target = [
            rot_target[0],
            rot_target[1],
            rot_target[2] + np.radians(theta_laser),
        ]

        target_laser_none_mat = np.concatenate((pos_target, rot_target), axis=0)

        # convert to pose
        target_laser_mat = TxyzRxyz_2_Pose(target_laser_none_mat)
        print(f"target_laser_mat:{target_laser_mat}")

        return target_laser_mat, pos_target

    def disCameraToObject(self, rf_camera2base, rf_camera2base_matrix):

        self.robot_module.cameraPosLeft(rf_camera2base)
        coordinate_pixel_Left = self.getCoordinates(False)

        self.robot_module.cameraPosRight(rf_camera2base)
        coordinate_pixel_Right = self.getCoordinates(False)

        dis_pixel = [coordinate_pixel_Left, coordinate_pixel_Right]
        print(f"dis_pixel:{dis_pixel}")

        if (
            abs(dis_pixel[0][0] - dis_pixel[1][0]) > 10
            and abs(dis_pixel[0][1] - dis_pixel[1][1]) < 10
        ):

            dis_cameraToObject, pixel_focalLength = (
                self.robot_module.distanceCameraToObject(
                    dis_pixel[0][0],
                    dis_pixel[1][0],
                    CFG.FOCAL_LENGTH,
                    CFG.HORIZONTAL_BASELINE,
                )
            )  # distance from camera to object
            print(f"Distance:{dis_cameraToObject}")

        elif (
            abs(dis_pixel[0][0] - dis_pixel[1][0]) < 10
            and abs(dis_pixel[0][1] - dis_pixel[1][1]) > 10
        ):

            dis_cameraToObject, pixel_focalLength = (
                self.robot_module.distanceCameraToObject(
                    dis_pixel[0][1],
                    dis_pixel[1][1],
                    CFG.FOCAL_LENGTH,
                    CFG.FORWARD_BASELINE,
                )
            )  # distance from camera to object
            print(f"Distance:{dis_cameraToObject}")

        elif (
            abs(dis_pixel[0][0] - dis_pixel[1][0]) >= 10
            and abs(dis_pixel[0][1] - dis_pixel[1][1]) >= 10
        ) or (
            abs(dis_pixel[0][0] - dis_pixel[1][0]) >= 10
            and abs(dis_pixel[0][1] - dis_pixel[1][1]) >= 10
        ):
            print(f"check 2 pixels")
            # distance from camera to object
        else:
            print("out 5 pixel")

        self.robot.MoveJ(rf_camera2base_matrix)

        return dis_cameraToObject, pixel_focalLength, dis_pixel

    def getTarget(self, dis_cameraToObject, pixel_focalLength):

        coordinate_pixel = self.getCoordinates(True)
        theta_laser = self.robot_module.rotLaser(coordinate_pixel)
        coordinate_pixel_1 = coordinate_pixel[0]
        print(coordinate_pixel)

        x_to_camera_01, y_to_camera_01 = self.robot_module.convertCoordinates(
            CFG.RESOLUTION_X,
            CFG.RESOLUTION_Y,
            coordinate_pixel_1[0],
            coordinate_pixel_1[1],
            pixel_focalLength,
            dis_cameraToObject,
            theta=-90,
        )  # cfg

        coordinate_pixel_2 = coordinate_pixel[1]
        x_to_camera_02, y_to_camera_02 = self.robot_module.convertCoordinates(
            CFG.RESOLUTION_X,
            CFG.RESOLUTION_Y,
            coordinate_pixel_2[0],
            coordinate_pixel_2[1],
            pixel_focalLength,
            dis_cameraToObject,
            theta=-90,
        )  # cfg
        z_laser_to_camera = dis_cameraToObject - CFG.DISTANCE_LASERtoOBJECT
        real_target01 = np.array([x_to_camera_01, y_to_camera_01, z_laser_to_camera])
        real_target02 = np.array([x_to_camera_02, y_to_camera_02, z_laser_to_camera])

        target01, pos_laser01 = self.createPoint(real_target01, 1, theta_laser)  # fix
        target02, pos_laser02 = self.createPoint(real_target02, 2, theta_laser)
        print(f"target01:{target01}, target02:{target02}")

        return target01, target02, pos_laser01, pos_laser02

    def runRobot(self, target_laser_mat, pos_laser_to_object):

        speeds = CFG.SPEEDS  # cfg - TEST
        self.robot.setSpeed(speeds[1])
        # try:
        self.robot.MoveJ(target_laser_mat)
        get_joint = self.robot.Joints()
        print(f"get_joint:{get_joint}")

    def attRobot(self, rf_camera2base_matrix):

        self.robot.setRounding(5)  # Set the rounding parameter
        self.robot.setSpeed(10, 10)  # Set linear speed in mm/s
        self.robot.setSpeedJoints(10)
        self.robot.MoveJ(rf_camera2base_matrix)

    def getParam(self):
        """Get custom binary data from this item. Use setParam to set the data"""
        current_joint_values = self.robot.Joints()
        limit = self.robot.JointLimits()

        return current_joint_values, limit


def main():
    VisRob = VisionRobot()
    rf_camera2base_matrix, rf_camera2base = VisRob.fixedRef()
    VisRob.attRobot(rf_camera2base_matrix)
    dis_cameraToObject, pixel_focalLength, dis_pixel = VisRob.disCameraToObject(
        rf_camera2base, rf_camera2base_matrix
    )
    target01, target02, pos_laser01, pos_laser02 = VisRob.getTarget(
        dis_cameraToObject, pixel_focalLength
    )
    VisRob.sleep_seconds(3)
    VisRob.runRobot(target01, pos_laser01)
    VisRob.sleep_seconds(10)
    VisRob.runRobot(target02, pos_laser02)
    current_joint_values, limit = VisRob.getParam()
    data_export = {
        "id": str(datetime.now()),
        # "target01": target01,
        # "target02": target02,
        "limit_lower": limit[0],
        "limit_upper": limit[1],
        "x1_pixel": dis_pixel[0][0],
        "y1_pixel": dis_pixel[0][1],
        "x2_pixel": dis_pixel[1][0],
        "y2_pixel": dis_pixel[1][1],
        "distance": dis_cameraToObject,
        #### add if need
    }
    rob.RobotModule.export_csv(data_export)


if __name__ == "__main__":
    app.main()
