import sys

sys.path.append(
    "D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot"
)  # config path
# sys.path.append(
#     "E:\\Project\\Robot-6DOF\\VIKO_UltraRobot"
# )  # config path
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
        # if RDK is None:
        self.RDK = Robolink()
    # if robot is None:
        self.robot = self.RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)
        self.robot_module = rob.RobotModule()
        # self.vision = vis.VisionModule()

    def pickRobot(self):
        if not self.robot.Valid():
            raise Exception("Invalid robot selected")
        
        RUN_ON_ROBOT = True
        if self.RDK.RunMode() != RUNMODE_SIMULATE:
            RUN_ON_ROBOT = False

        if RUN_ON_ROBOT:
            # Connect to the robot using default IP
            self.robot.Connect()  # Try to connect once
            self.robot.ConnectSafe()  # Try to connect multiple times
            status, status_msg = self.robot.ConnectedState()
            print(status)
            print(status_msg)
            if status != ROBOTCOM_READY:
                # Stop if the connection did not succeed
                print(status_msg)
                raise Exception("Failed to connect: " + status_msg)

            # This will set to run the API programs on the robot and the simulator (online programming)
            self.RDK.setRunMode(RUNMODE_RUN_ROBOT)

        self.robot.ConnectedState()
        print("ConnectedState:", self.robot.ConnectedState(), "\n")

    def fixedRef(self):

        # reference frame flange to base
        pos_flange2base, rot_flange2base = self.robot_module.rotPosRef(
            380, 0, 255, 180, 0, 0
        )
        rf_flage2base = self.robot_module.createRef(pos_flange2base, rot_flange2base)
        print("rf_flage2base:", rf_flage2base, "\n")

        # reference frame camera to flange
        pos_camera2flange, rot_camera2flange = self.robot_module.rotPosRef(
            0, 0, 190, 0, 0, 0
        )
        rf_camera2flange = self.robot_module.createRef(
            pos_camera2flange, rot_camera2flange
        )
        print("ref_camera2flange:", rf_camera2flange, "\n")

        # reference frame camera to base
        rf_camera2base = np.dot(rf_flage2base, rf_camera2flange)
        print("ref_camera2base:", rf_camera2base, "\n")

        pos_laser2camera, rot_laser2camera = self.robot_module.rotPosRef(54, 57, 0, 0, 0, 0)
        self.rf_laser2camera = self.robot_module.createRef(pos_laser2camera, rot_laser2camera )

        rf_laser2flange = np.dot(rf_camera2flange, self.rf_laser2camera)
        pos_laser2flange, rot_laser2flange = self.robot_module.rotPos(rf_laser2flange)

        self.rf_laser2base = np.dot(rf_camera2base, self.rf_laser2camera)

        pos_laser2base, rot_laser2base = self.robot_module.rotPos(self.rf_laser2base)
        # print("rot_camera2base:", rot_camera2base, "\n", "pos_camera2base:", pos_camera2base, "\n")

        rf_laser2base_non_matrix = np.concatenate((pos_laser2base, rot_laser2base))
        print("rf_laser2base_non_matrix:", rf_laser2base_non_matrix, "\n")
        self.rf_laser2base_matrix = TxyzRxyz_2_Pose(rf_laser2base_non_matrix)

        rf_laser2flange_non_matrix = np.concatenate(
            (pos_laser2flange, rot_laser2flange)
        )

        rf_laser2flange_matrix = TxyzRxyz_2_Pose(rf_laser2flange_non_matrix)
        print("ref_camera2flange_test:", rf_laser2flange_matrix, "\n")

        pos_setFrame = [0, 0, 0]  # Translation vector [Tx, Ty, Tz] ## sai so
        rot_setFramee = [
            np.radians(0),
            np.radians(0),
            np.radians(0),
        ]  # Rotation angles [Rx, Ry, Rz] in radians
        rf_setFrame = np.concatenate((pos_setFrame, rot_setFramee))
        setFrame = TxyzRxyz_2_Pose(rf_setFrame)
        
        self.robot.setPoseFrame(setFrame)
        # print(f"robot.PoseFrame():{robot.PoseFrame()}")
        self.robot.setPoseTool(rf_laser2flange_matrix)

        return self.rf_laser2base_matrix, self.rf_laser2base, self.rf_laser2camera

    def getCoordinates(self, flag: bool):
        self.vision = vis.VisionModule()
        self.vision._start_()
        self.vision._getImage_()
        self.vision.save_image()
        self.vision.load_model()        
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

    def createPoint(self, arr_target, count, theta_laser):

        target2Camera = self.robot_module.intialTarget(
            arr_target[0], arr_target[1], arr_target[2]
        )

        rf_target2laser = np.dot(np.linalg.inv(self.rf_laser2camera), target2Camera)

        rf_target2base_bf = np.dot(self.rf_laser2base, rf_target2laser)
        print("target_ref_base:", rf_target2base_bf, "\n")

        rot_Laser = rotz(np.radians(theta_laser))

        rf_target2base_af = np.dot(rf_target2base_bf, rot_Laser)
        print(f'rf_laser2base_af:{rf_target2base_af}')

        pos, rot  = self.robot_module.rotPos(rf_target2base_af)
        target2base_none_mat = np.concatenate((pos, rot), axis=0)
        print(f'pos, rot:{pos}, {rot}')

        target2base_mat = TxyzRxyz_2_Pose(target2base_none_mat)
        print(f"target_laser_mat:{target2base_mat}")

        return target2base_mat, pos

    def disCameraToObject(self):

        self.robot_module.cameraPosLeft(self.rf_laser2base)
        coordinate_pixel_Left = self.getCoordinates(False)

        self.robot_module.cameraPosRight(self.rf_laser2base)
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

        self.robot.MoveJ(self.rf_laser2base_matrix)
        self.dis_cameraToObject = dis_cameraToObject
        self.pixel_focalLength = pixel_focalLength

        return dis_cameraToObject, dis_pixel

    def getTarget(self):

        coordinate_pixel = self.getCoordinates(True)
        theta_laser = self.robot_module.rotLaser(coordinate_pixel)
        coordinate_pixel_1 = coordinate_pixel[0]
        coordinate_pixel_2 = coordinate_pixel[1]
        print(f'coordinate_pixel:{coordinate_pixel}')

        x_to_camera_01, y_to_camera_01 = self.robot_module.convertCoordinates(
            CFG.RESOLUTION_X,
            CFG.RESOLUTION_Y,
            coordinate_pixel_1[0],
            coordinate_pixel_1[1],
            self.pixel_focalLength,
            self.dis_cameraToObject,
            theta=-90,
        )  # cfg


        x_to_camera_02, y_to_camera_02 = self.robot_module.convertCoordinates(
            CFG.RESOLUTION_X,
            CFG.RESOLUTION_Y,
            coordinate_pixel_2[0],
            coordinate_pixel_2[1],
            self.pixel_focalLength,
            self.dis_cameraToObject,
            theta=-90,
        )  # cfg
        z_laser_to_camera = self.dis_cameraToObject - CFG.DISTANCE_LASERtoOBJECT
        real_target01 = np.array([x_to_camera_01, y_to_camera_01, z_laser_to_camera])
        real_target02 = np.array([x_to_camera_02, y_to_camera_02, z_laser_to_camera])

        target01, pos_laser01 = self.createPoint(real_target01, 1, theta_laser)  # fix
        target02, pos_laser02 = self.createPoint(real_target02, 2, theta_laser)
        print(f"target01:{target01}, target02:{target02}")

        return target01, target02, pos_laser01, pos_laser02

    def runMoveL(self, target_laser_mat, pos_laser_to_object):

        speeds = CFG.SPEEDS  # cfg - TEST
        self.robot.setSpeed(speeds[1])
        # try:
        self.robot.MoveL(target_laser_mat)
        get_joint = self.robot.Joints()
        # print(f"get_joint:{get_joint}")

    def runMoveJ(self, target_laser_mat, pos_laser_to_object):

        speeds = CFG.SPEEDS  # cfg - TEST
        self.robot.setSpeed(speeds[1])
        # try:
        self.robot.MoveJ(target_laser_mat)
        get_joint = self.robot.Joints()
        # print(f"get_joint:{get_joint}")

    def attRobot(self):

        self.robot.setRounding(5)  # Set the rounding parameter
        self.robot.setSpeed(10, 10)  # Set linear speed in mm/s
        self.robot.setSpeedJoints(10)
        print(f'rf_laser2base_matrix: {self.rf_laser2base_matrix}')
        self.robot.MoveJ(self.rf_laser2base_matrix)

    def getParam(self):
        """Get custom binary data from this item. Use setParam to set the data"""
        current_joint_values = self.robot.Joints()
        limit = self.robot.JointLimits()

        return current_joint_values, limit


def mainRob():
    VisRob = VisionRobot()
    VisRob.pickRobot()
    VisRob.fixedRef()
    VisRob.attRobot()

    dis_cameraToObject, dis_pixel = VisRob.disCameraToObject()
    target01, target02, pos_laser01, pos_laser02 = VisRob.getTarget()

    VisRob.sleep_seconds(3)
    VisRob.runMoveJ(target01, pos_laser01)
    VisRob.sleep_seconds(10)
    VisRob.runMoveL(target02, pos_laser02)
    VisRob.attRobot()

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
    print(f'data_export:{data_export}')
    rob.RobotModule.export_csv(data_export)


if __name__ == "__main__":
    # app.main()
    mainRob()
