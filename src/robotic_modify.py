import sys

sys.path.append("D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot")
from robodk.robolink import *
from robodk.robomath import *
import numpy as np
import time
from layout import app_robot as app

from library import vision_robotic as vis
from library import robot_lib_modify as rob
from datetime import datetime
from config import config as CFG


## class for vision robot
class VisionRobot:
    def __init__(self, RDK=None, robot=None):
        self.RDK = Robolink()
        self.robot = self.RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)
        self.robot_module = rob.RobotModule()
        self.connectRobot()
        # self.fixedRef()

    def getCoordinates(self, model, image, flag=True):
        self.vision = vis.VisionModule()
        self.vision.load_model(model)
        coordinate = self.vision._getCoordinateTest_(image)
        return coordinate

    def connectRobot(self)-> None:
        """
        Connect Robot and PC
        """
        if not self.robot.Valid():
            raise Exception("Invalid robot selected")

        RUN_ON_ROBOT = True
        if self.RDK.RunMode() != RUNMODE_SIMULATE:
            RUN_ON_ROBOT = False

        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        if RUN_ON_ROBOT:
            # Connect to the robot using default IP
            self.robot.Connect("192.168.10.111")  # Try to connect once
            self.robot.ConnectSafe("192.168.10.111")  # Try to connect multiple times
            self.status, status_msg = self.robot.ConnectedState()

            if self.status != ROBOTCOM_READY:
                # Stop if the connection did not succeed
                print(status_msg)
                raise Exception("Failed to connect: " + status_msg)

            # This will set to run the API programs on the robot and the simulator (online programming)
            self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
            # self.RDK.CloseRoboDK()
        
        print("ConnectedState:", self.robot.ConnectedState(), "\n")

    def disConnectRobot(self) -> None:
        """
        Disconnect PC RObot
        """
        if self.status == ROBOTCOM_READY:
            self.robot.Disconnect()
            print(f"Stop:{self.robot.ConnectedState()}")

    def fixedRef(self, y_flange2base):
        """
        Fixed refer
        """

        # reference frame flange to base
        pos_flange2base, rot_flange2base = self.robot_module.rotPosRef(380, y_flange2base, 405, 180, 0, 0)
        rf_flage2base = self.robot_module.createRef(pos_flange2base, rot_flange2base)
        print("rf_flage2base:", rf_flage2base, "\n")

        # reference frame camera to flange
        pos_camera2flange, rot_camera2flange = self.robot_module.rotPosRef(
            59, 0, 190, 0, 0, 0
        )
        rf_camera2flange = self.robot_module.createRef(
            pos_camera2flange, rot_camera2flange
        )
        print("ref_camera2flange:", rf_camera2flange, "\n")

        # reference frame camera to base
        rf_camera2base = np.dot(rf_flage2base, rf_camera2flange)
        # print("ref_camera2base:", rf_camera2base, "\n")

        pos_laser2camera, rot_laser2camera = self.robot_module.rotPosRef(
            -54.43, 57, 0, 0, 0, 0
        )
        self.rf_laser2camera = self.robot_module.createRef(
            pos_laser2camera, rot_laser2camera
        )

        rf_laser2flange = np.dot(rf_camera2flange, self.rf_laser2camera)
        pos_laser2flange, rot_laser2flange = self.robot_module.rotPos(rf_laser2flange)

        rf_laser2base = np.dot(rf_camera2base, self.rf_laser2camera)
        print(f'rf_laser2base:{rf_laser2base}')
        pos_laser2base, rot_laser2base = self.robot_module.rotPos(rf_laser2base)
        # print("rot_camera2base:", rot_camera2base, "\n", "pos_camera2base:", pos_camera2base, "\n")

        rf_laser2base_non_matrix = np.concatenate((pos_laser2base, rot_laser2base))
        print("rf_laser2base_non_matrix:", rf_laser2base_non_matrix, "\n")
        self.rf_laser2base_matrix = TxyzRxyz_2_Pose(rf_laser2base_non_matrix)

        rf_laser2flange_non_matrix = np.concatenate(
            (pos_laser2flange, rot_laser2flange)
        )

        rf_laser2flange_matrix = TxyzRxyz_2_Pose(rf_laser2flange_non_matrix)

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
        self.rf_laser2base = rf_laser2base
        return  rf_laser2base

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
        # print(f'rf_laser2base_af:{rf_target2base_af}')

        pos, rot = self.robot_module.rotPos(rf_target2base_af)
        target2base_none_mat = np.concatenate((pos, rot), axis=0)
        # print(f'pos, rot:{pos}, {rot}')

        target2base_mat = TxyzRxyz_2_Pose(target2base_none_mat)
        # print(f"target_laser_mat:{target2base_mat}")

        return target2base_mat, pos

    ### measure distance manually
    def movLeft(self):
        # self.connectRobot()
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.fixedRef(0)
        print(f'self.rf_laser2base:{self.rf_laser2base}')
        self.robot_module.moveLeft(self.rf_laser2base)

    def movRight(self):
        # self.connectRobot()
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.fixedRef(0)
        self.robot_module.moveRight(self.rf_laser2base)
    
    def homePos(self, linearSpeed, joinSpeed):
        self.setRobot(linearSpeed, joinSpeed)
        self.fixedRef(0)
        self.robot.MoveJ(self.rf_laser2base_matrix)

    ### auto measure distance
    def disCameraToObject(self):

        posObject = self.getCoordinates(False)
        x = posObject[0]
        y = posObject[1]
        w = posObject[2]
        h = posObject[3]

        if (x + w) / (y + h) > 1:
            self.robot_module.moveForward(self.rf_laser2base)
            coordinate_pixel_01 = self.getCoordinates(False)

            self.robot_module.moveBack(self.rf_laser2base)
            coordinate_pixel_02 = self.getCoordinates(False)

        else:
            self.robot_module.moveLeft(self.rf_laser2base)
            coordinate_pixel_01 = self.getCoordinates(False)

            self.robot_module.moveRight(self.rf_laser2base)
            coordinate_pixel_02 = self.getCoordinates(False)

        dis_pixel = [coordinate_pixel_01, coordinate_pixel_02]
        print(f"dis_pixel:{dis_pixel}")

        if (
            abs(dis_pixel[0][0] - dis_pixel[1][0]) > 15
            and abs(dis_pixel[0][1] - dis_pixel[1][1]) < 15
        ):

            dis_cameraToObject, pixel_focalLength = (
                self.robot_module.distanceCameraToObject(
                    dis_pixel[0][0],
                    dis_pixel[1][0],
                    CFG.FOCAL_LENGTH,
                    CFG.VERTICAL_BASELINE,
                )
            )  # distance from camera to object
            print(f"Distance:{dis_cameraToObject}")

        elif (
            abs(dis_pixel[0][0] - dis_pixel[1][0]) < 15
            and abs(dis_pixel[0][1] - dis_pixel[1][1]) > 15
        ):

            dis_cameraToObject, pixel_focalLength = (
                self.robot_module.distanceCameraToObject(
                    dis_pixel[0][1],
                    dis_pixel[1][1],
                    CFG.FOCAL_LENGTH,
                    CFG.HORIZONTAL_BASELINE,
                )
            )  # distance from camera to object
            print(f"Distance:{dis_cameraToObject}")

        elif (
            abs(dis_pixel[0][0] - dis_pixel[1][0]) >= 15
            and abs(dis_pixel[0][1] - dis_pixel[1][1]) >= 15
        ) or (
            abs(dis_pixel[0][0] - dis_pixel[1][0]) >= 15
            and abs(dis_pixel[0][1] - dis_pixel[1][1]) >= 15
        ):
            print(f"check 2 pixels")
            # distance from camera to object
        else:
            print("out 5 pixel")

        self.robot.MoveJ(self.rf_laser2base_matrix)
        self.dis_cameraToObject = dis_cameraToObject
        self.pixel_focalLength = pixel_focalLength

        return dis_cameraToObject, dis_pixel

    def getTarget(self, model, image):

        coordinate_pixel = self.getCoordinates(model, image, True)
        theta_laser = self.robot_module.rotLaser(coordinate_pixel)
        coordinate_pixel_1 = coordinate_pixel[0]
        coordinate_pixel_2 = coordinate_pixel[1]
        print(f"coordinate_pixel:{coordinate_pixel}")

        # ### fix value
        self.dis_cameraToObject = 569
        self.pixel_focalLength = CFG.FOCAL_LENGTH * 1000 / CFG.PIXEL_SIZE
        # ###

        x_to_camera_01, y_to_camera_01 = self.robot_module.convertCoordinates(
            CFG.RESOLUTION_X,
            CFG.RESOLUTION_Y,
            coordinate_pixel_1[0],
            coordinate_pixel_1[1],
            self.pixel_focalLength,
            self.dis_cameraToObject,
            theta=90,
        )  # cfg

        x_to_camera_02, y_to_camera_02 = self.robot_module.convertCoordinates(
            CFG.RESOLUTION_X,
            CFG.RESOLUTION_Y,
            coordinate_pixel_2[0],
            coordinate_pixel_2[1],
            self.pixel_focalLength,
            self.dis_cameraToObject,
            theta=90,
        )  # cfg
        z_laser_to_camera = self.dis_cameraToObject - CFG.DISTANCE_LASERtoOBJECT
        if z_laser_to_camera > 600:
            z_laser_to_camera = 330
            print("check the distance")
        real_target01 = np.array([x_to_camera_01, y_to_camera_01, z_laser_to_camera])
        real_target02 = np.array([x_to_camera_02, y_to_camera_02, z_laser_to_camera])

        target01, pos_laser01 = self.createPoint(real_target01, 1, theta_laser)  # fix
        target02, pos_laser02 = self.createPoint(real_target02, 2, theta_laser)
        print(f"target01:{target01}, target02:{target02}")

        return target01, target02

    def runMoveL(self, target_laser_mat):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.robot.MoveL(target_laser_mat)

    def runMoveJ(self, target_laser_mat):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
        self.robot.MoveJ(target_laser_mat)

    def setRobot(self, linearSpeed, jointSpeed):
        self.robot.setRounding(5)  # Set the rounding parameter
        self.robot.setSpeed(linearSpeed)  # Set linear speed in mm/s
        self.robot.setSpeedJoints(jointSpeed)

    def getParam(self):
        """Get custom binary data from this item. Use setParam to set the data"""
        current_joint_values = self.robot.Joints()
        limit = self.robot.JointLimits()

        return current_joint_values, limit

#############################################

def movL():
    global movement_status
    print("jumpt into function")
    VisRob = VisionRobot()
    VisRob.movLeft()
    VisRob.fixedRef(CFG.VERTICAL_BASELINE / 2)
    

def movR():
    VisRob = VisionRobot()
    VisRob.movRight()
    VisRob.fixedRef(-CFG.VERTICAL_BASELINE / 2)

def movHome():
    VisRob = VisionRobot()
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
    VisRob.fixedRef(0)

def run(model, image, pos_status = 'home'):
    VisRob = VisionRobot()
    if pos_status == 'home':
        VisRob.fixedRef(0)
    elif pos_status == 'left':
        VisRob.fixedRef(CFG.VERTICAL_BASELINE / 2)
    elif pos_status == 'right':
        VisRob.fixedRef(-CFG.VERTICAL_BASELINE / 2)
    else:
        stop()
    # dis_cameraToObject, dis_pixel = VisRob.disCameraToObject()

    target01, target02 = VisRob.getTarget(model, image)
    VisRob.runMoveJ(target01)
    # VisRob.sleep_seconds(5)
    VisRob.runMoveL(target02)
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

    current_joint_values, limit = VisRob.getParam()
    data_export = {
        "id": str(datetime.now()),
        # "target01": target01,
        # "target02": target02,
        "limit_lower": limit[0],
        "limit_upper": limit[1],
        # "x1_pixel": dis_pixel[0][0],
        # "y1_pixel": dis_pixel[0][1],
        # "x2_pixel": dis_pixel[1][0],
        # "y2_pixel": dis_pixel[1][1],
        # "distance": dis_cameraToObject,
        #### add if need
    }
    print(f"data_export:{data_export}")
    rob.RobotModule.export_csv(data_export)


def stop():
    VisRob = VisionRobot()
    VisRob.disConnectRobot()

        
if __name__ == "__main__":
    # app.main()
    # run()
    # stop()
    # obj = VisionRobot()
    # connect()
    # movHome()
    movL()
    # obj.getPixelLeft()
