import sys
sys.path.append("D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot")
from robodk.robolink import *
from robodk.robomath import *
import numpy as np
import time
from datetime import datetime

from layout import app_robot as app
from library import vision_robotic as vis
from library import robot_lib_modify_nhan as rob

from config import config as CFG


## class for vision robot
class VisionRobot:
    def __init__(self, RDK=None, robot=None):
        self.robot = rob.RobotModule()
        self.robot.connectRobot()
        self.rf_laser2camera, self.rf_laser2base, self.rf_laser2base_matrix = self.robot.fixedRef()

    def getCoordinates(self, model, image):
        self.vision = vis.VisionModule()
        self.vision.load_model(model)
        coordinate = self.vision._getCoordinateTest_(image)
        return coordinate
    
    ### measure distance manually
    def movLeft(self):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.robot.moveLeft(self.rf_laser2base)

    def movRight(self):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.robot.moveRight(self.rf_laser2base)

    ### auto measure distance
    def disCameraToObject(self):

        posObject = self.getCoordinates(False)
        x = posObject[0]
        y = posObject[1]
        w = posObject[2]
        h = posObject[3]

        if (x + w) / (y + h) > 1:
            self.robot.moveForward(self.rf_laser2base)
            coordinate_pixel_01 = self.getCoordinates()

            self.robot.moveBack(self.rf_laser2base)
            coordinate_pixel_02 = self.getCoordinates()

        else:
            self.robot.moveLeft(self.rf_laser2base)
            coordinate_pixel_01 = self.getCoordinates()

            self.robot.moveRight(self.rf_laser2base)
            coordinate_pixel_02 = self.getCoordinates()

        dis_pixel = [coordinate_pixel_01, coordinate_pixel_02]
        print(f"dis_pixel:{dis_pixel}")

        if (abs(dis_pixel[0][0] - dis_pixel[1][0]) > 15) and (abs(dis_pixel[0][1] - dis_pixel[1][1]) < 15):

            dis_cameraToObject, pixel_focalLength = (
                rob.distanceCameraToObject(
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
                rob.distanceCameraToObject(
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
        theta_laser = rob.rotLaser(coordinate_pixel)
        coordinate_pixel_1 = coordinate_pixel[0]
        coordinate_pixel_2 = coordinate_pixel[1]
        print(f"coordinate_pixel:{coordinate_pixel}")

        # ### fix value
        self.dis_cameraToObject = CFG.DISTANCE_CAMERA2OBJECT
        self.pixel_focalLength = CFG.FOCAL_LENGTH * 1000 / CFG.PIXEL_SIZE
        # ###

        x_to_camera_01, y_to_camera_01 = rob.convertCoordinates(
            CFG.RESOLUTION_X,
            CFG.RESOLUTION_Y,
            coordinate_pixel_1[0],
            coordinate_pixel_1[1],
            self.pixel_focalLength,
            self.dis_cameraToObject,
            theta=90,
        )  # cfg

        x_to_camera_02, y_to_camera_02 = rob.convertCoordinates(
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

        target01, pos_laser01 = self.robot.createPoint(real_target01, 1, theta_laser, self.rf_laser2camera, self.rf_laser2base)  # fix
        target02, pos_laser02 = self.robot.createPoint(real_target02, 2, theta_laser, self.rf_laser2camera, self.rf_laser2base)
        print(f"target01:{target01}, target02:{target02}")

        return target01, target02

    def runMoveL(self, target_laser_mat):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.robot.robot.MoveL(target_laser_mat)

    def runMoveJ(self, target_laser_mat):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
        self.robot.robot.MoveJ(target_laser_mat)

    def homePos(self, linearSpeed, joinSpeed):
        self.setRobot(linearSpeed, joinSpeed)
        self.robot.robot.MoveJ(self.rf_laser2base_matrix)
        return 0

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


def movLeft():
    VisRob = VisionRobot()
    VisRob.movLeft()


def movRight():
    VisRob = VisionRobot()
    VisRob.movRight()


def movHome():
    VisRob = VisionRobot()
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])


def run(model, image):
    VisRob = VisionRobot()
    VisRob.fixedRef()

    # dis_cameraToObject, dis_pixel = VisRob.disCameraToObject()
    target01, target02 = VisRob.getTarget(model, image)
    VisRob.runMoveJ(target01)
    VisRob.sleep_seconds(5)
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
    movLeft()
    # obj.getPixelLeft()
