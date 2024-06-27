import sys

sys.path.append("D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot")
# sys.path.append("E:\Project\Robot-6DOF\VIKO_UltraRobot")
from robodk.robolink import *
from robodk.robomath import *
import numpy as np
import time
# from layout import app_robot as app

from library import vision_robotic as vis
from library import robot_lib_modify as rob
from datetime import datetime
from config import config as CFG
import robodk as rdk


## class for vision robot
class VisionRobot:
    def __init__(self, RDK=None, robot=None):
        self.RDK = Robolink()
        self.robot = self.RDK.ItemUserPick("Motoman GP8 Base", ITEM_TYPE_ROBOT)
        self.robot_module = rob.RobotModule()
        self.connectRobot()
        # self.fixedRef()

    def connectRobot(self) -> None:
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
            self.robot.Connect(
                "192.168.10.102"
            )  # Try to connect once (Or 192.168.10.102)
            self.robot.ConnectSafe(
                "192.168.10.102"
            )  # Try to connect multiple times (Or 192.168.10.102)
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

    def fixedRef(self, y_flange2rf):
        """
        Fixed refer
        """

        # reference frame flange to base
        pos_flange2rf, rot_flange2rf = self.robot_module.rotPosRef(
            380, y_flange2rf, -405, 0, 0, 0
        )
        self.rf_flange2rf = self.robot_module.createRef(pos_flange2rf, rot_flange2rf)
        # print("rf_flange2rf:", self.rf_flange2rf, "\n")

        # reference frame camera to flange
        pos_camera2flange, rot_camera2flange = self.robot_module.rotPosRef(
            59, 0, 190, 0, 0, 0
        )
        rf_camera2flange = self.robot_module.createRef(
            pos_camera2flange, rot_camera2flange
        )
        # print("ref_camera2flange:", rf_camera2flange, "\n")

        # reference frame camera to base
        rf_camera2rf = np.dot(self.rf_flange2rf, rf_camera2flange)
        # print("ref_camera2rf:", rf_camera2rf, "\n")

        pos_laser2camera, rot_laser2camera = self.robot_module.rotPosRef(
            -54.43, 57, 0, 0, 0, 0
        )
        # print(f'pos_laser2camera, rot_laser2camera:{pos_laser2camera}, {rot_laser2camera}')

        self.rf_laser2camera = self.robot_module.createRef(
            pos_laser2camera, rot_laser2camera
        )

        self.rf_laser2flange = np.dot(rf_camera2flange, self.rf_laser2camera)
        pos_laser2flange, rot_laser2flange = self.robot_module.rotPos(
            self.rf_laser2flange
        )
        # print(f'pos_laser2flange, rot_laser2flange:{pos_laser2flange}, {rot_laser2flange}')
        rf_laser2rf = np.dot(rf_camera2rf, self.rf_laser2camera)
        print(f"rf_laser2rf:{rf_laser2rf}")
        pos_laser2rf, rot_laser2rf = self.robot_module.rotPos(rf_laser2rf)
        # print("rot_camera2rf:", rot_camera2rf, "\n", "pos_camera2rf:", pos_camera2rf, "\n")

        rf_laser2rf_non_matrix = np.concatenate((pos_laser2rf, rot_laser2rf))
        # print("rf_laser2rf_non_matrix:", rf_laser2rf_non_matrix, "\n")
        self.rf_laser2rf_matrix = TxyzRxyz_2_Pose(rf_laser2rf_non_matrix)

        rf_laser2flange_non_matrix = np.concatenate(
            (pos_laser2flange, rot_laser2flange)
        )

        rf_laser2flange_matrix = TxyzRxyz_2_Pose(rf_laser2flange_non_matrix)

        pos_setFrame = [0, 0, 0]  # Translation vector [Tx, Ty, Tz] ## sai so
        rot_setFramee = [
            np.radians(180),
            np.radians(0),
            np.radians(0),
        ]  # Rotation angles [Rx, Ry, Rz] in radians
        rf_setFrame = np.concatenate((pos_setFrame, rot_setFramee))
        setFrame = TxyzRxyz_2_Pose(rf_setFrame)

        self.robot.setPoseFrame(setFrame)
        # print(f"robot.PoseFrame():{robot.PoseFrame()}")
        self.robot.setPoseTool(rf_laser2flange_matrix)
        self.rf_laser2rf = rf_laser2rf
        print(f"Pose:{self.robot.Pose()}")
        return rf_laser2rf

    @staticmethod
    def sleep_seconds(seconds):
        print(f"Sleeping for {seconds} seconds...")
        time.sleep(seconds)
        print("Awake now!")



    def createPoint(self, target_01, target_02, theta_laser, backOx, alpha):
        
        ###### need to rotate Oz before figure out the target to laser
        ###### targetToBase = rfToBase * laserToRf * pointFrameToLaser * pointToPointFrame (rotOz)
        ###### targetToRf = laserToRf * orgTargetToLaser * targetTo_OrgTarget (rotOz)
        

        target01ToCamera = self.robot_module.intialTarget(
            target_01[0], target_01[1], target_01[2]
        )
        target02ToCamera = self.robot_module.intialTarget(
            target_02[0], target_02[1], target_02[2]
        )

        target01ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), target01ToCamera)
        target02ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), target02ToCamera)

        target01toRf = np.dot(self.rf_laser2rf, target01ToLaser)
        pos1, rot1 = self.robot_module.rotPos(target01toRf)
        print(f"orgpos1, orgrot1:{pos1}, {rot1}")
        target01ToBase = np.dot(
            np.dot(rotx(np.radians(180)), target01toRf), rotz(np.radians(theta_laser))
        )
        pos1toBase, rot1toBase = self.robot_module.rotPos(target01ToBase)
        # print(f"pos1toBase, rot1toBase:{pos1toBase}, {rot1toBase}" "\n")

        target02toRf = np.dot(self.rf_laser2rf, target02ToLaser)
        pos2, rot2 = self.robot_module.rotPos(target02toRf)
        print(f"orgpos2, orgrot2:{pos2}, {rot2}")
        target02ToBase = np.dot(
            np.dot(rotx(np.radians(180)), target02toRf), rotz(np.radians(theta_laser))
        )
        pos2toBase, rot2toBase = self.robot_module.rotPos(target02ToBase) 
        # print(f"pos2toBase, rot2toBase:{pos2toBase}, {rot2toBase}" "\n")

        pos2toPos1 = pos2toBase - pos1toBase
        pos_target02_oy = sqrt(pos2toPos1[0] ** 2 + pos2toPos1[1] ** 2) + CFG.ERROR_POS
        # print(f"pos_target02_oy:{pos_target02_oy}")

        if theta_laser > 0:
            pos_target02 = np.array([0, pos_target02_oy, 0])
        elif theta_laser <= 0:
            pos_target02 = np.array([0, -pos_target02_oy, 0])
        else:
            print(f"check the angle of laser:{theta_laser}")

        newPos1, newRot1 = self.robot_module.rotPos(target01ToBase)
        print(f"new1pos, new1rot: {newPos1}, {newRot1}")
        target2base_none_mat = np.concatenate((newPos1, newRot1), axis=0)
        newRf = TxyzRxyz_2_Pose(target2base_none_mat)
        self.robot.setPoseFrame(newRf)

        # target with respect to reference frame
        target01toRf = np.dot(target01ToBase, np.linalg.inv(target01ToBase))
        target02toRf = self.robot_module.createRef(pos_target02, rot2)
        if alpha != 0:
            newTarget01 = np.dot(
                np.dot(target01toRf, roty(np.radians(alpha))), rotx(np.radians(0))
            )
            newTarget02 = np.dot(
                np.dot(target02toRf, roty(np.radians(alpha))), rotx(np.radians(0))
            )
        else:
            newTarget01 = np.dot(
                np.dot(target01toRf, roty(np.radians(alpha))), rotx(np.radians(0))
            )
            newTarget02 = np.dot(
                np.dot(target02toRf, roty(np.radians(alpha))), rotx(np.radians(0))
            )

        print(f"back Ox: {backOx}")
        Pos1, Rot1 = self.robot_module.rotPos(newTarget01)
        print(f"Pos1 does not change:{Pos1}")
        Pos1[0] += backOx

        if alpha != 0:
            Pos1[1] += 0
        else:
            # pass
            print(f"Pos1 changed:{Pos1}")

        new01 = self.robot_module.createRef(Pos1, Rot1)
        new01toBase = np.dot(target01ToBase, new01)
        newPos01toBase, newRot01toBase = self.robot_module.rotPos(new01toBase)
        print(f"newPos01toBase:{newPos01toBase}, \n newRot01toBase:{newRot01toBase}")

        Pos2, Rot2 = self.robot_module.rotPos(newTarget02)
        print(f"Pos2 does not change:{Pos2}")
        Pos2[0] += backOx

        if alpha != 0:
            Pos2[1] += 0
        else:
            # pass
            print(f"Pos2 changed:{Pos2}")

        new02 = self.robot_module.createRef(Pos2, Rot2)
        new02toBase = np.dot(target01ToBase, new02)
        newPos02toBase, newRot02toBase = self.robot_module.rotPos(new02toBase)
        print(f"newPos02toBase:{newPos02toBase}, \n newRot02toBase:{newRot02toBase}")


        target01_none_mat = np.concatenate((Pos1, Rot1), axis=0)
        target02_none_mat = np.concatenate((Pos2, Rot2), axis=0)

        target01 = TxyzRxyz_2_Pose(target01_none_mat)
        target02 = TxyzRxyz_2_Pose(target02_none_mat)
        # print(f"target02:{target02}")

        return target01, target02, pos_target02_oy
    
    
   #####################################################
   #####################################################
    def newcreatePoint(self, target_01, target_02, theta_laser, backOx, alpha):
        """
        (new method) Create point with respect to reference frame
        """
        orgTarget01ToCamera = self.robot_module.intialTarget(target_01[0], target_01[1], target_01[2])
        orgTarget02ToCamera = self.robot_module.intialTarget(target_02[0], target_02[1], target_02[2])

        orgTarget01ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), orgTarget01ToCamera)
        orgTarget02ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), orgTarget02ToCamera)

        targetToOrgTarget = rotz(np.radians(theta_laser))

        target01toLaser = np.dot(orgTarget01ToLaser, targetToOrgTarget)
        target02toLaser = np.dot(orgTarget02ToLaser, targetToOrgTarget)

        target01ToRf = np.dot(self.rf_laser2rf, target01toLaser)
        target02ToRf = np.dot(self.rf_laser2rf, target02toLaser)

        pos1toRf, _ = self.robot_module.rotPos(target01ToRf)
        pos2toRf, _ = self.robot_module.rotPos(target02ToRf)
        print(f'pos1toRf:{pos1toRf},\n pos2toRf:{pos2toRf} \n')
        pos2ToPos1_rf = pos2toRf - pos1toRf
        pos_target02_oy_rf = sqrt(pos2ToPos1_rf[0] ** 2 + pos2ToPos1_rf[1] ** 2)
        print(f'pos_target02_oy_rf:{pos_target02_oy_rf} \n')

        rfToBase = self.robot_module.createRef([0, 0, 0], [np.radians(180), 0, 0])
        target01ToBase = np.dot(rfToBase, target01ToRf)
        target02ToBase = np.dot(rfToBase, target02ToRf)

        pos1toBase, _ = self.robot_module.rotPos(target01ToBase)
        pos2toBase, _ = self.robot_module.rotPos(target02ToBase)
        print(f'pos1toBase:{pos1toBase},\n pos2toBase:{pos2toBase}')
        pos2ToPos1 = pos2toBase - pos1toBase
        pos_target02_oy = sqrt(pos2ToPos1[0] ** 2 + pos2ToPos1[1] ** 2)
        print(f'pos_target02_oy:{pos_target02_oy} \n')

        if theta_laser > 0:
            newPos2 = np.array([0, pos_target02_oy, 0])
        elif theta_laser <= 0:
            newPos2 = np.array([0, -pos_target02_oy, 0])
        else:
            print(f"check the angle of laser:{theta_laser} \n")

        newRef = target01ToBase
        posRef, rotRef = self.robot_module.rotPos(newRef)
        newRef_nonMat = np.concatenate((posRef, rotRef), axis=0)
        setRef = TxyzRxyz_2_Pose(newRef_nonMat)
        self.robot.setPoseFrame(setRef)

        target01ToNewRf = np.dot(target01ToBase, np.linalg.inv(target01ToBase))
        pos01toNewRf, rot01ToNewRf = self.robot_module.rotPos(target01ToNewRf)
        target02toNewRf = self.robot_module.createRef(newPos2, rot01ToNewRf)
        pos02toNewRf, rot02ToNewRf = newPos2, rot01ToNewRf

        if alpha != 0:
            newT1toNewRf = np.dot(np.dot(target01ToNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))
            newT2toNewRf = np.dot(np.dot(target02toNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))
        else:
            newT1toNewRf = np.dot(np.dot(target01ToNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx
            newT2toNewRf = np.dot(np.dot(target02toNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx

        newPos1ToNewRf, newRot1ToNewRf = self.robot_module.rotPos(newT1toNewRf)
        newPos2ToNewRf, newRot2ToNewRf = self.robot_module.rotPos(newT2toNewRf)

        newPos1ToNewRf[0] += backOx
        newPos2ToNewRf[0] += backOx

        ##### calculate the target to base
        pos01toNewRf[0] += backOx
        pos02toNewRf[0] += backOx
        rot01ToNewRf[0], rot01ToNewRf[1] = np.radians(0), np.radians(alpha)
        rot02ToNewRf[0], rot02ToNewRf[1] = np.radians(0), np.radians(alpha)
        newT1toNewRf = np.concatenate((pos01toNewRf, rot01ToNewRf), axis=0)
        newT2toNewRf = np.concatenate((pos02toNewRf, rot02ToNewRf), axis=0)

        new01 = self.robot_module.createRef(pos01toNewRf, rot01ToNewRf)
        new01toBase = np.dot(newRef, new01)
        newPos01toBase, newRot01toBase = self.robot_module.rotPos(new01toBase)
        print(f"newPos01toBase:{newPos01toBase},\n newRot01toBase:{newRot01toBase} \n")

        new02 = self.robot_module.createRef(pos02toNewRf, rot02ToNewRf)
        new02toBase = np.dot(newRef, new02)
        newPos02toBase, newRot02toBase = self.robot_module.rotPos(new02toBase)
        print(f"newPos02toBase:{newPos02toBase},\n newRot02toBase:{newRot02toBase} \n")
        #######################################################################

        if alpha != 0:
            newPos1ToNewRf[1] += 0
            newPos2ToNewRf[1] += 0
        else:
            # pass
            print(f"Pos1 and Pos2 changed:{newPos1ToNewRf}, {newPos2ToNewRf}")

        target01_none_mat = np.concatenate((newPos1ToNewRf, newRot1ToNewRf), axis=0)
        target02_none_mat = np.concatenate((newPos2ToNewRf, newRot2ToNewRf), axis=0)
        target01 = TxyzRxyz_2_Pose(target01_none_mat)
        target02 = TxyzRxyz_2_Pose(target02_none_mat)

        return target01, target02, pos_target02_oy

    ### measure distance manually
    def movLeft(self):
        # self.connectRobot()
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.fixedRef(0)
        print(f"self.rf_laser2base:{self.rf_laser2rf}")
        self.robot_module.moveLeft(self.rf_laser2rf)

    def movRight(self):
        # self.connectRobot()
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.fixedRef(0)
        self.robot_module.moveRight(self.rf_laser2rf)

    def homePos(self, linearSpeed, joinSpeed):
        self.setRobot(linearSpeed, joinSpeed)
        self.fixedRef(0)
        self.robot.MoveJ(self.rf_laser2rf_matrix)

    ### auto measure distance
    def disCameraToObject(self):

        posObject = self.getCoordinates(False)
        x = posObject[0]
        y = posObject[1]
        w = posObject[2]
        h = posObject[3]

        if (x + w) / (y + h) > 1:
            self.robot_module.moveForward(self.rf_laser2rf)
            coordinate_pixel_01 = self.getCoordinates(False)

            self.robot_module.moveBack(self.rf_laser2rf)
            coordinate_pixel_02 = self.getCoordinates(False)

        else:
            self.robot_module.moveLeft(self.rf_laser2rf)
            coordinate_pixel_01 = self.getCoordinates(False)

            self.robot_module.moveRight(self.rf_laser2rf)
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

        self.robot.MoveJ(self.rf_laser2rf_matrix)
        self.dis_cameraToObject = dis_cameraToObject
        self.pixel_focalLength = pixel_focalLength

        return dis_cameraToObject, dis_pixel

    def getTarget(self, coordinate_pixel, shape, _suf_: int):
        # target01, target02, theta_laser = None, None, None
        test_target01, test_target02, theta_laser = None, None, None
        print(f"shape", shape)
        theta_laser = self.robot_module.rotLaser(coordinate_pixel)

        coordinate_pixel_1 = coordinate_pixel[0]
        coordinate_pixel_2 = coordinate_pixel[1]
        print(f"coordinate_pixel:{coordinate_pixel}")

        # ### fix value
        self.dis_cameraToObject = CFG.DISTANCE_CAMERA2OBJECT

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
        z_laser_to_object = self.dis_cameraToObject - CFG.DISTANCE_LASERtoOBJECT
        print(f"z_laser_to_object:{z_laser_to_object}")
        if z_laser_to_object > CFG.SAFE_DISTANCE:
            z_laser_to_object = 330
            print("check the distance")
        real_target01 = np.array([x_to_camera_01, y_to_camera_01, z_laser_to_object])
        real_target02 = np.array([x_to_camera_02, y_to_camera_02, z_laser_to_object])

        if shape == "90_degree":
            alpha = CFG.ROTATE_OY_LASER * _suf_
            print(f"alpha:{alpha}")
            if alpha < 0:
                backOx = (-np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.OX_POS_LASER_NEGATIVE)
                # backOx = 0
                # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
                test_target01, test_target02, test_length_weld = self.newcreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)
            else:
                backOx = (
                    np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.OX_POS_LASER_POSITIVE)
                # backOx = 0
                # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
                test_target01, test_target02, test_length_weld = self.newcreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)

        elif shape == "30_degree":
            alpha = CFG.ROTATE_OY_LASER * _suf_
            print(f"alpha:{alpha}")
            if alpha < 0:
                backOx = (
                    -np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.OX_POS_LASER_NEGATIVE)  #### need more condition
                # backOx = 0
                # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
                test_target01, test_target02, test_length_weld = self.newcreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)
            else:
                backOx = (np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.OX_POS_LASER_POSITIVE)  #### need more condition
                # backOx = 0
                # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
                test_target01, test_target02, test_length_weld = self.newcreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)

        elif shape == "0_degree":
            alpha = backOx = 0
            # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
            test_target01, test_target02, test_length_weld = self.newcreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)
        else:
            stop()

        # target01, target02 = self.createPoint(real_target01, real_target02, theta_laser)  # fix
        # print(f'testTarget01:{test_target01}, \n, testTarget02:{test_target02}, \n, test_length_weld:{test_length_weld}')

        # return target01, target02, theta_laser, length_weld
        return test_target01, test_target02, theta_laser , test_length_weld
        # return target01

    def runMoveL(self, target_laser_mat):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[0])
        self.robot.MoveL(target_laser_mat)

    def runMoveJ(self, target_laser_mat):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
        self.robot.MoveJ(target_laser_mat)

        return True

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


def movR():
    VisRob = VisionRobot()
    VisRob.movRight()


def movHome():
    VisRob = VisionRobot()
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])


def run(model, image, _suf_, pos_status="home"):
    VisRob = VisionRobot()
    if pos_status == "home":
        VisRob.fixedRef(0)
    elif pos_status == "left":
        VisRob.fixedRef(CFG.VERTICAL_BASELINE / 2)
    elif pos_status == "right":
        VisRob.fixedRef(-CFG.VERTICAL_BASELINE / 2)
    else:
        stop()
    # dis_cameraToObject, dis_pixel = VisRob.disCameraToObject()

    ## trigger and length to scan
    def trigger(length_weld):
        if os.path.exists("trigger.txt") and os.path.exists("length.txt"):
            os.remove("trigger.txt")
            os.remove("length.txt")
        with open("trigger.txt", "w") as f:
            f.write("True")
        with open("length.txt", "w") as f:
            f.write(str(length_weld))

    coordinate_pixel_list, model_weld_list = vis.getCoordinates(model, image, True)
    print(f"Weld model {model_weld_list}")
    # coordinate_pixel = coordinate_pixel_list[0]
    for index, coordinate_pixel in enumerate(coordinate_pixel_list):
        print(f"Weld model {model_weld_list[index]}")
        target01, target02, theta_laser, length_weld = VisRob.getTarget(
            coordinate_pixel, model_weld_list[index], _suf_
        )
        VisRob.runMoveJ(target01)
        VisRob.sleep_seconds(2)
        if VisRob.runMoveJ(target01) == True:
            trigger(length_weld)
            VisRob.runMoveL(target02)
        else:
            print("Error: Failed to reach target01")

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
        "theta_laser": theta_laser,
        #### add if need
    }
    # print(f"data_export:{data_export}")
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
