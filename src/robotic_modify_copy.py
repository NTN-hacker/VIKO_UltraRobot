import sys

sys.path.append("E:\Quan\AutoRoboticInspection\VIKO_UltraRobot")
# sys.path.append("E:\Quan\VIKO_UltraRobot")
from robodk.robolink import *
from robodk.robomath import *
import numpy as np
import time
# from layout import app_robot as app
# from library import vision_robotic as vis
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

    def connectRobot(self) -> None:
        """
        Connect Robot and PC
        """
        if not self.robot.Valid():
            raise Exception("Invalid robot selected")

        # RUN_ON_ROBOT = True
        # if self.RDK.RunMode() != RUNMODE_SIMULATE:
        #     RUN_ON_ROBOT = False

        # self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        self.RDK.setRunMode(RUNMODE_SIMULATE)
        # if RUN_ON_ROBOT:
        #     # Connect to the robot using default IP
        #     self.robot.Connect(
        #         "192.168.10.102"
        #     )  # Try to connect once (Or 192.168.10.102)
        #     self.robot.ConnectSafe(
        #         "192.168.10.102"
        #     )  # Try to connect multiple times (Or 192.168.10.102)
        #     self.status, status_msg = self.robot.ConnectedState()

        #     if self.status != ROBOTCOM_READY:
        #         # Stop if the connection did not succeed
        #         print(status_msg)
        #         raise Exception("Failed to connect: " + status_msg)

        #     # This will set to run the API programs on the robot and the simulator (online programming)
        #     self.RDK.setRunMode(RUNMODE_RUN_ROBOT)

        # print("ConnectedState:", self.robot.ConnectedState(), "\n")

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
        pos_flange2rf, rot_flange2rf = self.robot_module.rotPosRef(380, y_flange2rf, 405, 180, 0, 0)
        self.rf_flange2rf = self.robot_module.createRef(pos_flange2rf, rot_flange2rf)

        # reference frame camera to flange
        pos_camera2flange, rot_camera2flange = self.robot_module.rotPosRef(59, 0, 190, 0, 0, 0)
        rf_camera2flange = self.robot_module.createRef(pos_camera2flange, rot_camera2flange)

        # reference frame camera to base
        rf_camera2rf = np.dot(self.rf_flange2rf, rf_camera2flange)

        # reference frame laser to camera
        pos_laser2camera, rot_laser2camera = self.robot_module.rotPosRef(-54.43, 57, 0, 0, 0, 0)
        self.rf_laser2camera = self.robot_module.createRef(pos_laser2camera, rot_laser2camera)

        # reference frame laser to flange
        self.rf_laser2flange = np.dot(rf_camera2flange, self.rf_laser2camera)
        rf_laser2rf = np.dot(rf_camera2rf, self.rf_laser2camera)

        self.rf_laser2rf = rf_laser2rf

        return rf_laser2rf

    
   #####################################################
   #####################################################
    def newCreatePoint(self, target_01, target_02, theta_laser, backOx, alpha):
        """
        (new method) Create point with respect to reference frame
        """
        orgTarget01ToCamera = self.robot_module.intialTarget(target_01[0], target_01[1], target_01[2])
        orgTarget02ToCamera = self.robot_module.intialTarget(target_02[0], target_02[1], target_02[2])

        orgTarget01ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), orgTarget01ToCamera)
        orgTarget02ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), orgTarget02ToCamera)
        
        # print(f'posTargetToLaser, rotTargetToLaser:{posTargetToLaser}, {rotTargetToLaser}')
        print(f'orgTarget01ToLaser: {orgTarget01ToLaser}')

        test_target01toLaser = np.dot(rotz(np.radians(theta_laser)),orgTarget01ToLaser)
        test_posTargetToLaser, test_rotTargetToLaser = self.robot_module.rotPos(test_target01toLaser)
        print(f'test_target01toLaser: {test_posTargetToLaser}, {test_rotTargetToLaser}')

        target01toLaser = np.dot(orgTarget01ToLaser, rotz(np.radians(theta_laser)))
        target02toLaser = np.dot(orgTarget02ToLaser, rotz(np.radians(theta_laser)))
        posTargetToLaser, rotTargetToLaser = self.robot_module.rotPos(target01toLaser)
        print(f'posTargetToLaser, rotTargetToLaser:{posTargetToLaser}, {rotTargetToLaser}')
        # print(f'target01toLaser:{target01toLaser}')
        # print(f'rotz:{rotz(np.radians(theta_laser))}')

        
        target01ToRf = np.dot(self.rf_laser2rf, target01toLaser)
        target02ToRf = np.dot(self.rf_laser2rf, target02toLaser)

        # print('back Ox:', backOx)

        rfToBase = self.robot_module.createRef([0, 0, 0], [np.radians(180), 0, 0])
        target01ToBase = np.dot(rfToBase, target01ToRf)
        target02ToBase = np.dot(rfToBase, target02ToRf)

        pos1ToBase, rot1ToBase = self.robot_module.rotPos(target01ToBase)
        pos2ToBase, rot2ToBase= self.robot_module.rotPos(target02ToBase)
        # print(f'pos1toBase:{pos1ToBase}, pos2toBase:{pos2ToBase}\n rot1ToBase:{rot1ToBase}, rot2ToBase:{rot2ToBase}\n')
        pos2ToPos1 = pos1ToBase - pos2ToBase
        pos_target02_oy = sqrt(pos2ToPos1[0] ** 2 + pos2ToPos1[1] ** 2)
        print(f'pos_target02_oy:{pos_target02_oy}\n')

        if theta_laser > 0:

            newPos2 = np.array([0, pos_target02_oy, 0])
        elif theta_laser <= 0:
            newPos2 = np.array([0, -pos_target02_oy, 0])
        else:
            print(f"check the angle of laser:{theta_laser} \n")

        newRef = target01ToBase
        posRef, rotRef = self.robot_module.rotPos(newRef)
        print(f'posRef, rotRef:{posRef}, {rotRef}')
        newRef_nonMat = np.concatenate((posRef, rotRef), axis=0)
        setRef = TxyzRxyz_2_Pose(newRef_nonMat)
        print(f'setRef:{setRef}')
        self.robot.setPoseFrame(setRef)

        target01ToNewRf = np.dot(target01ToBase, np.linalg.inv(target01ToBase))
        pos01toNewRf, rot01ToNewRf = self.robot_module.rotPos(target01ToNewRf)
        target02ToNewRf = self.robot_module.createRef(newPos2, rot01ToNewRf)
        pos02toNewRf, rot02ToNewRf = newPos2, rot01ToNewRf

        if alpha != 0:
            newT1toNewRf = np.dot(np.dot(target01ToNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))
            newT2toNewRf = np.dot(np.dot(target02ToNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))
        else:
            newT1toNewRf = np.dot(np.dot(target01ToNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx
            newT2toNewRf = np.dot(np.dot(target02ToNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx

        newPos1ToNewRf, newRot1ToNewRf = self.robot_module.rotPos(newT1toNewRf)
        newPos2ToNewRf, newRot2ToNewRf = self.robot_module.rotPos(newT2toNewRf)
      
        newPos1ToNewRf[0] += backOx
        newPos2ToNewRf[0] += backOx

        #######################################
        ####################################### (METHOD 3) test the target to base
        H_target01ToCamera = self.robot_module.createRef([target_01[0], target_01[1], target_01[2]],[0, 0, np.radians(theta_laser)])
        H_target02ToCamera = self.robot_module.createRef([target_02[0], target_02[1], target_02[2]],[0, 0, np.radians(theta_laser)])
        pos01ToCamera, rot01ToCamera = self.robot_module.rotPos(H_target01ToCamera)
        pos02ToCamera, rot02ToCamera = self.robot_module.rotPos(H_target02ToCamera)
        # print(f'pos01ToCamera:{pos01ToCamera}, {rot01ToCamera},\n pos02ToCamera:{pos02ToCamera}, {rot02ToCamera}\n')
        # print(theta_laser)
        H_target01ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), H_target01ToCamera)
        H_target02ToLaser = np.dot(np.linalg.inv(self.rf_laser2camera), H_target02ToCamera)
        H_target01tobase = np.dot(self.rf_laser2rf, H_target01ToLaser)
        H_target02tobase = np.dot(self.rf_laser2rf, H_target02ToLaser)
        newPos01, newRot01 = self.robot_module.rotPos(H_target01tobase)
        newPos02, newRot02 = self.robot_module.rotPos(H_target02tobase)
        # print(f'newPos01:{newPos01}, {newRot01},\n newPos02:{newPos02}, {newRot02}\n')
        H_flangeToBase01 = np.dot(H_target01tobase, np.linalg.inv(self.rf_laser2flange))
        H_flangeToBase02 = np.dot(H_target02tobase, np.linalg.inv(self.rf_laser2flange))
        # print(f'rf_laser2flange:{self.rf_laser2flange}')
        newFlangePos01, newFlangeRot01 = self.robot_module.rotPos(H_flangeToBase01)
        newFlangePos02, newFlangeRot02 = self.robot_module.rotPos(H_flangeToBase02)
        # print(f'newFlangePos01:{newFlangePos01}, {newFlangeRot01},\n newFlangePos02:{newFlangePos02}, {newFlangeRot02}\n')

        ##############################################
        ##############################################

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
        # print(f"target01_none_mat:{target01_none_mat},\n target02_none_mat:{target02_none_mat}")

        return target01, target02, pos_target02_oy
        # return newTarget01, newTarget02, pos_target02_oy

    def homePos(self, linearSpeed, joinSpeed):
        self.setRobot(linearSpeed, joinSpeed)
        self.fixedRef(0)
        self.robot.MoveJ(self.rf_laser2rf_matrix)

    def getTarget(self, coordinate_pixel, shape, _suf_: int):
        # target01, target02, theta_laser = None, None, None
        test_target01, test_target02, theta_laser = None, None, None
        # print(f"shape", shape)
        theta_laser = self.robot_module.rotLaser(coordinate_pixel)

        coordinate_pixel_1 = coordinate_pixel[0]
        coordinate_pixel_2 = coordinate_pixel[1]
        # print(f"coordinate_pixel:{coordinate_pixel}")

        # ### fix value
        self.dis_cameraToObject = CFG.DISTANCE_CAMERA2OBJECT

        self.pixel_focalLength = CFG.FOCAL_LENGTH * 1000 / CFG.PIXEL_SIZE

        x_to_camera_01, y_to_camera_01 = self.robot_module.convertCoordinates(CFG.RESOLUTION_X, CFG.RESOLUTION_Y, coordinate_pixel_1[0], coordinate_pixel_1[1], 
                                                                              self.pixel_focalLength, self.dis_cameraToObject, theta=90,)  # cfg
        x_to_camera_02, y_to_camera_02 = self.robot_module.convertCoordinates(CFG.RESOLUTION_X, CFG.RESOLUTION_Y, coordinate_pixel_2[0], coordinate_pixel_2[1], 
                                                                              self.pixel_focalLength,self.dis_cameraToObject,theta=90)  # cfg
        
        z_laser_to_object = self.dis_cameraToObject - CFG.DISTANCE_LASER2OBJECT
       
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
                test_target01, test_target02, test_length_weld = self.newCreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)
            else:
                backOx = (
                    np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.OX_POS_LASER_POSITIVE)
                # backOx = 0
                # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
                test_target01, test_target02, test_length_weld = self.newCreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)

        elif shape == "30_degree":
            alpha = CFG.ROTATE_OY_LASER * _suf_
            print(f"alpha:{alpha}")
            if alpha < 0:
                backOx = (
                    -np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.OX_POS_LASER_NEGATIVE)  #### need more condition
                # backOx = 0
                # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
                test_target01, test_target02, test_length_weld = self.newCreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)
            else:
                backOx = (np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.OX_POS_LASER_POSITIVE)  #### need more condition
                # backOx = 0bb
                # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
                test_target01, test_target02, test_length_weld = self.newCreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)

        elif shape == "0_degree":
            alpha = backOx = 0
            # target01, target02, length_weld = self.createPoint(real_target01, real_target02, theta_laser, backOx, alpha)
            test_target01, test_target02, test_length_weld = self.newCreatePoint(real_target01, real_target02, theta_laser, backOx, alpha)
        else:
            stop()

        # target01, target02 = self.createPoint(real_target01, real_target02, theta_laser)  # fix
        # print(f'testTarget01:{test_target01}, \n, testTarget02:{test_target02}, \n, test_length_weld:{test_length_weld}')

        # return target01, target02, theta_laser, length_weld
        return test_target01, test_target02, theta_laser , test_length_weld, coordinate_pixel_1, coordinate_pixel_2
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


def movHome():
    VisRob = VisionRobot()
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

def stop():
    VisRob = VisionRobot()
    VisRob.disConnectRobot()

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

    for index, coordinate_pixel in enumerate(coordinate_pixel_list):
        # print(f"Weld model {model_weld_list[index]}")
        target01, target02, theta_laser, length_weld, coordinate_pixel_1, coordinate_pixel_2 = VisRob.getTarget(coordinate_pixel, model_weld_list[index], _suf_)
        VisRob.runMoveJ(target01)

        if VisRob.runMoveJ(target01) == True:
            trigger(length_weld)
            # VisRob.runMoveL(target02)
        else:
            print("Error: Failed to reach target01")

    # VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

    current_joint_values, limit = VisRob.getParam()
    data_export = {
        "id": str(datetime.now()),
        "pixel_01": coordinate_pixel_1,
        "pixel_02": coordinate_pixel_2,
        "theta_laser": theta_laser,
        #### add if need
    }
    # print(f"data_export:{data_export}")
    rob.RobotModule.export_csv(data_export)


if __name__ == "__main__":
    # app.main()
    VisRob = VisionRobot()
    VisRob.fixedRef(0)
    VisRob.getTarget(CFG.TEST_TARGET, "0_degree", -1)
    # obj = VisionRobot()
    print('done')

