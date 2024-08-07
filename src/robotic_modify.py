import sys

sys.path.append("E:\Quan\AutoRoboticInspection-V1\VIKO_UltraRobot")
# sys.path.append("E:\Project\Robot-6DOF\VIKO_UltraRobot")
from robodk.robolink import *
from robodk.robomath import *
import numpy as np
import time
from library import vision_robotic as vis
from library import robot_lib_modify as rob
from datetime import datetime
from config import config as CFG


def getCoordinates(model, image):
    return vis.getCoordinates(model, image, True)

def movHome():
    VisRob = VisionRobot()
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

def stop():
    VisRob = VisionRobot()
    VisRob.disConnectRobot()

def run(coordinate_pixel_list, model_weld_list, _suf_, pos_status="home"):
    try:
        with open("trigger.txt", "r") as f:
            checkData = f.read().strip()
    except FileNotFoundError:
        checkData = ''  # Set to empty string if file doesn't exist

    if checkData != '0':
        with open("trigger.txt", "w") as f:
            f.write('0')

    VisRob = VisionRobot()
    if pos_status == "home":
        VisRob.fixedRef(0)

    else:
        stop()
    
    # Trigger and length to scan
    def trigger(length_weld, trigger=False):
        assert length_weld != 0, "No weld to scan. Can not trigger the laser"
        with open("trigger.txt", "w") as f:
            data = f'{trigger}'
            f.write(data)
            print(f'trigger:{data}')

    for index, coordinate_pixel in enumerate(coordinate_pixel_list):
        print(f"Weld model {model_weld_list[index]}")
        target01, target02, thetaLaser, lengthWeld = VisRob.getTarget(coordinate_pixel, model_weld_list[index], _suf_)
        time_scan = lengthWeld/CFG.RESOLUTION_Y_LASER/CFG.FREQUENCY
        speeedScan = lengthWeld / time_scan
        print('The information robotic laser ')
        print('Speed scan: ', speeedScan)
        print('Length planing ', lengthWeld)
        
        startWeld = VisRob.runMoveJ(target01)
        # startWeld = True

        if startWeld:
            trigger(lengthWeld, 1)
            def robot_movement():
                time.sleep(1)
                VisRob.runMoveL(target02, speeedScan)
                time.sleep(5)
                # VisRob.runMoveL(target02, CFG.LINEAR_SPEEDS[1])
                trigger(lengthWeld, 0)

            robot_movement()  
        else:
            print("Error: Failed to reach target01")

    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

    data_export = {
        "id": str(datetime.now()),
        "theta_laser": thetaLaser,
        # Add additional data to export if needed
    }
    print(f"data_export: {data_export}")
    # rob.RobotModule.export_csv(data_export)   


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

        RUN_ON_ROBOT = True
        if self.RDK.RunMode() != RUNMODE_SIMULATE:
            RUN_ON_ROBOT = False

        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        if RUN_ON_ROBOT:
            # Connect to the robot using default IP
            self.robot.Connect("192.168.10.102")  # Try to connect once (Or 192.168.10.111)
            self.robot.ConnectSafe("192.168.10.102")  # Try to connect multiple times (Or 192.168.10.111)
            self.status, status_msg = self.robot.ConnectedState()

            if self.status != ROBOTCOM_READY:
                # Stop if the connection did not succeed
                raise Exception("Failed to connect: " + status_msg)

            # This will set to run the API programs on the robot and the simulator (online programming)
            self.RDK.setRunMode(RUNMODE_RUN_ROBOT)

    def disConnectRobot(self) -> None:
        """
        Disconnect PC RObot
        """
        if self.status == ROBOTCOM_READY:
            self.robot.Disconnect()
            # print(f"Stop:{self.robot.ConnectedState()}")

    def fixedRef(self, y_flange2rf):
        """
        Fixed refer
        """

        # reference frame flange to base
        pos_flange2rf, rot_flange2rf = self.robot_module.rotPosRef(380, y_flange2rf, -405, 0, 0, 0)
        self.rf_flange2rf = self.robot_module.createRef(pos_flange2rf, rot_flange2rf)

        # reference frame camera to flange
        pos_camera2flange, rot_camera2flange = self.robot_module.rotPosRef(0, -70.5, 88, 0, 0, 0)
        rf_camera2flange = self.robot_module.createRef(pos_camera2flange, rot_camera2flange)

        rf_camera2rf = np.dot(self.rf_flange2rf, rf_camera2flange)

        # reference laser to camera
        pos_laser2camera, rot_laser2camera = self.robot_module.rotPosRef(0, 151, 12, 0, 0, 0)
        self.rf_laser2camera = self.robot_module.createRef(pos_laser2camera, rot_laser2camera)

        self.rf_laser2flange = np.dot(rf_camera2flange, self.rf_laser2camera)
        pos_laser2flange, rot_laser2flange = self.robot_module.rotPos(self.rf_laser2flange)

        rf_laser2rf = np.dot(rf_camera2rf, self.rf_laser2camera)
        pos_laser2rf, rot_laser2rf = self.robot_module.rotPos(rf_laser2rf)

        rf_laser2rf_non_matrix = np.concatenate((pos_laser2rf, rot_laser2rf))
        self.rf_laser2rf_matrix = TxyzRxyz_2_Pose(rf_laser2rf_non_matrix)

        rf_laser2flange_non_matrix = np.concatenate((pos_laser2flange, rot_laser2flange))

        rf_laser2flange_matrix = TxyzRxyz_2_Pose(rf_laser2flange_non_matrix)

        pos_setFrame = [0, 0, 0]  # Translation vector [Tx, Ty, Tz] ## sai so
        rot_setFramee = [np.radians(180), np.radians(0), np.radians(0)]  # Rotation angles [Rx, Ry, Rz] in radians
        rf_setFrame = np.concatenate((pos_setFrame, rot_setFramee))
        setFrame = TxyzRxyz_2_Pose(rf_setFrame)

        self.robot.setPoseFrame(setFrame)
        self.robot.setPoseTool(rf_laser2flange_matrix)
        self.rf_laser2rf = rf_laser2rf
        return rf_laser2rf
    
    def test_fixedRef(self):
        """
        Fixed refer
        """

        # reference frame flange to base
        test_pos_flange2rf, test_rot_flange2rf = self.robot_module.rotPosRef(380, 0, 405, 180, 0, 0)
        self.test_rf_flange2rf = self.robot_module.createRef(test_pos_flange2rf, test_rot_flange2rf)

        # reference frame camera to flange
        test_pos_camera2flange, test_rot_camera2flange = self.robot_module.rotPosRef(0, -70.5, 88, 0, 0, 0)
        test_rf_camera2flange = self.robot_module.createRef(test_pos_camera2flange, test_rot_camera2flange)

        # reference frame camera to base
        test_rf_camera2rf = np.dot(self.test_rf_flange2rf, test_rf_camera2flange)

        # reference frame laser to camera
        test_pos_laser2camera, test_rot_laser2camera = self.robot_module.rotPosRef(0, 151, 12, 0, 0, 0)
        self.test_rf_laser2camera = self.robot_module.createRef(test_pos_laser2camera, test_rot_laser2camera)

        # reference frame laser to flange
        self.test_rf_laser2flange = np.dot(test_rf_camera2flange, self.test_rf_laser2camera)
        test_rf_laser2rf = np.dot(test_rf_camera2rf, self.test_rf_laser2camera)

        self.test_rf_laser2rf = test_rf_laser2rf


    def test_target(self, target_01, target_02, theta_laser, backOx, alpha_rot_Oy):

        H_target01ToCamera = self.robot_module.createRef([target_01[0], target_01[1], target_01[2]],[0, 0, np.radians(theta_laser)])
        H_target02ToCamera = self.robot_module.createRef([target_02[0], target_02[1], target_02[2]],[0, 0, np.radians(theta_laser)])

        # R_target01ToCamera = self.robot_module.createRef([0, 0, 0], [rob.rotx(CFG.ROTATE_OX_LASER), rob.roty(alpha_rot_Oy), 0])
        # R_target02ToCamera = self.robot_module.createRef([0, 0, 0], [rob.rotx(CFG.ROTATE_OX_LASER), rob.roty(alpha_rot_Oy), 0])
        
        # pos01ToCamera, rot01ToCamera = self.robot_module.rotPos(H_target01ToCamera)
        # pos02ToCamera, rot02ToCamera = self.robot_module.rotPos(H_target02ToCamera)
        # print(f'pos01ToCamera:{pos01ToCamera}, {rot01ToCamera},\n pos02ToCamera:{pos02ToCamera}, {rot02ToCamera}\n')

        H_target01ToLaser = np.dot(np.linalg.inv(self.test_rf_laser2camera), H_target01ToCamera)
        H_target02ToLaser = np.dot(np.linalg.inv(self.test_rf_laser2camera), H_target02ToCamera)
        H_target01tobase = np.dot(self.test_rf_laser2rf, H_target01ToLaser)
        H_target02tobase = np.dot(self.test_rf_laser2rf, H_target02ToLaser)
        # posLaser01, rotLaser01 = self.robot_module.rotPos(H_target01tobase)
        # posLaser02, rotLaser02 = self.robot_module.rotPos(H_target02tobase)
        # print(f'newPos01:{posLaser01}, {rotLaser01}')
        
        def config_target(X, Y, theta_laser, backOx, alpha_rot_Oy):    
            L1 = np.abs(backOx)
            L2 = np.abs((np.tan(np.radians(CFG.ROTATE_OX_LASER)) * CFG.DISTANCE_LASER2OBJECT))
            delta_x = delta_y = 0
            theta_laser = abs(theta_laser)
            if theta_laser > 0:
                if alpha_rot_Oy < 0:
                    delta_x = L1 *np.cos(np.radians(theta_laser)) + L2 * np.sin(np.radians(theta_laser))
                    delta_y = L1 *np.sin(np.radians(theta_laser)) - L2 * np.cos(np.radians(theta_laser))
                    new_X = X - delta_x
                    new_Y = Y + delta_y

                elif alpha_rot_Oy > 0:
                    delta_x = L1 *np.cos(np.radians(theta_laser)) - L2 * np.sin(np.radians(theta_laser))
                    delta_y = L1 *np.sin(np.radians(theta_laser)) + L2 * np.cos(np.radians(theta_laser))
                    new_X = X + delta_x
                    new_Y = Y + delta_y

                else:
                    delta_x = L2 * np.cos(np.radians(theta_laser))
                    delta_y = L2 * np.sin(np.radians(theta_laser))
                    new_X = X - delta_x
                    new_Y = Y + delta_y
                
            elif theta_laser < 0:
                if alpha_rot_Oy < 0:
                    delta_x = L1 *np.cos(np.radians(theta_laser)) - L2 * np.sin(np.radians(theta_laser))
                    delta_y = L1 *np.sin(np.radians(theta_laser)) + L2 * np.cos(np.radians(theta_laser))
                    new_X = X - delta_x
                    new_Y = Y + delta_y

                elif alpha_rot_Oy > 0:
                    delta_x = L1 *np.cos(np.radians(theta_laser)) + L2 * np.sin(np.radians(theta_laser))
                    delta_y = L1 *np.sin(np.radians(theta_laser)) - L2 * np.cos(np.radians(theta_laser))
                    new_X = X + delta_x
                    new_Y = Y - delta_y

                else:
                    delta_x = L2 * np.cos(np.radians(theta_laser))
                    delta_y = L2 * np.sin(np.radians(theta_laser))
                    new_X = X + delta_x
                    new_Y = Y + delta_y
            else:
                delta_y = L2 * np.sin(np.radians(theta_laser))
                new_X = X
                new_Y = Y + delta_y
            
            return [new_X, new_Y]
        
        pos_targetToLaser01, rot_targetToLaser01 = self.robot_module.rotPos(H_target01ToLaser)
        pos_targetToLaser02, rot_targetToLaser02 = self.robot_module.rotPos(H_target02ToLaser)

        # calculate the new XY position of laser
        newXY_01 = config_target(pos_targetToLaser01[0], pos_targetToLaser01[1], theta_laser, backOx, alpha_rot_Oy)
        newXY_02 = config_target(pos_targetToLaser02[0], pos_targetToLaser02[1], theta_laser, backOx, alpha_rot_Oy)

        pos_targetToLaser01[0] = newXY_01[0]
        pos_targetToLaser01[1] = newXY_01[1]

        pos_targetToLaser02[0] = newXY_02[0]
        pos_targetToLaser02[1] = newXY_02[1]

        # create the H matrix about new laser position and new rotation
        H_newTargetToLaser01 = self.robot_module.createRef(pos_targetToLaser01, rot_targetToLaser01)
        H_newTargetToLaser02 = self.robot_module.createRef(pos_targetToLaser02, rot_targetToLaser02)

        refTarget = self.robot_module.createRef([0, 0, 0], [0, 0, 0])
        # R_refTarget = np.dot(np.dot(refTarget, roty(np.radians(alpha_rot_Oy))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))

        R_targetToLaser01 = np.dot(np.dot(H_newTargetToLaser01, roty(np.radians(alpha_rot_Oy))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))
        R_targetToLaser02 = np.dot(np.dot(H_newTargetToLaser02, roty(np.radians(alpha_rot_Oy))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))

        # create the H matrix new target to basec
        H_newTarget01tobase = np.dot(self.test_rf_laser2rf, R_targetToLaser01)
        H_newTarget02tobase = np.dot(self.test_rf_laser2rf, R_targetToLaser02)

        # new flange position
        H_flangeToBase01 = np.dot(H_newTarget01tobase, np.linalg.inv(self.test_rf_laser2flange))
        H_flangeToBase02 = np.dot(H_newTarget02tobase, np.linalg.inv(self.test_rf_laser2flange))
        # print(f'rf_laser2flange:{self.rf_laser2flange}')
        newFlangePos01, newFlangeRot01 = self.robot_module.rotPos(H_flangeToBase01)
        newFlangePos02, newFlangeRot02 = self.robot_module.rotPos(H_flangeToBase02)
        print(f'newFlangePos01:{newFlangePos01}, {newFlangeRot01},\n newFlangePos02:{newFlangePos02}, {newFlangeRot02}\n')

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
        # print(f'posTarget01toLaser, posTarget02toLaser:{posTarget01toLaser}, {posTarget02toLaser}')

        target01ToRf = np.dot(self.rf_laser2rf, target01toLaser)
        target02ToRf = np.dot(self.rf_laser2rf, target02toLaser)

        pos1toRf, _ = self.robot_module.rotPos(target01ToRf)
        pos2toRf, _ = self.robot_module.rotPos(target02ToRf)
        print(f'pos1toRf, pos2toRf:{pos1toRf}, {pos2toRf}')

        pos2ToPos1_rf = pos2toRf - pos1toRf
        pos_target02_oy_rf = sqrt(pos2ToPos1_rf[0] ** 2 + pos2ToPos1_rf[1] ** 2)

        rfToBase = self.robot_module.createRef([0, 0, 0], [np.radians(180), 0, 0])
        target01ToBase = np.dot(rfToBase, target01ToRf)

        change_OX = CFG.DISTANCE_LASER2OBJECT * tan(np.radians(CFG.ROTATE_OX_LASER))
        print("changeOx:", change_OX)
  
        newPos2 = np.array([0, -pos_target02_oy_rf + change_OX, 0])
        newRef = target01ToBase
        posRef, rotRef = self.robot_module.rotPos(newRef)
        newRef_nonMat = np.concatenate((posRef, rotRef), axis=0)
        setRef = TxyzRxyz_2_Pose(newRef_nonMat)
        self.robot.setPoseFrame(setRef)

        
        target01toNewRf = np.dot(target01ToBase, np.linalg.inv(target01ToBase))
        pos01toNewRf, rot01ToNewRf = self.robot_module.rotPos(target01toNewRf)
        target02toNewRf = self.robot_module.createRef(newPos2, rot01ToNewRf)
        pos02toNewRf, rot02ToNewRf = self.robot_module.rotPos(target02toNewRf)

        # pos01toNewRf = np.array([backOx, change_OX, 0])
        # rot01ToNewRf = np.array([0, 0, 0])
        # pos02toNewRf = np.array([backOx, -pos_target02_oy_rf + change_OX, 0])
        # rot02ToNewRf = np.array([0, 0, 0])
        # # pos01toNewRf[0] += backOx
        # # pos01toNewRf[1] = change_OX
        # # pos02toNewRf[0] += backOx
        # target01toNewRf = self.robot_module.createRef(pos01toNewRf, rot01ToNewRf)
        # target02toNewRf = self.robot_module.createRef(pos02toNewRf, rot02ToNewRf)

        if alpha != 0:
            newT1toNewRf = np.dot(np.dot(target01toNewRf, roty(np.radians(alpha))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))
            newT2toNewRf = np.dot(np.dot(target02toNewRf, roty(np.radians(alpha))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))
        else:
            newT1toNewRf = np.dot(np.dot(target01toNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx
            newT2toNewRf = np.dot(np.dot(target02toNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx

        newPos1ToNewRf, newRot1ToNewRf = self.robot_module.rotPos(newT1toNewRf)
        newPos2ToNewRf, newRot2ToNewRf = self.robot_module.rotPos(newT2toNewRf)
        print(f'backOx:{backOx}')

        newPos1ToNewRf[0] += backOx
        newPos2ToNewRf[0] += backOx
        print(f'newPos1ToNewRf, newRot1ToNewRf:{newPos1ToNewRf}, {newRot1ToNewRf}\n newPos2ToNewRf, newRot2ToNewRf:{newPos2ToNewRf}, {newRot2ToNewRf}')
        
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

        return target01, target02, pos_target02_oy_rf

    def homePos(self, linearSpeed, joinSpeed):
        self.setRobot(linearSpeed, joinSpeed)
        self.fixedRef(0)
        self.robot.MoveJ(self.rf_laser2rf_matrix)

    def getTarget(self, pixel, shape, _suf_: int):
        # target01, target02, theta_laser = None, None, None
        obj = VisionRobot()
        obj.test_fixedRef()
        testTarget01, testTarget02, angleLaserToObject = None, None, None
        angleLaserToObject = self.robot_module.rotLaser(pixel)

        coorPixel1 = pixel[0]
        coorPixel2 = pixel[1]
        print(f"coordinate_pixel:{pixel}")

        #### fix value
        self.disCameraToObject = CFG.DISTANCE_CAMERA2OBJECT
        self.pixelFocalLength = CFG.FOCAL_LENGTH * 1000 / CFG.PIXEL_SIZE
        ####

        xToCamera01, yToCamera01 = self.robot_module.convertCoordinates(CFG.RESOLUTION_X, CFG.RESOLUTION_Y, coorPixel1[0], coorPixel1[1],\
                                                                        self.pixelFocalLength, self.disCameraToObject, theta=180,)  # cfg

        xToCamera02, yToCamera02 = self.robot_module.convertCoordinates(CFG.RESOLUTION_X, CFG.RESOLUTION_Y, coorPixel2[0], coorPixel2[1],\
                                                                        self.pixelFocalLength, self.disCameraToObject, theta=180,)  # cfg
        
        print("xToCamera01, yToCamera01:", xToCamera01, yToCamera01)
        print("xToCamera02, yToCamera02:", xToCamera02, yToCamera02)

        zLaserToObject = CFG.HOME_LASER - CFG.DISTANCE_LASER2OBJECT
        print(f"zLaserToObject:{zLaserToObject}")
        if zLaserToObject > CFG.SAFE_DISTANCE:
            zLaserToObject = 330
            print("Warning the collision. Check the distance")
        realTarget01 = np.array([xToCamera01, yToCamera01, zLaserToObject])
        realTarget02 = np.array([xToCamera02, yToCamera02, zLaserToObject])

        if shape == "90_degree":
            alpha = CFG.ROTATE_OY_LASER * _suf_
            print(f"alpha:{alpha}")
            if alpha < 0:
                backOx = (-np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.DISTANCE_LASER2OBJECT)
                testTarget01, testTarget02, test_length_weld = self.newcreatePoint(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)
                obj.test_target(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)
            else:
                backOx = (np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.DISTANCE_LASER2OBJECT)
                testTarget01, testTarget02, test_length_weld = self.newcreatePoint(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)
                obj.test_target(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)

        elif shape == "30_degree":
            alpha = CFG.ROTATE_OY_LASER * _suf_
            print(f"alpha:{alpha}")
            if alpha < 0:
                backOx = (-np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.DISTANCE_LASER2OBJECT)  #### need more condition
                testTarget01, testTarget02, test_length_weld = self.newcreatePoint(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)
                obj.test_target(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)
            else:
                backOx = (np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.DISTANCE_LASER2OBJECT)  #### need more condition
                testTarget01, testTarget02, test_length_weld = self.newcreatePoint(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)
                obj.test_target(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)

        elif shape == "0_degree":
            alpha = backOx = 0
            testTarget01, testTarget02, test_length_weld = self.newcreatePoint(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)

            obj.test_target(realTarget01, realTarget02, angleLaserToObject, backOx, alpha)

        else:
            stop()
    
        return testTarget01, testTarget02, angleLaserToObject , test_length_weld

    def runMoveL(self, target, speedScan):
        self.setRobot(speedScan, CFG.JOINT_SPEEDS[0])
        self.robot.MoveL(target)
        return True

    def runMoveJ(self, target):
        self.setRobot(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
        self.robot.MoveJ(target)
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

if __name__ == "__main__":
    print("Done")

