import sys

sys.path.append("E:\Quan\AutoRoboticInspection-V1\VIKO_UltraRobot")
# sys.path.append("E:\Project\Robot-6DOF\VIKO_UltraRobot")
from robodk.robolink import *
from robodk.robomath import *
import numpy as np
import time
from library import vision_robotic as vis
from library import robot_lib_modify as rob
from datetime import datetime, timedelta
from config import config as CFG
from IPCDataMs import IPCData
import subprocess
import signal
from queue import Queue

import mmap
import struct
import sys
import json


IDProcessLaser = None

def LaserTrigger(action):
    global IDProcessLaser
    if action == "start":
        if IDProcessLaser and IDProcessLaser.poll() is None:  # Check if process is still running
            print(f"Laser process already running with PID: {IDProcessLaser.pid}")
        else:
            try:
                IDProcessLaser = subprocess.Popen([CFG.PATH_LASER_PROGRAM])
                print(f"Started new laser process with PID: {IDProcessLaser.pid}")
            except Exception as e:
                print(f"Error starting laser process: {e}")
    elif action == "stop":
        if IDProcessLaser and IDProcessLaser.poll() is None:  # Check if process is still running
            try:
                print(f"Stopping laser process with PID: {IDProcessLaser.pid}")
                IDProcessLaser.terminate()
                IDProcessLaser.wait()  # Wait for the process to terminate
            except Exception as e:
                print(f"Error stopping laser process: {e}")
        else:
            print("No laser process is running.")
    else:
        print("Invalid action. Use 'start' or 'stop'.")

def save_lidar_data(lidar_data, filename):
    with open(filename, 'w') as file:
        for point in lidar_data:
            x, z = point
            file.write(f"{x}, {z}\n")

def scan_data(length_weld, result_queue):
    end_time = datetime.now() + timedelta(seconds=2)
    LIDAR_data = None 
    while datetime.now() < end_time:                  
        IPCData.sendLIDARcs(length_weld)
        time.sleep(0.1)
    
    IPCData.sendLIDARcs(0)
    start_time2 = datetime.now()
    end_time2 = start_time2 + timedelta(seconds=100)
    while datetime.now() < end_time2:
        temp, LIDAR_data = IPCData.get_lidar_data()
        if temp != 0:
            print('Exit')
            break
        time.sleep(0.1)
    
    lidar_data = np.array(LIDAR_data)
    # current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # save_lidar_data(lidar_data, f'laser/experimental/Lidar_data_{current_time}.txt')

    result_queue.put(lidar_data)
    
def run(coordinate_pixel_list, model_weld_list, _suf_, pos_status="home"):
    
    laser_data = None
    
    VisRob = VisionRobot()      
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
    if pos_status == "home":
        VisRob.fixedRef()
    else:
        stop()

    for index, coordinate_pixel in enumerate(coordinate_pixel_list):
  
        LaserTrigger("start")
        result_queue = Queue()
        target01, target02, thetaLaser, lengthWeld = VisRob.getTarget(coordinate_pixel, model_weld_list[index], _suf_)
        time_scan = lengthWeld / CFG.RESOLUTION_Y_LASER / CFG.FREQUENCY
        speedScan = lengthWeld / time_scan
        print('The information robotic laser')
        print('Speed scan: ', speedScan)
        print('Length planning ', lengthWeld)
        print('Time scan: ', time_scan)
  
        startWeld = VisRob.runMoveJ(target01)

        if startWeld:            
            scan_thread = threading.Thread(target=scan_data, args= (int(round(lengthWeld)), result_queue))
            scan_thread.start()
            time.sleep(1)
            VisRob.runMoveL(target02, speedScan)            
            scan_thread.join()
            if not result_queue.empty():
                laser_data = result_queue.get()
                print("Laser data received:", laser_data)
            else:
                print("No laser data received.")            
        else:
            print("Error: Failed to reach target01")  
        LaserTrigger("stop")
        VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])
    print("Scan data done!")   
    data_export = {
        "id": str(datetime.now()),
        "distance_camera2object": CFG.DISTANCE_CAMERA2OBJECT,
        "lengthWeld": lengthWeld,
        "thetaLaser": thetaLaser,
    }

    rob.RobotModule.export_csv(data_export) 
    
    return laser_data

def getCoordinates(model, image):
    return vis.getCoordinates(model, image, True)

def movHome():
    VisRob = VisionRobot()
    VisRob.homePos(CFG.LINEAR_SPEEDS[0], CFG.JOINT_SPEEDS[1])

def stop():
    VisRob = VisionRobot()
    VisRob.disConnectRobot()


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

    def fixedRef(self):
        """
        Fixed refer
        """

        # reference frame flange to base
        pos_flange2rf, rot_flange2rf = self.robot_module.rotPosRef(380, 0, -360, 0, 0, 0)
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

        # reference frame flange to base (home position)
        test_pos_flange2rf, test_rot_flange2rf = self.robot_module.rotPosRef(380, 0, 360, 180, 0, 0)
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
        self.test_rf_laser2rf = np.dot(test_rf_camera2rf, self.test_rf_laser2camera)


    def test_target(self, target_01, target_02, theta_laser, backOx, alpha_rot_Oy):

        H_target01ToCamera = self.robot_module.createRef([target_01[0], target_01[1], target_01[2]],[0, 0, np.radians(theta_laser)])
        H_target02ToCamera = self.robot_module.createRef([target_02[0], target_02[1], target_02[2]],[0, 0, np.radians(theta_laser)])

        H_target01ToLaser = np.dot(np.linalg.inv(self.test_rf_laser2camera), H_target01ToCamera)
        H_target02ToLaser = np.dot(np.linalg.inv(self.test_rf_laser2camera), H_target02ToCamera)
        
        pos_targetToLaser01, rot_targetToLaser01 = self.robot_module.rotPos(H_target01ToLaser)
        pos_targetToLaser02, rot_targetToLaser02 = self.robot_module.rotPos(H_target02ToLaser)

        # create the H matrix about new laser position and new rotation
        H_newTargetToLaser01 = self.robot_module.createRef(pos_targetToLaser01, rot_targetToLaser01)
        H_newTargetToLaser02 = self.robot_module.createRef(pos_targetToLaser02, rot_targetToLaser02)

        R_targetToLaser01 = np.dot(np.dot(H_newTargetToLaser01, roty(np.radians(alpha_rot_Oy))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))
        R_targetToLaser02 = np.dot(np.dot(H_newTargetToLaser02, roty(np.radians(alpha_rot_Oy))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))

        # create the H matrix new target to base
        H_newTarget01tobase = np.dot(self.test_rf_laser2rf, R_targetToLaser01)
        H_newTarget02tobase = np.dot(self.test_rf_laser2rf, R_targetToLaser02)

        # new flange position
        H_flangeToBase01 = np.dot(H_newTarget01tobase, np.linalg.inv(self.test_rf_laser2flange))
        H_flangeToBase02 = np.dot(H_newTarget02tobase, np.linalg.inv(self.test_rf_laser2flange))
        # print(f'rf_laser2flange:{self.rf_laser2flange}')
        newFlangePos01, newFlangeRot01 = self.robot_module.rotPos(H_flangeToBase01)
        newFlangePos02, newFlangeRot02 = self.robot_module.rotPos(H_flangeToBase02)
        # print(f'newFlangePos01:{newFlangePos01}, {newFlangeRot01},\n newFlangePos02:{newFlangePos02}, {newFlangeRot02}\n')

        newPos01, newRot01 = self.robot_module.rotPos(H_newTarget01tobase)
        newPos02, newRot02 = self.robot_module.rotPos(H_newTarget02tobase)
        # print(f'T01:{newPos01}, {newRot01},\n T02:{newPos02}, {newRot02}\n')

        return [newFlangePos01, newFlangeRot01], [newFlangePos02, newFlangeRot02]
        
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
        # print(f'pos1toRf, pos2toRf:{pos1toRf}, {pos2toRf}')

        pos2ToPos1_rf = pos2toRf - pos1toRf
        pos_target02_oy_rf = sqrt(pos2ToPos1_rf[0] ** 2 + pos2ToPos1_rf[1] ** 2)

        rfToBase = self.robot_module.createRef([0, 0, 0], [np.radians(180), 0, 0])
        target01ToBase = np.dot(rfToBase, target01ToRf)

        change_OX = CFG.DISTANCE_LASER2OBJECT * tan(np.radians(CFG.ROTATE_OX_LASER))
  
        newPos2 = np.array([0, -pos_target02_oy_rf + change_OX, 0])
        newRef = target01ToBase
        posRef, rotRef = self.robot_module.rotPos(newRef)
        newRef_nonMat = np.concatenate((posRef, rotRef), axis=0)
        setRef = TxyzRxyz_2_Pose(newRef_nonMat)
        self.robot.setPoseFrame(setRef)
        
        target01toNewRf = np.dot(target01ToBase, np.linalg.inv(target01ToBase))
        _, rot01ToNewRf = self.robot_module.rotPos(target01toNewRf)
        target02toNewRf = self.robot_module.createRef(newPos2, rot01ToNewRf)

        if alpha != 0:
            newT1toNewRf = np.dot(np.dot(target01toNewRf, roty(np.radians(alpha))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))
            newT2toNewRf = np.dot(np.dot(target02toNewRf, roty(np.radians(alpha))), rotx(np.radians(-CFG.ROTATE_OX_LASER)))
        else:
            newT1toNewRf = np.dot(np.dot(target01toNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx
            newT2toNewRf = np.dot(np.dot(target02toNewRf, roty(np.radians(alpha))), rotx(np.radians(0)))  ## not config rotx

        newPos1ToNewRf, newRot1ToNewRf = self.robot_module.rotPos(newT1toNewRf)
        newPos2ToNewRf, newRot2ToNewRf = self.robot_module.rotPos(newT2toNewRf)

        newPos1ToNewRf[0] += backOx
        newPos2ToNewRf[0] += backOx

        if alpha != 0:
            newPos1ToNewRf[1] += 0
            newPos2ToNewRf[1] += 0
        else:
            pass

        max_lenght = 202
        delta_error = 1

        while pos_target02_oy_rf > max_lenght:
            print(f"check_length:{pos_target02_oy_rf}")
            check_error = pos_target02_oy_rf - max_lenght
            if check_error > 2:
                newPos1ToNewRf[1] -= delta_error
                pos_target02_oy_rf -= delta_error
            else:
                break

        target01_none_mat = np.concatenate((newPos1ToNewRf, newRot1ToNewRf), axis=0)
        target02_none_mat = np.concatenate((newPos2ToNewRf, newRot2ToNewRf), axis=0)
        target01 = TxyzRxyz_2_Pose(target01_none_mat)
        target02 = TxyzRxyz_2_Pose(target02_none_mat)

        return target01, target02, pos_target02_oy_rf

    def homePos(self, linearSpeed, joinSpeed):
        self.setRobot(linearSpeed, joinSpeed)
        self.fixedRef()
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
        
        T1T2_camera = np.sqrt((xToCamera01 - xToCamera02) ** 2 + (yToCamera01 - yToCamera02) ** 2)
        print(f"xToCamera01, yToCamera01:{xToCamera01}, {yToCamera01}")
        zLaserToObject = CFG.HOME_LASER - CFG.DISTANCE_LASER2OBJECT
        print(f"zLaserToObject:{zLaserToObject}")
        if zLaserToObject > CFG.SAFE_DISTANCE:
            zLaserToObject = 330
            print("Warning the collision. Check the distance")
        target01ToCamera = np.array([xToCamera01, yToCamera01, zLaserToObject])
        target02ToCamera = np.array([xToCamera02, yToCamera02, zLaserToObject])

        if shape == "90_degree":
            alpha = CFG.ROTATE_OY_LASER * _suf_
            print(f"alpha:{alpha}")
            if alpha < 0:
                backOx = (-np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.DISTANCE_LASER2OBJECT)
                testTarget01, testTarget02, test_length_weld = self.newcreatePoint(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)
                T1, T2  = obj.test_target(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)
            else:
                backOx = (np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.DISTANCE_LASER2OBJECT)
                testTarget01, testTarget02, test_length_weld = self.newcreatePoint(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)
                T1, T2  = obj.test_target(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)

        elif shape == "30_degree":
            alpha = CFG.ROTATE_OY_LASER * _suf_
            print(f"alpha:{alpha}")
            if alpha < 0:
                backOx = (-np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.DISTANCE_LASER2OBJECT)  #### need more condition
                testTarget01, testTarget02, test_length_weld = self.newcreatePoint(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)
                T1, T2  = obj.test_target(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)
            else:
                backOx = (np.tan(np.radians(CFG.ROTATE_OY_LASER)) * CFG.DISTANCE_LASER2OBJECT)  #### need more condition
                testTarget01, testTarget02, test_length_weld = self.newcreatePoint(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)
                T1, T2  = obj.test_target(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)

        elif shape == "0_degree":
            alpha = backOx = 0
            testTarget01, testTarget02, test_length_weld = self.newcreatePoint(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)

            T1, T2  =  obj.test_target(target01ToCamera, target02ToCamera, angleLaserToObject, backOx, alpha)
            # print(f"T1:{T1}, T2:{T2}")

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

