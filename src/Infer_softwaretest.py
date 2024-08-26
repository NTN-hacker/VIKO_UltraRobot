import sys
sys.path.append(
    "E:\\Quan\\AutoRoboticInspection-V1\VIKO_UltraRobot"
)
import numpy as np
import cv2
import threading
import time
from pypylon import pylon
from ultralytics import YOLO
from datetime import datetime

from library.experimental_src.laser import Laser
from config import config as CFG
from src import robotic_modify as rm
from library import viko_lib as vl
from IPCDataMs import IPCData

global status
global idx, idx_Coord, idx_below_frame,idx_rosposition
global image, laser_img, result_image
global running

class CameraPanel():
    
    def __init__(self):
         
        # Init the global variables
        global idx, idx_Coord, idx_Chart, idx_below_frame, idx_rosposition
        idx, idx_Coord, idx_Chart, idx_below_frame, idx_rosposition = 0, 0, 0, 0, 0

        global status
        status = None

        global image, laser_img, result_image
        image, laser_img, result_image = None, None, None

        global running 
        running = True

        # Init the IPC
        self.ipc_data = IPCData()

        # Init the robot                
        self.pos_status = 'home' 
        self._suf_left_ = 1
        self._suf_right_ = -1
        
        # Init the laser
        self.laser_data = None

        # Init the trigger signal
        running = True  # trigger robot
        self.response_result = False # trigger button
        self.data_laser = False #trigger view 3d from laser data
        self.rosposition = False # trigger the position of the robot

        # Init the model  
        self.model = YOLO(CFG.MODEL['YOLOV9']['WEIGHT'], task= 'segment') # Model for weld identification
        self.model_inspection = YOLO(CFG.MODEL['INSPECTION']['WEIGHT'], task= 'segment') # Model for inspection

        # Init the threading
        self.lock = threading.Lock()
        
        # Init the camera
        self.h, self.w = 640, 640
        self.frame_count = 0
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        for device in devices:
            print(device.GetModelName(), device.GetSerialNumber())
            self.model_name = device.GetModelName()
        self.camera = pylon.InstantCamera()
        self.camera.Attach(tl_factory.CreateDevice(devices[0]))

        # Start the check button thread
        self.button_thread = threading.Thread(target=self.check_buttons)
        self.button_thread.daemon = True
        self.button_thread.start()

        
    
    def setting_camera(self):
        # Setting some parameters for camera
        self.camera.Open()
        self.camera.GainAuto.SetValue("Off")  
        self.camera.Gain.SetValue(5.0)
        self.camera.ExposureTime.SetValue(2000.0)
        # Check if camera is already grabbing
        if not self.camera.IsGrabbing():
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        return converter

    def main(self):
        global image, result_image
        global status
        global idx, idx_rosposition   
  
        try: 
            # Khởi tạo chương trình thì sẽ chạy running = True
            while running:
                # Init the converter
                converter = self.setting_camera()
                # get image and send data
                while self.camera.IsGrabbing() and running:
                    with self.lock:
                        grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                        if grabResult.GrabSucceeded():
                            # Access the image data
                            image1 = converter.Convert(grabResult)
                            image = image1.GetArray()

                            # get image from camera and send image
                            img = cv2.resize(image, (self.h, self.w), cv2.INTER_CUBIC)
                            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            image = img.copy()
                            img_np = np.frombuffer(image, dtype=np.uint8).reshape((640, 640, 3))
                            self.ipc_data.send_frame(img_np)
                            self.frame_count += 1
                            time.sleep(0.01)    # 10 ms sẽ update ảnh một lần
                            
                            if self.response_result:
                                # send result image
                                result_image = cv2.resize(result_image, (640, 640), cv2.INTER_CUBIC)
                                result_image_np = np.frombuffer(result_image, dtype=np.uint8).reshape((640, 640, 3))
                                self.ipc_data.send_frame_below(idx_below_frame, result_image_np)  

                                # send status
                                self.ipc_data.send_status(idx,f'Machine Vision: {status}' )   #dict {'index': 'context}
                                
                                # send laser data to view 3D
                                if self.data_laser:
                                    data = []
                                    data = self.laser_data.copy()
                                    print('Data sent to view 3D ', self.laser_data)
                                    self.ipc_data.send_3Ddata(idx_Coord, data.shape[0], data)
                                    self.data_laser = False  
                            self.response_result == False                      
                    grabResult.Release()
        except Exception as e:
            idx += 1
            status = f"Error: {str(e)}. Please restart the program."
            self.camera.StopGrabbing()
            self.camera.Close()

    def check_buttons(self):
        start = [-1,-1,-1,-1,-1,-1]
        
        while running:
            trigger_list = self.ipc_data.get_data()
            current_time = datetime.now()

            for idx, trigger_value in enumerate(trigger_list):
                if trigger_value == 1:
                    if start[idx] == -1:
                        start[idx] = current_time
                        self.process_button(idx)
                    elif (current_time - start[idx]).total_seconds() > 5:
                        start[idx] = current_time
                        self.process_button(idx) 
                    print ("Button: ", idx)
            time.sleep(33/1000)  
        self.button_thread.join()
            
    def process_button(self, button_idx):       

        global result_image, laser_img, image
        global idx, idx_Coord, idx_Chart, idx_below_frame, idx_rosposition 
        global status       
        global running
    
        self.result_image = None
        if  button_idx == 0:
            idx += 1
            status = "Skip"

        elif button_idx == 1:

            # Estimate the planning weld for robot to sample
            idx+=1
            status = "Planning weld..."        
            coordinate_pixel_list, model_weld_list, planning_img = rm.getCoordinates(self.model, image)
            self.response_result = True
            idx_below_frame += 1
            result_image = planning_img.copy()           

            # Robot move to sample to laser scan sample
            idx+=1
            status = "Moving..."
            self.laser_data = rm.run(coordinate_pixel_list, model_weld_list, self._suf_left_, self.pos_status) # default _suf_left_ and will update _suf_right_ with another sample
            idx+=1
            status = "Robot Completed."
            
            # Laser data post-processing and Inspection the laser image
            idx+=1
            status = "Inspection Processing..."
            self.laser_data, laser_img = vl.convert_to_grayscale_image(self.laser_data)         
            inspection_result = vl.run_inspection(self.model_inspection, laser_img, 0.2)
            idx_below_frame+= 1
            result_image = inspection_result.copy()
            idx+=1
            status = "Inspection Completed." # will save 3D file .dat

            # View 3D from laser data
            idx+=1
            status = "Start 3D View..."
            idx_Coord += 1
            self.data_laser = True 
            idx+=1
            status = "3D View Completed."

            # Done
            idx+=1
            status = "Finished."

        elif button_idx == 2:
            idx+=1
            status = "Skip"

        elif button_idx == 3: 
            idx+=1
            status = "Skip"

        elif button_idx == 4:
            idx+=1
            status = "Skip"

        elif button_idx == 5:
            idx_rosposition += 1
            self.rosposition = True
            print(idx_rosposition)   



if __name__ == "__main__":
    InpectionBackend = CameraPanel()
    InpectionBackend.main()



