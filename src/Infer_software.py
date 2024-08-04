import sys
import io
sys.path.append(
    "E:\\Quan\\AutoRoboticInspection-V1\VIKO_UltraRobot"
)
# sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
# sys.stdin = io.TextIOWrapper(sys.stdin.buffer,'utf-8')

import wx
import numpy as np
import cv2
import threading
import time
from pypylon import pylon
from ultralytics import YOLO
from datetime import datetime
from laser import Laser

from config import config as CFG
from src import robotic_modify as rm
from library import viko_lib as vl
import subprocess
import os
import signal
from IPCDataMs import IPCData

global IDProcessLaser
global Inpection_Dict

def LaserTrigger():
    global IDProcessLaser
    try:
        if IDProcessLaser.poll() is None:  # Check if process is still running
            os.kill(IDProcessLaser.pid, signal.SIGTERM)  
    except:
        print ("Nothing")
    IDProcessLaser = subprocess.Popen([CFG.PATH_LASER_PROGRAM])

def load_model(weight):
    model = YOLO(weight, task= 'detect')
    return model

def run_inspection(model, img):
    img = cv2.resize(img, (640, 640), interpolation=cv2.INTER_CUBIC)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = model.predict(source=img, conf=0.65) #CFG.CONF_ACC    
    plot = results[0].plot()
    return plot

def get_coordinate(model, img):
    return rm.getCoordinates(model, img) 

def run_robot(coordinate_pixel_list, model_weld_list, _suf_, pos_status):
    rm.run(coordinate_pixel_list, model_weld_list, _suf_,  pos_status)

def stop_robot():
    print('Stop.')
    rm.stop()

def move_center():
    print('Move center.')
    rm.movHome()

def inspection(model, image):
    global Inpection_Dict
    print('Inpection Result')
    current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    img_cvt = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    dict_re = model.predict(img_cvt)
    result_image = cv2.resize(dict_re[0].plot(), (640, 640), cv2.INTER_CUBIC)
    cv2.imwrite(f'laser/inspection/inspection_{current_time}.png', result_image)
    
    out_labels = vl.getLabels(dict_re)

    ref_label = [CFG.MODEL_INSPECTION[label] for label in out_labels]

    defects = np.array(ref_label)
    Inpection_Dict =  vl.count_defects(defects)

    total = sum(Inpection_Dict.values())
    for key in Inpection_Dict:
        Inpection_Dict[key] = (Inpection_Dict[key] / total) * 100

    return result_image


global status, idx, idx_Coord, status_time, result_image, idx_below_frame
global image

class CameraPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)

        #Connect laser
        self.laser = Laser()
        self.laser.connect()

        #IPC Time
        # Create IPCData object

        global idx, idx_Coord, idx_Chart, idx_below_frame
        idx = 0
        idx_Coord = 0
        idx_Chart = 0
        idx_below_frame = 0 
        global status
        status = None

        global image 
        image = None

        self.ipc_data = IPCData()

        self.h, self.w = 640, 640
        self.robot_running = False  #trigger robot start
        self.pos_status = 'home' 
        self._suf_left_ = 1
        self._suf_right_ = -1
        self.result_image = None
        self.response_result_laser = False
        self.running = True  
        self.response_result = False # trigger button
        self.data_laser = False
        self.data_inspection = False
        self.lock = threading.Lock()
        self.frame_count = 0
        # model   
        self.model = YOLO(CFG.MODEL['YOLOV9']['WEIGHT'], task= 'segment')


        # model inspection
        self.model_inspection = load_model(CFG.MODEL['INSPECTION']['WEIGHT'])

        self.bitmap = wx.Bitmap(self.w, self.h)
        self.SetDoubleBuffered(True)
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        for device in devices:
            print(device.GetModelName(), device.GetSerialNumber())
            self.model_name = device.GetModelName()
        
        self.camera = pylon.InstantCamera()
        self.camera.Attach(tl_factory.CreateDevice(devices[0]))

        self.camera_thread = threading.Thread(target=self.update_camera) #setter
        self.button_thread = threading.Thread(target=self.check_buttons) #getter
        # self.laser_thread  = threading.Thread(target=self.scan_laser)

        self.camera_thread.daemon = True
        self.camera_thread.start()

        self.button_thread.daemon = True
        self.button_thread.start()

        # self.laser_thread.daemon = True
        # self.laser_thread.start()

    def update_image(self, img):
        if self:                          
            img = cv2.resize(img, (self.h, self.w), cv2.INTER_CUBIC)
            buf = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).tobytes()
            self.bitmap.CopyFromBuffer(buf)
            self.Refresh()

    def scan_laser(self):
        while self.running:
            try:
                with open('trigger.txt', 'r') as file:
                    content = file.read().strip() 
                if content == '1':
                    # sleep to start scan
                    #time.sleep(1)
                    global result_image, idx_below_frame
                    result_image_ = self.laser.start_scan()
                    idx_below_frame += 1
                    self.response_result == True
                    result_image = result_image_.copy()

            except FileNotFoundError:
                print("File 'trigger.txt' not found.")
            except Exception as e:
                print(f"Error occurred: {e}")

    def update_camera(self):
        global image
        while self.running:
            # setting
            self.camera.Open()
            self.camera.GainAuto.SetValue("Off")  
            self.camera.Gain.SetValue(5.0)
            self.camera.ExposureTime.SetValue(2000.0)
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            converter = pylon.ImageFormatConverter()
            converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            # get image and send data
            while self.camera.IsGrabbing() and self.running:
                with self.lock:
                    grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                    if grabResult.GrabSucceeded():
                        # Access the image data
                        image1 = converter.Convert(grabResult)
                        image = image1.GetArray()

                        #### get image from camera and send image
                        img = cv2.resize(image, (self.h, self.w), cv2.INTER_CUBIC)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        image = img.copy()
                        img_np = np.frombuffer(image, dtype=np.uint8).reshape((640, 640, 3))
                        self.ipc_data.send_frame(img_np)
                        self.frame_count += 1
                        time.sleep(1/35)      
                        wx.CallAfter(self.update_image, image)  
                        
                        global result_image 
                        if self.response_result:
                            #### send result image
                            result_image_np = np.frombuffer(result_image, dtype=np.uint8).reshape((640, 640, 3))
                            self.ipc_data.send_frame_below(idx_below_frame, result_image_np)  

                            #send status
                            self.ipc_data.send_status(idx,f'Machine Vision: {status}' )   #dict {'index': 'context}
                            
                            #send data
                            if self.data_laser:
                                data = []
                                filepath = 'laser/datascan/datascan.txt'
                                with open(filepath, 'r') as file:
                                    for line in file:
                                        row = list(map(float, line.split()))
                                        data.append(row)
                                        
                                print("3D shared memory is updated")
                                self.ipc_data.send_3Ddata(idx_Coord,3072,data)
                                self.data_laser = False                            

                            #send the information inspection
                            if self.data_inspection:
                                global Inpection_Dict
                                self.ipc_data.send_chart(idx_Chart, Inpection_Dict)
                                self.data_inspection = False

                            self.response_result == False                               
                        
                    grabResult.Release()
            self.camera.StopGrabbing()
            self.camera.Close()
    def stop_camera(self):
        self.running = False  
        if self.camera_thread.is_alive():
            self.camera_thread.join()  
    def on_close(self, event):
        self.stop_camera()
        self.Destroy()

    def check_buttons(self):
        start = [-1,-1,-1,-1,-1,-1]
        while self.running:
            trigger_list = self.ipc_data.get_data()
            current_time = datetime.now()

            for idx, trigger_value in enumerate(trigger_list):
                if trigger_value == 1:
                    if start[idx] == -1:
                        start[idx] = current_time
                        self.process_button(idx)
                    elif (current_time - start[idx]).total_seconds() > 7:
                        start[idx] = current_time
                        self.process_button(idx) 
                    print ("Button: ", idx)
            time.sleep(33/1000)  
            
    def process_button(self, button_idx):
        global result_image
        global idx, idx_Coord, idx_Chart, idx_below_frame
        global status
        global image
        global Inpection_Dict
    
        self.result_image = None
        if  button_idx == 0:
            status = "Processing" 
            idx+=1
            idx_below_frame+= 1                      
            result_image = run_inspection(self.model, image)
            if not isinstance(result_image, np.ndarray):
                status = 'No image'
                print(status)
                return
            idx+=1
            status = "Detected"    
            self.response_result = True
        elif button_idx == 1:
            idx+=1
            status = "Planning weld..." 
            self.response_result = True
            coordinate_pixel_list, model_weld_list, planning_img = get_coordinate(model = self.model, img = image)
            idx_below_frame += 1
            result_image = planning_img.copy()           

            idx+=1
            status = "Moving..."
            run_robot(coordinate_pixel_list, model_weld_list, self._suf_left_, self.pos_status)
                        
            idx+=1
            idx_Coord+=1
            status = "Completed."
            self.data_laser = True 
        elif button_idx == 2:
            idx+=1
            status = "Planning weld..." 
            self.response_result = True
            coordinate_pixel_list, model_weld_list, planning_img = get_coordinate(model = self.model, img = image)
            idx_below_frame += 1
            result_image = planning_img.copy()
            
            idx+=1
            status = "Moving..."
            run_robot(coordinate_pixel_list, model_weld_list, self._suf_right_, self.pos_status)
                        
            idx+=1    
            idx_Coord+=1
            status = "Completed."        
            self.data_laser = True 
        elif button_idx == 3: 
            idx+=1
            status = "Inpection Processing."

            path_img = 'laser/depthmap/datascan-python.png'
            image = cv2.imread(path_img, cv2.IMREAD_COLOR)
            idx_below_frame+= 1
            result_image = inspection(self.model_inspection, image)

            idx_Chart += 1
            self.data_inspection = True 

        elif button_idx == 4:
            idx+=1
            status = "Back Home."
            move_center()   

        time.sleep(2)    

    def stop_threads(self):
        self.running = False
        self.camera_thread.join()
        self.button_thread.join()
        self.laser.disconnect()

class InspectionFrame(wx.Frame):
    def __init__(self, parent, title):
        super(InspectionFrame, self).__init__(parent, title=title, size=(1400, 700))  # Tăng kích thước của khung để chứa cả panel camera
        rm.movHome()
        notebook = wx.Notebook(self)
        tab1 = wx.Panel(notebook)
        notebook.AddPage(tab1, "Inspection")  
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)
        #Connect camera
        self.camera_panel = CameraPanel(tab1)  
        self.camera_panel.SetMinSize((640, 640)) 
        main_sizer.Add(self.camera_panel, 1, wx.EXPAND | wx.ALL, 5)  # Thêm panel camera vào layout chính
        tab1.SetSizer(main_sizer)
        # self.Show()
        #Connect Laser



app = wx.App(False)
frame = InspectionFrame(None, "Robot Inspection System")
app.MainLoop()