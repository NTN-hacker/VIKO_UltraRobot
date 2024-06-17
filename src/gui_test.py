import wx
import numpy as np
import cv2
import threading
import time
from pypylon import pylon
from pypylon import genicam
from ultralytics import YOLO
import sys
sys.path.append(
    "D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot"
)
from config import config as CFG
from library import viko_lib as lib
from src import robotic_modify as rm
from layout.gui import gui_vision 

import math

class DistanceCalculator:
    def __init__(self, known_width, focal_length, sensor_width):
        self.known_width = known_width  
        self.focal_length = focal_length  
        self.sensor_width = sensor_width  

    def calculate_distance(self, width_in_frame):
           
        distance = 70

        return distance
class ImagePanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        h, w = 640, 640
        src = (255 * np.random.rand(h, w)).astype(np.uint8)
        buf = np.dstack([src] * 3).tobytes()
        self.bitmap = wx.Bitmap.FromBuffer(w, h, buf)
        self.SetDoubleBuffered(True)
        self.Bind(wx.EVT_PAINT, self.OnPaint)

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.bitmap, 0, 0)

    def update_image(self, img):
        h, w = 640, 640 #display
        img = cv2.resize(img, (h, w), cv2.INTER_CUBIC)
        buf = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).tobytes()
        self.bitmap = wx.Bitmap.FromBuffer(w, h, buf)
        self.Refresh()

class CameraPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        self.h, self.w = 640, 640
        self.bitmap = wx.Bitmap(self.w, self.h)
        self.SetDoubleBuffered(True)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        for device in devices:
            print(device.GetModelName(), device.GetSerialNumber())
            self.model_name = device.GetModelName()
        
        self.camera = pylon.InstantCamera()
        self.camera.Attach(tl_factory.CreateDevice(devices[0]))
        self.camera_thread = threading.Thread(target=self.update_camera)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        self.running = True  
        self.lock = threading.Lock()
        self.capturing_image = False  # Thêm cờ kiểm soát việc lấy ảnh

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.bitmap, 0, 0)

    def update_image(self, img):
        if self:  
            img = cv2.resize(img, (self.h, self.w), cv2.INTER_CUBIC)
            buf = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).tobytes()
            self.bitmap.CopyFromBuffer(buf)
            self.Refresh()

    def update_camera(self):
        self.camera.Open()
        self.camera.GainAuto.SetValue("Off")  
        self.camera.Gain.SetValue(8.0)
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        while self.camera.IsGrabbing() and self.running:
            with self.lock:
                if not self.capturing_image:  # Kiểm tra cờ trước khi lấy ảnh
                    grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                    if grabResult.GrabSucceeded():
                        # Access the image data
                        image1 = converter.Convert(grabResult)
                        image = image1.GetArray()
                        # image = cv2.resize(image, (640, 640), fx=0.8366, fy=1, interpolation=cv2.INTER_LINEAR)
                        wx.CallAfter(self.update_image, image)
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

    def capture_image(self):
        with self.lock:
            self.capturing_image = True  # Đặt cờ trước khi lấy ảnh
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            image = None
            if grabResult.GrabSucceeded():
                converter = pylon.ImageFormatConverter()
                converter.OutputPixelFormat = pylon.PixelType_BGR8packed
                converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
                image = converter.Convert(grabResult).GetArray()
            grabResult.Release()
            self.capturing_image = False  
            return image
    
class ConfirmationDialog(wx.Dialog):
    def __init__(self, parent, title, data):
        super(ConfirmationDialog, self).__init__(parent, title=title, size=(350, 200))
        
        panel = wx.Panel(self)
        sizer = wx.BoxSizer(wx.VERTICAL)
        
        for key, value in data.items():
            sizer.Add(wx.StaticText(panel, label=f"{key}: {value}"), 0, wx.ALL, 10)
        
        button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.ok_button = wx.Button(panel, label="OK")
        self.not_ok_button = wx.Button(panel, label="Not OK")
        
        button_sizer.Add(self.ok_button, 1, wx.EXPAND | wx.ALL, 10)
        button_sizer.Add(self.not_ok_button, 1, wx.EXPAND | wx.ALL, 10)
        
        sizer.Add(button_sizer, 0, wx.CENTER)
        
        panel.SetSizer(sizer)
        
        self.ok_button.Bind(wx.EVT_BUTTON, self.on_ok)
        self.not_ok_button.Bind(wx.EVT_BUTTON, self.on_not_ok)
        
    def on_ok(self, event):
        self.EndModal(wx.ID_OK)
        
    def on_not_ok(self, event):
        self.EndModal(wx.ID_CANCEL)



class InspectionFrame(wx.Frame):
    def __init__(self, parent, title):
        super(InspectionFrame, self).__init__(parent, title=title, size=(1400, 700))  # Tăng kích thước của khung để chứa cả panel camera
        self.h, self.w = 640, 640

        self.load_model()

        #Distance
        self.distance_calculator = DistanceCalculator(
            known_width=10,  
            focal_length=16,  
            sensor_width=3.45/1000  
        )

        notebook = wx.Notebook(self)
        tab1 = wx.Panel(notebook)
        tab2 = wx.Panel(notebook)
        notebook.AddPage(tab1, "Inspection")
        notebook.AddPage(tab2, "Settings")

        self.pos_status = 'home'

        # Setup tab1 layout
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)
        left_panel = wx.Panel(tab1)
        left_sizer = wx.BoxSizer(wx.VERTICAL)

        # inspection_points = ["home", "center", "left", 'right']
        # for point in inspection_points:
        #     left_sizer.Add(wx.Button(left_panel, label=point), 0, wx.EXPAND | wx.ALL, 5)

        # Load icon
        icon = wx.Bitmap("layout/png/lab.png", wx.BITMAP_TYPE_PNG)

        # Create an icon above the buttons
        
        button_size = (100, 30)  
        icon = wx.ImageFromBitmap(icon).Scale(button_size[0], button_size[1], wx.IMAGE_QUALITY_HIGH).ConvertToBitmap()
        self.icon_bitmap = wx.StaticBitmap(left_panel, bitmap=icon)
        self.run_center_btn = wx.Button(left_panel, label="center")
        self.run_left_btn = wx.Button(left_panel, label="left")
        self.run_right_btn = wx.Button(left_panel, label="right")

        self.run_center_btn.Bind(wx.EVT_BUTTON, self.on_run_center)
        self.run_left_btn.Bind(wx.EVT_BUTTON, self.on_run_left)
        self.run_right_btn.Bind(wx.EVT_BUTTON, self.on_run_right)

        self.run_center_btn.Bind(wx.EVT_ENTER_WINDOW, self.on_hover)
        self.run_center_btn.Bind(wx.EVT_LEAVE_WINDOW, self.on_leave)
        self.run_left_btn.Bind(wx.EVT_ENTER_WINDOW, self.on_hover)
        self.run_left_btn.Bind(wx.EVT_LEAVE_WINDOW, self.on_leave)
        self.run_right_btn.Bind(wx.EVT_ENTER_WINDOW, self.on_hover)
        self.run_right_btn.Bind(wx.EVT_LEAVE_WINDOW, self.on_leave)

        left_sizer.Add(self.icon_bitmap, 0, wx.ALL | wx.CENTER, 5)
        left_sizer.Add(self.run_center_btn, 0, wx.ALL, 5)
        left_sizer.Add(self.run_left_btn, 0, wx.ALL, 5)
        left_sizer.Add(self.run_right_btn, 0, wx.ALL, 5)

        left_panel.SetSizer(left_sizer)

        right_panel = wx.Panel(tab1)
        right_sizer = wx.BoxSizer(wx.VERTICAL)

        self.image_panel = ImagePanel(right_panel)
        self.image_panel.SetMinSize((640, 640))
        right_sizer.Add(self.image_panel, 1, wx.EXPAND, 5)

        button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.run_inspection_plan_btn = wx.Button(right_panel, label="Run Inspection Plan")
        self.start_robot_btn = wx.Button(right_panel, label="Start Robot")
        self.stop_robot_btn = wx.Button(right_panel, label="Stop Robot")
        self.save_image_btn = wx.Button(right_panel, label = "Save Image")

        self.run_inspection_plan_btn.Bind(wx.EVT_BUTTON, self.on_run_inspection_plan)
        self.start_robot_btn.Bind(wx.EVT_BUTTON, self.on_start_robot)
        self.stop_robot_btn.Bind(wx.EVT_BUTTON, self.on_stop_robot)
        self.save_image_btn.Bind(wx.EVT_BUTTON, self.on_save)

        button_sizer.Add(self.run_inspection_plan_btn, 0, wx.ALL, 5)
        button_sizer.Add(self.start_robot_btn, 0, wx.ALL, 5)
        button_sizer.Add(self.stop_robot_btn, 0, wx.ALL, 5)
        button_sizer.Add(self.save_image_btn, 0, wx.ALL, 5)

        right_sizer.Add(button_sizer, 0, wx.ALIGN_CENTER)

        self.progress_bar = wx.Gauge(right_panel, range=100, size=(250, 25))
        self.status_text = wx.StaticText(right_panel, label="")

        right_sizer.Add(self.progress_bar, 0, wx.ALIGN_CENTER | wx.ALL, 5)
        right_sizer.Add(self.status_text, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        right_panel.SetSizer(right_sizer)

        self.camera_panel = CameraPanel(tab1)  
        self.camera_panel.SetMinSize((640, 640)) 

        main_sizer.Add(left_panel, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(right_panel, 1, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(self.camera_panel, 1, wx.EXPAND | wx.ALL, 5)  # Thêm panel camera vào layout chính

        tab1.SetSizer(main_sizer)

        # Setup tab2 layout
        settings_sizer = wx.BoxSizer(wx.VERTICAL)
        
        settings_sizer.Add(wx.StaticText(tab2, label="Settings go here"), 0, wx.ALL, 10)
        tab2.SetSizer(settings_sizer)
        # self.SettingWindow = gui_vision.BaslerGuiWindow(tab2)

        self.Show()

        self.robot_running = False
        self.inspection_thread = None

        self.Bind(wx.EVT_CLOSE, self.on_close)

    def on_run_inspection_plan(self, event):
        self.robot_running = True
        self.inspection_thread = threading.Thread(target=self.run_inspection)
        self.inspection_thread.start()

    def on_start_robot(self, event):
        self.robot_running = True
        
        self.start_robot()

    def on_stop_robot(self, event):
        self.robot_running = False
        self.stop_robot()
        if self.inspection_thread is not None:
            self.inspection_thread.join()
    ###################GUI#############################
    def on_hover(self, event):
        button = event.GetEventObject()
        button.SetBackgroundColour(wx.Colour(173, 216, 230))  # Change to whatever color you want
        button.Refresh()

    def on_leave(self, event):
        button = event.GetEventObject()
        button.SetBackgroundColour(wx.NullColour)  # Reset to default color
        button.Refresh()
    ###################################################
    ################# NEW FUNCTION ####################
        
    def on_save(self, event):
        self.save()

    def on_run_center(self, event):
        self.robot_running = True
        self.pos_status = 'home'
        self.move_center()

    def on_run_left(self, event):
        self.robot_running = True
        self.pos_status = 'left'
        self.move_left()
    
    def on_run_right(self, event):
        self.robot_running = True
        self.pos_status = 'right'
        self.move_right()
    #####################################
    def run_inspection(self):

        try:
            img = self.camera_panel.capture_image()  
            cv2.imwrite('temp.png', img)
            if img is not None:
                wx.CallAfter(self.image_panel.update_image, img)
                detected_img = self.run_inspection_point(img)
                wx.CallAfter(self.image_panel.update_image, detected_img)
                time.sleep(1)  
            else:
                print("Failed to capture image")
        except Exception as e:
            print(f"Error during inspection: {e}")

    # def run_inspection_point(self, img):
    #     detected_img, coordinate, flag = self.run_ai_model(img)
    #     if flag == 0:
    #     #     data = {
    #     #         "Z_Distance": "70 cm",
    #     #         "Start_Point": f"{coordinate[0]}",
    #     #         "End_Point": f"{coordinate[1]}",
    #     #         "Object": "A"
    #     #     }
    #     # else:
    #         data = {"Note":"No weld object"}
    #         wx.CallAfter(self.show_dialog, data)
    #     else:
    #         pass
    #     return detected_img

    def run_inspection_point(self, img):
        detected_img, boxes, name_obj, flag = self.run_ai_model(img)
        print('boxes', boxes)
        print('Model weld  ', name_obj)
        if flag == 1:
            if boxes is not None and len(boxes) > 0:
                box = boxes.tolist()  
                object_width = box[2] - box[0]  
                distance = self.distance_calculator.calculate_distance(object_width)
                data = {
                    "Z_Distance": f"{distance:.2f} cm",
                    "Start_Point": f"{box[0]}, {box[1]}",
                    "End_Point": f"{box[2]}, {box[3]}",
                    "Object": name_obj
                }
            else:
                data = {"Note": "No weld object"}
            wx.CallAfter(self.show_dialog, data)
        else:
            pass
        return detected_img

    # def start_robot(self):
    #     img = cv2.imread('temp.png', cv2.IMREAD_ANYCOLOR)
    #     print('Testing')
    #     rm.run(self.model, img, self.pos_status)
    
    #     return True

    def start_robot(self):
        self.robot_running = True
        robot_thread = threading.Thread(target=self.run_robot)
        robot_thread.start()

    def run_robot(self):
        img = cv2.imread('temp.png', cv2.IMREAD_ANYCOLOR)
        rm.run(self.model, img, self.pos_status)

    def stop_robot(self):
        print('Testing')
        rm.stop()
        return True

    def show_dialog(self, data):
        dialog = ConfirmationDialog(self, "Inspection Confirmation", data)
        result = dialog.ShowModal()
        if result == wx.ID_OK:
            wx.MessageBox("User confirmed OK", "Info", wx.OK | wx.ICON_INFORMATION)
        else:
            wx.MessageBox("User confirmed Not OK", "Info", wx.OK | wx.ICON_INFORMATION)
        dialog.Destroy()

    def load_model(self):
        # self.status_text.SetLabel("Loading model, please wait...")
        # wx.Yield()
        
        def update_gauge():
            for i in range(0, 101, 20):
                time.sleep(0.2)  # loading steps
                wx.CallAfter(self.progress_bar.SetValue, i)
        
        gauge_thread = threading.Thread(target=update_gauge)
        gauge_thread.start()

        self.model = YOLO(CFG.MODEL['YOLOV9']['WEIGHT'])
        
        gauge_thread.join()  # gauge to complete
        # wx.CallAfter(self.status_text.SetLabel, "Model loaded successfully")
        # wx.CallAfter(self.progress_bar.SetValue, 100)

    def run_ai_model(self, img):
        print(img.shape)
        img = cv2.resize(img, (640, 640), interpolation=cv2.INTER_CUBIC)
        results = self.model.predict(source=img, conf=0.65)
        
        plot = results[0].plot()

        print(1)

        #save data
        label_out = lib.getLabels(results)
        name_obj = 'Object test' #lib.findContainingPairs(results)
        print(label_out)
        print(name_obj)

        flag = 1 if 1 in label_out else 0 #NEW MODEL
        # flag = 0 if 0 in label_out else 1
        print(flag)

        coordinate_re = results[0].masks.xy[0]
        pts = np.zeros((len(coordinate_re), 2))

        for idx, point in enumerate(coordinate_re):
            pts[idx] = [int(point[0]), int(point[1])]
        pts = np.array(pts, np.int32)
        
        return plot, results[0].boxes.xyxy[0], name_obj, flag

    def on_close(self, event):
        if hasattr(self, 'camera_panel') and self.camera_panel:
            self.camera_panel.stop_camera()  
        self.Destroy()
    

    ################ NEW FUNCTION ####################

    def move_center(self):
        rm.movHome()
        print('Testing')
        return True
    
    def move_left(self):
        rm.movL()
        print('Testing')
        return True
    
    def move_right(self):
        rm.movR()
        print('Testing')
        return True

    def save(self):
        from datetime import datetime
        from PIL import Image
        img_arr = self.camera_panel.capture_image()  
        img_arr = cv2.cvtColor(img_arr, cv2.COLOR_BGR2RGB)
        print(img_arr.shape)
        img = Image.fromarray(img_arr)
        current_time = datetime.now()
        timestamp = current_time.strftime("%Y%m%d_%H%M%S")
        new_filename = f"data/AI/Data_Label/image_{timestamp}.png"
        img.save(new_filename)
        
        print(f"Ảnh đã được lưu với tên: {new_filename}")

    ###################################

app = wx.App(False)
frame = InspectionFrame(None, "Robot Inspection System")
app.MainLoop()