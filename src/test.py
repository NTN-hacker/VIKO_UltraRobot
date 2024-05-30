import wx
import numpy as np
import cv2
import threading
import time
from pypylon import pylon
from pypylon import genicam
from ultralytics import YOLO

class ImagePanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        h, w = 580, 780
        src = (255 * np.random.rand(h, w)).astype(np.uint8)
        buf = np.dstack([src] * 3).tobytes()
        self.bitmap = wx.Bitmap.FromBuffer(w, h, buf)
        self.SetDoubleBuffered(True)
        self.Bind(wx.EVT_PAINT, self.OnPaint)

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.bitmap, 0, 0)

    def update_image(self, img):
        h, w = img.shape[:2]
        buf = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).tobytes()
        self.bitmap = wx.Bitmap.FromBuffer(w, h, buf)
        self.Refresh()

class CameraPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        h, w = 2048, 2448
        self.bitmap = wx.Bitmap(w, h)
        self.SetDoubleBuffered(True)
        self.Bind(wx.EVT_PAINT, self.OnPaint)

        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera_thread = threading.Thread(target=self.update_camera)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        self.running = True  

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.bitmap, 0, 0)

    def update_image(self, img):
        if self:  
            h, w = img.shape[:2]
            buf = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).tobytes()
            self.bitmap.CopyFromBuffer(buf)
            self.Refresh()

    def update_camera(self):
        self.camera.Open()
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        while self.camera.IsGrabbing() and self.running:
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grabResult.GrabSucceeded():
                image = converter.Convert(grabResult).GetArray()
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
        super(InspectionFrame, self).__init__(parent, title=title, size=(1200, 600))  # Tăng kích thước của khung để chứa cả panel camera
        self.h, self.w = 580, 780

        notebook = wx.Notebook(self)
        tab1 = wx.Panel(notebook)
        tab2 = wx.Panel(notebook)
        notebook.AddPage(tab1, "Inspection")
        notebook.AddPage(tab2, "Settings")

        # Setup tab1 layout
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)
        left_panel = wx.Panel(tab1)
        left_sizer = wx.BoxSizer(wx.VERTICAL)

        inspection_points = ["home", "center", "left", 'right']
        for point in inspection_points:
            left_sizer.Add(wx.Button(left_panel, label=point), 0, wx.EXPAND | wx.ALL, 5)

        left_panel.SetSizer(left_sizer)

        right_panel = wx.Panel(tab1)
        right_sizer = wx.BoxSizer(wx.VERTICAL)

        self.image_panel = ImagePanel(right_panel)
        right_sizer.Add(self.image_panel, 1, wx.EXPAND, 5)

        button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.run_inspection_plan_btn = wx.Button(right_panel, label="Run Inspection Plan")
        self.start_robot_btn = wx.Button(right_panel, label="Start Robot")
        self.stop_robot_btn = wx.Button(right_panel, label="Stop Robot")

        self.run_inspection_plan_btn.Bind(wx.EVT_BUTTON, self.on_run_inspection_plan)
        self.start_robot_btn.Bind(wx.EVT_BUTTON, self.on_start_robot)
        self.stop_robot_btn.Bind(wx.EVT_BUTTON, self.on_stop_robot)

        button_sizer.Add(self.run_inspection_plan_btn, 0, wx.ALL, 5)
        button_sizer.Add(self.start_robot_btn, 0, wx.ALL, 5)
        button_sizer.Add(self.stop_robot_btn, 0, wx.ALL, 5)

        right_sizer.Add(button_sizer, 0, wx.ALIGN_CENTER)

        self.progress_bar = wx.Gauge(right_panel, range=100, size=(250, 25))
        self.status_text = wx.StaticText(right_panel, label="")

        right_sizer.Add(self.progress_bar, 0, wx.ALIGN_CENTER | wx.ALL, 5)
        right_sizer.Add(self.status_text, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        right_panel.SetSizer(right_sizer)

        self.camera_panel = CameraPanel(tab1)  # Lưu tham chiếu tới CameraPanel
        main_sizer.Add(left_panel, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(right_panel, 1, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(self.camera_panel, 1, wx.EXPAND | wx.ALL, 5)  # Thêm panel camera vào layout chính

        tab1.SetSizer(main_sizer)

        # Setup tab2 layout
        settings_sizer = wx.BoxSizer(wx.VERTICAL)
        settings_sizer.Add(wx.StaticText(tab2, label="Settings go here"), 0, wx.ALL, 10)
        tab2.SetSizer(settings_sizer)

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

    def run_inspection(self):
        self.load_model()
        try:
            img_path = 'data/AI/Record_2024-05-03/20240503-230659223037-acA2440-35uc_22481496.png'
            img = cv2.resize(cv2.imread(img_path, cv2.IMREAD_ANYCOLOR), (self.w, self.h))
            wx.CallAfter(self.image_panel.update_image, img)
            detected_img = self.run_inspection_point(img)
            wx.CallAfter(self.image_panel.update_image, detected_img)
            time.sleep(1)  
        except Exception as e:
            print(f"Error during inspection: {e}")

    def run_inspection_point(self, img):
        detected_img, coordinate = self.run_ai_model(img)
        data = {
            "Z_Distance": "70 cm",
            "Start_Point": f"{coordinate[0]}",
            "End_Point": f"{coordinate[1]}",
            "Object": "A"
        }
        wx.CallAfter(self.show_dialog, data)
        return detected_img
    
    def start_robot(self):
        print('Developing')
        return True
    def stop_robot(self):
        print('Developing')
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
        self.status_text.SetLabel("Loading model, please wait...")
        wx.Yield()
        
        def update_gauge():
            for i in range(0, 101, 20):
                time.sleep(0.2)  # loading steps
                wx.CallAfter(self.progress_bar.SetValue, i)
        
        gauge_thread = threading.Thread(target=update_gauge)
        gauge_thread.start()

        self.model = YOLO('weight/best.pt')
        
        gauge_thread.join()  # gauge to complete
        wx.CallAfter(self.status_text.SetLabel, "Model loaded successfully")
        wx.CallAfter(self.progress_bar.SetValue, 100)

    def run_ai_model(self, img):
        print(img.shape)
        results = self.model.predict(source=img, save=True, conf=0.25)
        plot = results[0].plot()
        
        return plot, results[0].boxes.xyxy[0]

    def on_close(self, event):
        if hasattr(self, 'camera_panel') and self.camera_panel:
            self.camera_panel.stop_camera()  
        self.Destroy()

app = wx.App(False)
frame = InspectionFrame(None, "Robot Inspection System")
app.MainLoop()
