import wx
import numpy as np
import cv2
import threading
import time
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
        super(InspectionFrame, self).__init__(parent, title=title, size=(800, 600))
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

        main_sizer.Add(left_panel, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(right_panel, 1, wx.EXPAND | wx.ALL, 5)
        tab1.SetSizer(main_sizer)

        # Setup tab2 layout
        settings_sizer = wx.BoxSizer(wx.VERTICAL)
        settings_sizer.Add(wx.StaticText(tab2, label="Settings go here"), 0, wx.ALL, 10)
        tab2.SetSizer(settings_sizer)

        self.Show()

        self.robot_running = False
        self.inspection_thread = None

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
            img_path = 'data/AI/Weld_Data/Weld_Data/Data/Image_(202).png'
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
    
    def start_robot():
        print('Developing')
        return True
    def stop_robot():
        print('Developing')
        return True

    def show_dialog(self, data):
        dialog = ConfirmationDialog(self, "Inspection Confirmation", data)
        result = dialog.ShowModal()
        if result == wx.ID_OK:
            wx.MessageBox("User confirmed OK", "Info", wx.OK | wx.ICON_INFORMATION)
        else:
            wx.MessageBox("User confirmed Not OK", "Info", wx.OK | wx.ICON_INFORMATION)

    def run_ai_model(self, img):
        img_resize = cv2.resize(img, (640, 640), interpolation=cv2.INTER_CUBIC)
        dict_re = self.model.predict(img_resize)
        img_re = dict_re[0][0].orig_img

        coordinate_re = dict_re[0].masks.xy[0]
        img_cp = img_re.copy()
        pts = np.zeros((len(coordinate_re), 2))

        for idx, point in enumerate(coordinate_re):
            cv2.circle(img_cp, (int(point[0]), int(point[1])), 1, (255, 0, 0), 1)
            pts[idx] = [int(point[0]), int(point[1])]
        pts = np.array(pts, np.int32)

        id = 0
        cv2.fillPoly(img_cp, pts=[pts], color=(0, 0, 100 + int(id) * 100, 20))
        x_ratio, y_ratio = 1, 1
        POINT_END = 30
        coordinate = [
            [int(pts[0][0] * y_ratio), int(pts[0][1] * x_ratio)], 
            [int(pts[POINT_END][0] * y_ratio), int(pts[POINT_END][1] * x_ratio)]
        ]
        cv2.circle(img_cp, (int(pts[0][0]), int(pts[0][1])), color=(255, 0, 0), radius=10, thickness=5)
        cv2.circle(img_cp, (int(pts[POINT_END][0]), int(pts[POINT_END][1])), color=(255, 255, 0), radius=10, thickness=5)
        return img_cp, coordinate

    def load_model(self):
        self.status_text.SetLabel("Loading model, please wait...")
        wx.Yield()
        
        def update_gauge():
            for i in range(0, 101, 20):
                time.sleep(0.2)  # loading steps
                wx.CallAfter(self.progress_bar.SetValue, i)
        
        gauge_thread = threading.Thread(target=update_gauge)
        gauge_thread.start()

        self.model = YOLO('src/runs/segment/train7/weights/best.pt')
        
        gauge_thread.join()  # gauge to complete
        wx.CallAfter(self.status_text.SetLabel, "Model loaded successfully")
        wx.CallAfter(self.progress_bar.SetValue, 100)

if __name__ == "__main__":
    app = wx.App()
    frame = InspectionFrame(None, "Inspection Plan")
    app.MainLoop()
