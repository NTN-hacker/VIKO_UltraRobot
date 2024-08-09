import sys
import io
import wx
import numpy as np
import cv2
import threading
import time
from pypylon import pylon
from ultralytics import YOLO
from datetime import datetime, timedelta
from library.experimental_src.laser import Laser
from config import config as CFG
from src import robotic_modify as rm
from library import viko_lib as vl
from IPCDataMs import IPCData

# Add custom module paths
sys.path.append("E:\\Quan\\AutoRoboticInspection-V1\\VIKO_UltraRobot")

# Uncomment if encoding issues occur
# sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
# sys.stdin = io.TextIOWrapper(sys.stdin.buffer, encoding='utf-8')

# Global Variables
global Inpection_Dict, status, idx, idx_Coord, result_image, idx_below_frame, image


# Function to load YOLO model
def load_model(weight):
    return YOLO(weight, task='detect')


# Function to run inspection using the model
def run_inspection(model, img):
    img = cv2.resize(img, (640, 640), interpolation=cv2.INTER_CUBIC)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = model.predict(source=img, conf=0.65)  # CFG.CONF_ACC
    return results[0].plot()


# Function to get coordinates from the model
def get_coordinate(model, img):
    return rm.getCoordinates(model, img)


# Function to run robot operations
def run_robot(coordinate_pixel_list, model_weld_list, _suf_, pos_status):
    return rm.run(coordinate_pixel_list, model_weld_list, _suf_, pos_status)


# Function to stop the robot
def stop_robot():
    print('Stop.')
    rm.stop()


# Function to move robot to the center
def move_center():
    print('Move center.')
    rm.movHome()


# Function to run an inspection process
def inspection(model, image):
    global Inpection_Dict
    print('Inspection Result')
    current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    img_cvt = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    dict_re = model.predict(img_cvt)
    result_image = cv2.resize(dict_re[0].plot(), (640, 640), cv2.INTER_CUBIC)
    cv2.imwrite(f'laser/inspection/inspection_{current_time}.png', result_image)

    out_labels = vl.getLabels(dict_re)
    ref_label = [CFG.MODEL_INSPECTION[label] for label in out_labels]
    defects = np.array(ref_label)
    Inpection_Dict = vl.count_defects(defects)

    total = sum(Inpection_Dict.values())
    for key in Inpection_Dict:
        Inpection_Dict[key] = (Inpection_Dict[key] / total) * 100

    return result_image


# Camera Panel Class
class CameraPanel(wx.Panel):
    def __init__(self, parent):
        super().__init__(parent)
        global idx, idx_Coord, idx_Chart, idx_below_frame, status, image

        # Initialize global variables
        idx = idx_Coord = idx_Chart = idx_below_frame = 0
        status = None
        image = None

        self.ipc_data = IPCData()
        self.h, self.w = 640, 640
        self.robot_running = False
        self.pos_status = 'home'
        self._suf_left_ = 1
        self._suf_right_ = -1
        self.result_image = None
        self.response_result_laser = False
        self.running = True
        self.response_result = False
        self.data_laser = False
        self.data_inspection = False
        self.lock = threading.Lock()
        self.frame_count = 0
        self.laser_data = None

        # Load models
        self.model = YOLO(CFG.MODEL['YOLOV9']['WEIGHT'], task='segment')
        self.model_inspection = load_model(CFG.MODEL['INSPECTION']['WEIGHT'])

        # Camera setup
        self.bitmap = wx.Bitmap(self.w, self.h)
        self.SetDoubleBuffered(True)
        self.camera = self.initialize_camera()

        # Threads for camera and button checking
        self.camera_thread = threading.Thread(target=self.update_camera, daemon=True)
        self.button_thread = threading.Thread(target=self.check_buttons, daemon=True)
        self.camera_thread.start()
        self.button_thread.start()

    def initialize_camera(self):
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()

        for device in devices:
            print(device.GetModelName(), device.GetSerialNumber())
            self.model_name = device.GetModelName()

        camera = pylon.InstantCamera()
        camera.Attach(tl_factory.CreateDevice(devices[0]))
        return camera

    def update_image(self, img):
        if self:
            img = cv2.resize(img, (self.h, self.w), cv2.INTER_CUBIC)
            buf = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).tobytes()
            self.bitmap.CopyFromBuffer(buf)
            self.Refresh()

    def update_camera(self):
        global image

        while self.running:
            self.camera.Open()
            self.camera.GainAuto.SetValue("Off")
            self.camera.Gain.SetValue(5.0)
            self.camera.ExposureTime.SetValue(2000.0)
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            converter = pylon.ImageFormatConverter()
            converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

            while self.camera.IsGrabbing() and self.running:
                with self.lock:
                    grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                    if grabResult.GrabSucceeded():
                        image1 = converter.Convert(grabResult)
                        image = image1.GetArray()

                        img = cv2.resize(image, (self.h, self.w), cv2.INTER_CUBIC)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        image = img.copy()
                        img_np = np.frombuffer(image, dtype=np.uint8).reshape((640, 640, 3))
                        self.ipc_data.send_frame(img_np)
                        self.frame_count += 1
                        time.sleep(1 / 35)
                        wx.CallAfter(self.update_image, image)

                        global result_image
                        if self.response_result:
                            self.send_results()

                    grabResult.Release()
            self.camera.StopGrabbing()
            self.camera.Close()

    def send_results(self):
        global result_image, idx_below_frame

        result_image_np = np.frombuffer(result_image, dtype=np.uint8).reshape((640, 640, 3))
        self.ipc_data.send_frame_below(idx_below_frame, result_image_np)

        self.ipc_data.send_status(idx, f'Machine Vision: {status}')

        if self.data_laser:
            self.send_laser_data()

        if self.data_inspection:
            global Inpection_Dict
            self.ipc_data.send_chart(idx_Chart, Inpection_Dict)
            self.data_inspection = False

        self.response_result = False

    def send_laser_data(self):
        filepath = 'laser/datascan/datascan.txt'
        data = []

        with open(filepath, 'r') as file:
            for line in file:
                row = list(map(float, line.split()))
                data.append(row)

        self.ipc_data.send_3Ddata(idx_Coord, 3072, data)
        self.data_laser = False

    def stop_camera(self):
        self.running = False
        if self.camera_thread.is_alive():
            self.camera_thread.join()

    def on_close(self, event):
        self.stop_camera()
        self.Destroy()

    def check_buttons(self):
        start = [-1] * 6
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
                    print("Button:", idx)
            time.sleep(33 / 1000)

    def process_button(self, button_idx):
        global result_image, idx, idx_Coord, idx_Chart, idx_below_frame, status, image, Inpection_Dict

        self.result_image = None
        if button_idx == 0:
            self.handle_button_0()
        elif button_idx == 1:
            self.handle_button_1()
        elif button_idx == 2:
            self.handle_button_2()
        elif button_idx == 3:
            self.handle_button_3()
        elif button_idx == 4:
            self.handle_button_4()

        time.sleep(2)

    def handle_button_0(self):
        global idx, status, result_image, image

        status = "Processing"
        idx += 1
        result_image = run_inspection(self.model, image)

        if not isinstance(result_image, np.ndarray):
            status = 'No image'
            print(status)
            return

        idx += 1
        status = "Detected"
        self.response_result = True

    def handle_button_1(self):
        global idx, idx_Coord, idx_below_frame, status, result_image, image

        idx += 1
        status = "Planning weld..."
        self.response_result = True

        coordinate_pixel_list, model_weld_list, planning_img = get_coordinate(model=self.model, img=image)
        result_image = planning_img.copy()
        idx_below_frame += 1

        idx += 1
        status = "Moving..."
        self.laser_data = run_robot(coordinate_pixel_list, model_weld_list, self._suf_left_, self.pos_status)
        print("Laser data at the end:", type(self.laser_data), self.laser_data)

        idx += 1
        idx_Coord += 1
        status = "Completed."
        result_image = vl.convert_to_grayscale_image(self.laser_data)
        self.save_laser_image()

    def handle_button_2(self):
        global idx, idx_Coord, idx_below_frame, status, result_image, image

        idx += 1
        status = "Planning weld..."
        self.response_result = True

        coordinate_pixel_list, model_weld_list, planning_img = get_coordinate(model=self.model, img=image)
        result_image = planning_img.copy()
        idx_below_frame += 1

        idx += 1
        status = "Moving..."
        self.laser_data = run_robot(coordinate_pixel_list, model_weld_list, self._suf_right_, self.pos_status)
        print("Laser data at the end:", self.laser_data)

        idx += 1
        idx_Coord += 1
        status = "Completed."
        result_image = vl.convert_to_grayscale_image(self.laser_data)
        self.save_laser_image()

    def handle_button_3(self):
        global idx, idx_Chart, idx_below_frame, status, result_image, image, Inpection_Dict

        idx += 1
        status = "Inspection Processing."

        path_img = 'laser/depthmap/datascan-python.png'
        image = cv2.imread(path_img, cv2.IMREAD_COLOR)
        idx_below_frame += 1
        result_image = inspection(self.model_inspection, image)

        idx_Chart += 1
        self.data_inspection = True

    def handle_button_4(self):
        global idx, status
        idx += 1
        status = "Back Home."
        move_center()

    def save_laser_image(self):
        global result_image

        current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        cv2.imwrite(f'laser/experimental/laser_{current_time}.png', result_image)
        self.data_laser = True

    def stop_threads(self):
        self.running = False
        self.camera_thread.join()
        self.button_thread.join()


# Inspection Frame Class
class InspectionFrame(wx.Frame):
    def __init__(self, parent, title):
        super().__init__(parent, title=title, size=(1400, 700))
        rm.movHome()

        notebook = wx.Notebook(self)
        tab1 = wx.Panel(notebook)
        notebook.AddPage(tab1, "Inspection")

        main_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.camera_panel = CameraPanel(tab1)
        self.camera_panel.SetMinSize((640, 640))
        main_sizer.Add(self.camera_panel, 1, wx.EXPAND | wx.ALL, 5)

        tab1.SetSizer(main_sizer)


if __name__ == "__main__":
    app = wx.App(False)
    frame = InspectionFrame(None, "Robot Inspection System")
    app.MainLoop()
