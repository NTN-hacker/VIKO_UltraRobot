# import pandas as pd

import wx
from skimage.feature import graycomatrix
from scipy.optimize import curve_fit
import numpy as np
import cv2
import datetime
import time
import threading
from pypylon import pylon

import wx.lib.agw.floatspin as FS
import csv


class ImagePanel(wx.Panel):

    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        h, w = 640, 640
        src = (255 * np.random.rand(h, w)).astype(np.uint8)
        buf = src.repeat(3, 1).tobytes()
        self.bitmap = wx.Image(w, h, buf).ConvertToBitmap()
        self.SetDoubleBuffered(True)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Size = (640, 640)
        self.Fit()

    def OnPaint(self, evt):
        wx.BufferedPaintDC(self, self.bitmap)

    def update(self, input_image):
        self.bitmap = input_image
        wx.BufferedDC(wx.ClientDC(self), self.bitmap)


class BaslerGuiWindow(wx.Frame):

    output_string_1 = ""
    output_string_2 = ""
    update_ratio = 0
    font = cv2.FONT_HERSHEY_SIMPLEX
    lock = threading.Lock()

    auto_index_on = False
    current_index = 0
    append_date_flag = False
    measurement_interval = 2
    sequence_length = 1
    current_step = 0
    last_capture_time = 0
    frames_to_capture = 360
    selected_mode = 0
    capture_mode = 0
    time_to_next = 0

    selected_camera = 0
    auto_exposure_on = False
    auto_gain_on = False
    preview_on = False
    capture_on = False
    framerate = 35
    exposure = 7
    gain = 0
    cameras_list = []
    capture_sequence_timer = None
    capture_status_timer = None
    camera_connected = False
    camera = []

    frame_width = 640
    frame_height = 640

    roi_on = False
    roi_x = 0
    roi_y = 0
    roi_width = 10
    roi_height = 10
    min_gray_val = 5
    preview_thread_obj = None
    capture_thread_obj = None
    process_thread_obj = None
    max_contrast = 0.8

    current_frame = np.zeros((640, 640, 1), np.float32)
    gray = np.zeros((640, 640, 1), np.uint8)
    mean_img_sq = np.zeros((640, 640, 1), np.float32)
    sq = np.zeros((640, 640, 1), np.float32)
    img = np.zeros((640, 640, 1), np.float32)
    mean_img = np.zeros((640, 640, 1), np.float32)
    sq_img_mean = np.zeros((640, 640, 1), np.float32)
    std = np.zeros((640, 640, 1), np.float32)
    LASCA = np.zeros((640, 640, 1), np.uint8)
    im_color = np.zeros((640, 640, 3), np.uint8)
    mask = np.zeros((640, 640, 1), bool)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)).astype(np.float32)
    kernel /= np.sum(kernel)

    def __init__(self, *args, **kwargs):
        super(BaslerGuiWindow, self).__init__(*args, **kwargs)
        self.InitUI()
        self.Bind(wx.EVT_CLOSE, self.OnCloseWindow)
        self.Centre()
        self.Show()

    def InitUI(self):
        devices = pylon.TlFactory.GetInstance().EnumerateDevices()
        cameras = []
        for device in devices:
            cameras.append(device.GetModelName() + "_" + device.GetSerialNumber())
            self.cameras_list.append({"name": device.GetModelName(),
                                      "serial": device.GetSerialNumber()})

        panel = wx.Panel(self)
        self.SetTitle('Basler CAM GUI')
        sizer = wx.GridBagSizer(0, 0)

        selected_ctrl_label = wx.StaticText(panel, label="Selected camera:")
        sizer.Add(selected_ctrl_label, pos=(0, 0),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.cam_combo = wx.ComboBox(panel, choices=cameras)
        sizer.Add(self.cam_combo, pos=(1, 0),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.cam_combo.Bind(wx.EVT_COMBOBOX, self.OnCamCombo)
        self.cam_combo.SetSelection(self.selected_camera)

        self.connect_btn = wx.Button(panel, label="Connect")
        self.connect_btn.Bind(wx.EVT_BUTTON, self.OnConnect)
        sizer.Add(self.connect_btn, pos=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)

        self.refresh_btn = wx.Button(panel, label="Refresh list")
        self.refresh_btn.Bind(wx.EVT_BUTTON, self.OnRefreshList)
        sizer.Add(self.refresh_btn, pos=(2, 0),
                  flag=wx.EXPAND | wx.ALL, border=5)

        self.preview_btn = wx.Button(panel, label="Preview START")
        self.preview_btn.Bind(wx.EVT_BUTTON, self.OnPreview)
        sizer.Add(self.preview_btn, pos=(2, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        
        self.save_btn = wx.Button(panel, label = "Save Image")
        self.save_btn.Bind(wx.EVT_BUTTON, self.OnSaveImage)
        sizer.Add(self.save_btn, pos = (2, 2), flag = wx.EXPAND | wx.ALL, border = 5)

        capmode_ctrl_label = wx.StaticText(panel, label="Capture mode:")
        sizer.Add(capmode_ctrl_label, pos=(13, 0), flag=wx.EXPAND | wx.ALL, border=5)
        modes = ['RAW_VIDEO', 'LASCA_VIDEO', 'DATA_TABLE', 'RAW_IMAGE', 'DATA_&_VIDEO']
        self.capmode_combo = wx.ComboBox(panel, choices=modes)
        sizer.Add(self.capmode_combo, pos=(13, 1), flag=wx.ALL, border=5)
        self.capmode_combo.Bind(wx.EVT_COMBOBOX, self.OnCapModeCombo)
        self.capmode_combo.SetSelection(self.capture_mode)

        interval_ctrl_label = wx.StaticText(panel,
                                            label="Measurement interval (sec):")
        sizer.Add(interval_ctrl_label, pos=(14, 0),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.interval_ctrl = wx.TextCtrl(panel)
        self.interval_ctrl.SetValue(str(self.measurement_interval))
        sizer.Add(self.interval_ctrl, pos=(14, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)

        sequence_ctrl_label = wx.StaticText(panel, label="Sequence length (num):")
        sizer.Add(sequence_ctrl_label, pos=(15, 0),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.sequence_ctrl = wx.TextCtrl(panel)
        self.sequence_ctrl.SetValue(str(self.sequence_length))
        sizer.Add(self.sequence_ctrl, pos=(15, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)

        mode_ctrl_label = wx.StaticText(panel, label="Preview mode:")
        sizer.Add(mode_ctrl_label, pos=(3, 0), flag=wx.EXPAND | wx.ALL, border=5)
        modes = ['RAW', 'LASCA', 'HISTOGRAM']
        self.mode_combo = wx.ComboBox(panel, choices=modes)
        sizer.Add(self.mode_combo, pos=(3, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.mode_combo.Bind(wx.EVT_COMBOBOX, self.OnModeCombo)
        self.mode_combo.SetSelection(self.selected_mode)

        framescap_ctrl_label = wx.StaticText(panel, label="Video length (sec):")
        sizer.Add(framescap_ctrl_label, pos=(16, 0),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.framescap_ctrl = wx.TextCtrl(panel)
        self.framescap_ctrl.SetValue(str(self.frames_to_capture))
        sizer.Add(self.framescap_ctrl, pos=(16, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)

        self.capture_btn = wx.Button(panel, label="Capture START")
        self.capture_btn.Bind(wx.EVT_BUTTON, self.OnCapture)
        sizer.Add(self.capture_btn, pos=(17, 0), span=(1, 2),
                  flag=wx.EXPAND | wx.ALL, border=5)

        self.framerate_ctrl_label = wx.StaticText(panel, label="Framerate (Hz):")
        sizer.Add(self.framerate_ctrl_label, pos=(4, 0), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.framerate_slider = FS.FloatSpin(panel, -1,  min_val=0, max_val=1,
                                             size=(140, -1), increment=1.0,
                                             value=0.1, agwStyle=FS.FS_LEFT)
        self.framerate_slider.SetFormat("%f")
        self.framerate_slider.SetDigits(2)
        self.framerate_slider.Bind(FS.EVT_FLOATSPIN, self.FramerteSliderScroll)
        sizer.Add(self.framerate_slider, pos=(4, 1), span=(1, 1),
                  flag=wx.ALL, border=5)

        self.exposure_ctrl_label = wx.StaticText(panel, label="Exposure (us):")
        sizer.Add(self.exposure_ctrl_label, pos=(5, 0),  span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.exposure_slider = FS.FloatSpin(panel, -1,  min_val=0, max_val=1,
                                            size=(140, -1), increment=1.0,
                                            value=0.1, agwStyle=FS.FS_LEFT)
        self.exposure_slider.SetFormat("%f")
        self.exposure_slider.SetDigits(2)
        self.exposure_slider.Bind(FS.EVT_FLOATSPIN, self.ExposureSliderScroll)
        sizer.Add(self.exposure_slider, pos=(5, 1), span=(1, 1),
                  flag=wx.ALL, border=5)

        self.gain_ctrl_label = wx.StaticText(panel, label="Gain (dB):")
        sizer.Add(self.gain_ctrl_label, pos=(6, 0), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.gain_slider = FS.FloatSpin(panel, -1,  min_val=0, max_val=1,
                                        size=(140, -1), increment=0.01, value=0.1,
                                        agwStyle=FS.FS_LEFT)
        self.gain_slider.Bind(FS.EVT_FLOATSPIN, self.GainSliderScroll)
        self.gain_slider.SetFormat("%f")
        self.gain_slider.SetDigits(3)
        sizer.Add(self.gain_slider, pos=(6, 1), span=(1, 1),
                  flag=wx.ALL, border=5)

        self.set_auto_exposure = wx.CheckBox(panel, label="Auto Exposure")
        sizer.Add(self.set_auto_exposure, pos=(7, 0), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.set_auto_exposure.SetBackgroundColour(wx.NullColour)
        self.set_auto_exposure.Bind(wx.EVT_CHECKBOX, self.OnEnableAutoExposure)

        self.set_auto_gain = wx.CheckBox(panel, label="Auto Gain")
        sizer.Add(self.set_auto_gain, pos=(7, 1), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.set_auto_gain.SetBackgroundColour(wx.NullColour)
        self.set_auto_gain.Bind(wx.EVT_CHECKBOX, self.OnEnableAutoGain)

        self.set_roi = wx.CheckBox(panel, label="Set ROI")
        sizer.Add(self.set_roi, pos=(15, 3), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.set_roi.SetBackgroundColour(wx.NullColour)
        self.set_roi.Bind(wx.EVT_CHECKBOX, self.OnEnableRoi)

        roi_x_ctrl_label = wx.StaticText(panel, label="Center X:")
        sizer.Add(roi_x_ctrl_label, pos=(16, 3), span=(1, 1),
                  flag=wx.ALL | wx.ALIGN_CENTER, border=0)
        self.roi_x_ctrl = wx.Slider(panel, value=0, minValue=0, maxValue=640,
                                    size=(220, -1),
                                    style=wx.SL_HORIZONTAL | wx.SL_LABELS)
        sizer.Add(self.roi_x_ctrl, pos=(17, 3), span=(1, 1),
                  flag=wx.ALL | wx.ALIGN_CENTER, border=0)
        self.roi_x_ctrl.Bind(wx.EVT_SCROLL, self.OnSetRoiX)

        roi_y_ctrl_label = wx.StaticText(panel, label="Center Y:")
        sizer.Add(roi_y_ctrl_label, pos=(18, 3), span=(1, 1),
                  flag=wx.ALL | wx.ALIGN_CENTER, border=0)
        self.roi_y_ctrl = wx.Slider(panel, value=0, minValue=0, maxValue=640,
                                    size=(220, 20),
                                    style=wx.SL_HORIZONTAL | wx.SL_LABELS)
        sizer.Add(self.roi_y_ctrl, pos=(19, 3), span=(1, 1),
                  flag=wx.ALL | wx.ALIGN_CENTER, border=0)
        self.roi_y_ctrl.Bind(wx.EVT_SCROLL, self.OnSetRoiY)

        roi_width_ctrl_label = wx.StaticText(panel, label="Width:")
        sizer.Add(roi_width_ctrl_label, pos=(16, 4), span=(1, 1),
                  flag=wx.ALL | wx.ALIGN_CENTER, border=0)
        self.roi_width_ctrl = wx.Slider(panel, value=10, minValue=10,
                                        maxValue=640, size=(220, -1),
                                        style=wx.SL_HORIZONTAL | wx.SL_LABELS)
        sizer.Add(self.roi_width_ctrl, pos=(17, 4), span=(1, 1),
                  flag=wx.ALL | wx.ALIGN_CENTER, border=0)
        self.roi_width_ctrl.Bind(wx.EVT_SCROLL, self.OnSetRoiWidth)

        roi_height_ctrl_label = wx.StaticText(panel, label="Height:")
        sizer.Add(roi_height_ctrl_label, pos=(18, 4), span=(1, 1),
                  flag=wx.ALL | wx.ALIGN_CENTER, border=0)
        self.roi_height_ctrl = wx.Slider(panel, value=10, minValue=10,
                                         maxValue=640, size=(220, 20),
                                         style=wx.SL_HORIZONTAL | wx.SL_LABELS)
        sizer.Add(self.roi_height_ctrl, pos=(19, 4), span=(1, 1),
                  flag=wx.ALL | wx.ALIGN_CENTER, border=0)
        self.roi_height_ctrl.Bind(wx.EVT_SCROLL, self.OnSetRoiHeight)
        

        self.roi_x_ctrl.Disable()
        self.roi_y_ctrl.Disable()
        self.roi_width_ctrl.Disable()
        self.roi_height_ctrl.Disable()

        exportfile_ctrl_label = wx.StaticText(panel, label="Export file name:")
        sizer.Add(exportfile_ctrl_label, pos=(8, 0), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.exportfile_ctrl = wx.TextCtrl(panel)
        sizer.Add(self.exportfile_ctrl, pos=(8, 1), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)

        exportfolder_ctrl_label = wx.StaticText(panel, label="Export directory:")
        sizer.Add(exportfolder_ctrl_label, pos=(9, 0), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)

        self.select_folder_btn = wx.Button(panel, label="Select folder")
        self.select_folder_btn.Bind(wx.EVT_BUTTON, self.OnSelectFolder)
        sizer.Add(self.select_folder_btn, pos=(9, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)

        self.exportfolder_ctrl = wx.TextCtrl(panel)
        sizer.Add(self.exportfolder_ctrl, pos=(10, 0), span=(1, 2),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.exportfolder_ctrl.Disable()

        self.append_date = wx.CheckBox(panel, label="Append date and time")
        sizer.Add(self.append_date, pos=(11, 0), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.append_date.SetBackgroundColour(wx.NullColour)
        self.append_date.Bind(wx.EVT_CHECKBOX, self.OnAppendDate)

        self.auto_index = wx.CheckBox(panel, label="Auto index")
        sizer.Add(self.auto_index, pos=(12, 0), span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.auto_index.SetBackgroundColour(wx.NullColour)
        self.auto_index.Bind(wx.EVT_CHECKBOX, self.OnAutoIndex)

        self.index_ctrl = wx.TextCtrl(panel)
        self.index_ctrl.SetValue(str(1))
        sizer.Add(self.index_ctrl, pos=(12, 1), flag=wx.EXPAND | wx.ALL, border=5)

        self.current_state = wx.StaticText(panel, label="Cuttent state: idle")
        sizer.Add(self.current_state, pos=(18, 0), span=(1, 2),
                  flag=wx.EXPAND | wx.ALL, border=5)

        self.frame = np.zeros([640, 640, 3], dtype=np.uint8)
        self.frame[:] = 255

        self.Window = ImagePanel(panel)
        self.Window.SetSize = (640, 640)
        self.Window.Fit()
        sizer.Add(self.Window, pos=(0, 3), span=(15, 4),
                  flag=wx.LEFT | wx.TOP | wx.EXPAND, border=5)

        lasca_filter_label = wx.StaticText(panel, label="LASCA filter size:")
        sizer.Add(lasca_filter_label, pos=(16, 5),
                  flag=wx.EXPAND | wx.ALL, border=5)
        modes = ['3x3', '5x5', '7x7', '9x9', '11x11']
        self.lasca_combo = wx.ComboBox(panel, choices=modes)
        sizer.Add(self.lasca_combo, pos=(17, 5), flag=wx.ALL, border=5)
        self.lasca_combo.Bind(wx.EVT_COMBOBOX, self.OnLascaCombo)
        self.lasca_combo.SetSelection(2)

        self.max_contrast_label = wx.StaticText(panel, label="Max contrast:")
        sizer.Add(self.max_contrast_label, pos=(18, 5),  span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.contrast_slider = FS.FloatSpin(panel, -1,  min_val=0.01,
                                            max_val=1, size=(140, -1),
                                            increment=0.01, value=0.8,
                                            agwStyle=FS.FS_LEFT)
        self.contrast_slider.SetFormat("%f")
        self.contrast_slider.SetDigits(2)
        self.contrast_slider.Bind(FS.EVT_FLOATSPIN, self.ContrastSliderScroll)
        sizer.Add(self.contrast_slider, pos=(19, 5), span=(1, 1),
                  flag=wx.ALL, border=5)

        self.min_gray_label = wx.StaticText(panel, label="Min gray:")
        sizer.Add(self.min_gray_label, pos=(16, 6),  span=(1, 1),
                  flag=wx.EXPAND | wx.ALL, border=5)
        self.gray_slider = FS.FloatSpin(panel, -1,  min_val=0,
                                        max_val=255, size=(140, -1),
                                        increment=1, value=5,
                                        agwStyle=FS.FS_LEFT)
        self.gray_slider.SetFormat("%f")
        self.gray_slider.SetDigits(2)
        self.gray_slider.Bind(FS.EVT_FLOATSPIN, self.GraySliderScroll)
        sizer.Add(self.gray_slider, pos=(17, 6), span=(1, 1), flag=wx.ALL,  border=5)

        self.preview_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.Draw, self.preview_timer)

        self.capture_status_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.capture_status, self.capture_status_timer)

        self.capture_sequence_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.count_elapsed, self.capture_sequence_timer)

        self.border = wx.BoxSizer()
        self.border.Add(sizer, 1, wx.ALL | wx.EXPAND, 20)

        panel.SetSizerAndFit(self.border)
        self.Fit()
        self.preview_thread_obj = threading.Thread(target=self.preview_thread)
        self.capture_thread_obj = threading.Thread(target=self.capture_thread)
        self.EnableGUI(False)


    def AllocateMemory(self):
        self.frame_width = 640
        self.frame_height = 640
        # self.camera.Width = self.frame_width
        # self.camera.Height = self.frame_height
        self.gray = np.zeros((self.frame_height, self.frame_width), np.uint8)
        self.mean_img_sq = np.zeros((self.frame_height, self.frame_width), np.float32)
        self.sq = np.zeros((self.frame_height, self.frame_width), np.float32)
        self.img = np.zeros((self.frame_height, self.frame_width), np.float32)
        self.mean_img = np.zeros((self.frame_height, self.frame_width), np.float32)
        self.sq_img_mean = np.zeros((self.frame_height, self.frame_width), np.float32)
        self.std = np.zeros((self.frame_height, self.frame_width), np.float32)
        self.LASCA = np.zeros((self.frame_height, self.frame_width), np.uint8)
        self.im_color = np.zeros((self.frame_height, self.frame_width, 3), np.uint8)
        self.mask = np.zeros((self.frame_height, self.frame_width), bool)

    def CalculateLASCA(self):
        self.img = cv2.cvtColor(self.frame, code = cv2.COLOR_BGR2GRAY)
        self.img = self.img.astype(np.float32, copy=False)
        cv2.filter2D(self.img, dst=self.mean_img, ddepth=cv2.CV_32F,
                     kernel=self.kernel)
        np.multiply(self.mean_img, self.mean_img, out=self.mean_img_sq)
        np.multiply(self.img, self.img, out=self.sq)
        cv2.filter2D(self.sq, dst=self.sq_img_mean, ddepth=cv2.CV_32F,
                     kernel=self.kernel)
        cv2.subtract(self.sq_img_mean, self.mean_img_sq, dst=self.std)
        cv2.sqrt(self.std, dst=self.std)
        self.mask = self.mean_img < self.min_gray_val
        cv2.pow(self.mean_img, power=-1.0, dst=self.mean_img)
        cv2.multiply(self.std, self.mean_img, dst=self.mean_img,
                     scale=255.0/self.max_contrast, dtype=cv2.CV_32F)
        self.mean_img[self.mean_img > 255.0] = 255.0
        self.LASCA = self.mean_img.astype(np.uint8)
        self.LASCA = 255 - self.LASCA
        self.LASCA[self.mask] = 0
        cv2.filter2D(self.LASCA, dst=self.LASCA, ddepth=cv2.CV_8U, kernel=self.kernel)

    def Draw(self, evt):

        self.lock.acquire()
        if (self.selected_mode == 0):
            self.im_color = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            # cv2.imwrite('test.png', self.frame)
            if self.roi_on is True:
                cv2.line(self.im_color, (self.roi_x, self.roi_y),
                         (self.roi_x + self.roi_width, self.roi_y), (255, 0, 0), 3)
                cv2.line(self.im_color, (self.roi_x, self.roi_y + self.roi_height),
                         (self.roi_x + self.roi_width, self.roi_y + self.roi_height),
                         (255, 0, 0), 3)
                cv2.line(self.im_color, (self.roi_x, self.roi_y),
                         (self.roi_x, self.roi_y + self.roi_height), (255, 0, 0), 3)
                cv2.line(self.im_color, (self.roi_x + self.roi_width, self.roi_y),
                         (self.roi_x + self.roi_width, self.roi_y + self.roi_height),
                         (255, 0, 0), 3)

                self.update_ratio += 1

                if self.update_ratio > 5:
                    sd = self.frame[self.roi_y:(self.roi_y+self.roi_height),
                                    self.roi_x:(self.roi_x+self.roi_width)].std()
                    mn = self.frame[self.roi_y:(self.roi_y+self.roi_height),
                                    self.roi_x:(self.roi_x+self.roi_width)].mean()
                    self.output_string_1 = "C: " + str(np.round(sd / mn, 3))
                    self.output_string_2 = "M: " + str(np.round(mn, 3))
                    self.update_ratio = 0

                cv2.putText(self.im_color, self.output_string_1,
                            (self.roi_x+5, self.roi_y-23),
                            self.font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                cv2.putText(self.im_color, self.output_string_2,
                            (self.roi_x+5, self.roi_y-9),
                            self.font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

            # self.bitmap = wx.Image(self.frame_width, self.frame_height,
            #                        self.im_color.tobytes()).ConvertToBitmap()

        if (self.selected_mode == 1):
            self.CalculateLASCA()
            cv2.applyColorMap(self.LASCA, dst=self.im_color, colormap=cv2.COLORMAP_JET)
            cv2.cvtColor(src=self.im_color, code=cv2.COLOR_BGR2RGB, dst=self.im_color)

            if self.roi_on is True:
                cv2.line(self.im_color, (self.roi_x, self.roi_y),
                         (self.roi_x + self.roi_width, self.roi_y), (255, 0, 0), 3)
                cv2.line(self.im_color, (self.roi_x, self.roi_y + self.roi_height),
                         (self.roi_x + self.roi_width, self.roi_y + self.roi_height),
                         (255, 0, 0), 3)
                cv2.line(self.im_color, (self.roi_x, self.roi_y),
                         (self.roi_x, self.roi_y + self.roi_height), (255, 0, 0), 3)
                cv2.line(self.im_color, (self.roi_x + self.roi_width, self.roi_y),
                         (self.roi_x + self.roi_width, self.roi_y + self.roi_height),
                         (255, 0, 0), 3)

            # self.bitmap = wx.Image(self.frame_width,
            #                        self.frame_height,
            #                        self.im_color.tobytes()).ConvertToBitmap()

        if (self.selected_mode == 2):

            if self.roi_on is True:
                roi_min_row = self.roi_y
                roi_max_row = self.roi_y + self.roi_height
                roi_min_col = self.roi_x
                roi_max_col = self.roi_x + self.roi_width
                self.im_color = self.DrawHistogram(self.frame[roi_min_row:roi_max_row,
                                                              roi_min_col:roi_max_col],
                                                   (self.frame_width, self.frame_height),
                                                   (255, 255, 255),
                                                   (250, 155, 0))
            else:
                self.im_color = self.DrawHistogram(self.frame,
                                                   (self.frame_width, self.frame_height),
                                                   (255, 255, 255),
                                                   (250, 155, 0))

            # self.bitmap = wx.Image(self.frame_width, self.frame_height,
            #                        self.im_color.tobytes()).ConvertToBitmap()
        input_img = cv2.resize(self.im_color, (640, 640), cv2.INTER_CUBIC)
        self.bitmap = wx.Image(self.frame_width, self.frame_height,
                                   input_img.tobytes()).ConvertToBitmap()
        self.Window.update(self.bitmap)

        if self.preview_on is True:
            self.preview_timer.Start(50, oneShot=True)

        self.lock.release()

    def EnableGUI(self, value):
        if value is True:
            self.interval_ctrl.Enable()
            self.sequence_ctrl.Enable()
            self.framescap_ctrl.Enable()
            self.framerate_slider.Enable()
            self.exposure_slider.Enable()
            self.gain_slider.Enable()
            self.exportfile_ctrl.Enable()
            self.cam_combo.Disable()
            self.capmode_combo.Enable()
            self.preview_btn.Enable()
            self.set_roi.Enable()
            self.select_folder_btn.Enable()
            self.capture_btn.Enable()
            self.append_date.Enable()
            self.set_auto_gain.Enable()
            self.set_auto_exposure.Enable()
            self.auto_index.Enable()
            self.index_ctrl.Enable()

            if self.roi_on is True:
                self.roi_x_ctrl.Enable()
                self.roi_y_ctrl.Enable()
                self.roi_width_ctrl.Enable()
                self.roi_height_ctrl.Enable()

            if self.auto_exposure_on is True:
                self.set_auto_exposure.Enable()
                self.exposure_slider.Disable()
            else:
                self.exposure_slider.Enable()

            if self.auto_gain_on is True:
                self.set_auto_exposure.Enable()
                self.gain_slider.Disable()
            else:
                self.gain_slider.Enable()

            if self.auto_index_on is True:
                self.auto_index.Enable()
                self.index_ctrl.Disable()
            else:
                self.index_ctrl.Enable()

            return
        else:
            self.interval_ctrl.Disable()
            self.sequence_ctrl.Disable()
            self.framescap_ctrl.Disable()
            self.framerate_slider.Disable()
            self.exposure_slider.Disable()
            self.gain_slider.Disable()
            self.exportfile_ctrl.Disable()
            self.capmode_combo.Disable()
            self.preview_btn.Disable()
            self.roi_x_ctrl.Disable()
            self.roi_y_ctrl.Disable()
            self.roi_width_ctrl.Disable()
            self.roi_height_ctrl.Disable()
            self.set_roi.Disable()
            self.select_folder_btn.Disable()
            self.capture_btn.Disable()
            self.append_date.Disable()
            self.set_auto_gain.Disable()
            self.set_auto_exposure.Disable()
            self.auto_index.Disable()
            self.index_ctrl.Disable()
            return

    def BlockGUI(self, value):
        if value is True:
            self.interval_ctrl.Enable()
            self.sequence_ctrl.Enable()
            self.framescap_ctrl.Enable()
            self.framerate_slider.Enable()
            self.exportfile_ctrl.Enable()
            self.mode_combo.Enable()
            self.capmode_combo.Enable()
            self.preview_btn.Enable()
            self.set_roi.Enable()
            self.select_folder_btn.Enable()
            self.append_date.Enable()
            self.connect_btn.Enable()

            if self.roi_on is True:
                self.roi_x_ctrl.Enable()
                self.roi_y_ctrl.Enable()
                self.roi_width_ctrl.Enable()
                self.roi_height_ctrl.Enable()

            if self.auto_exposure_on is True:
                self.set_auto_exposure.Enable()
                self.exposure_slider.Disable()
            else:
                self.exposure_slider.Enable()

            if self.auto_gain_on is True:
                self.set_auto_exposure.Enable()
                self.gain_slider.Disable()
            else:
                self.gain_slider.Enable()

            if self.auto_index_on is True:
                self.auto_index.Enable()
                self.index_ctrl.Disable()
            else:
                self.index_ctrl.Enable()

            return
        else:
            self.interval_ctrl.Disable()
            self.sequence_ctrl.Disable()
            self.framescap_ctrl.Disable()
            self.framerate_slider.Disable()
            self.exposure_slider.Disable()
            self.gain_slider.Disable()
            self.exportfile_ctrl.Disable()
            self.mode_combo.Disable()
            self.capmode_combo.Disable()
            self.preview_btn.Disable()
            self.roi_x_ctrl.Disable()
            self.roi_y_ctrl.Disable()
            self.roi_width_ctrl.Disable()
            self.roi_height_ctrl.Disable()
            self.set_roi.Disable()
            self.select_folder_btn.Disable()
            self.append_date.Disable()
            self.connect_btn.Disable()
            self.set_auto_gain.Disable()
            self.set_auto_exposure.Disable()
            self.auto_index.Disable()
            self.index_ctrl.Disable()
        return

    def OnConnect(self, event):

        if self.camera_connected is False:
            tlFactory = pylon.TlFactory.GetInstance()
            devices = tlFactory.EnumerateDevices()
            if len(devices) == 0:
                raise pylon.RuntimeException("No camera present.")

            device_name = self.cameras_list[self.selected_camera]["name"]
            device_serial = self.cameras_list[self.selected_camera]["serial"]

            for i, device in enumerate(devices):

                if device.GetModelName() == device_name:
                    if device.GetSerialNumber() == device_serial:

                        self.camera = pylon.InstantCamera(tlFactory.CreateDevice(devices[i]))
                        self.camera.Open()
                        # self.camera.MaxNumBuffer = 180
                        self.camera.PixelFormat = "BGR8"  # Đặt định dạng pixel để nhận ảnh màu
                        # self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
                        self.camera.AcquisitionFrameRateEnable.SetValue(True)
                        self.camera.AcquisitionFrameRate.SetValue(30.0)
                        resulting_framerate = self.camera.ResultingFrameRate.GetValue()
                        if (resulting_framerate != self.framerate):
                            self.framerate = resulting_framerate
                            self.framerate_slider.SetValue(self.framerate)

                        self.camera.GainAuto.SetValue("Off")
                        self.camera.ExposureAuto.SetValue("Off")

                        self.exposure_slider.SetMax(self.camera.ExposureTime.Max)
                        self.exposure_slider.SetMin(self.camera.ExposureTime.Min)
                        self.exposure_slider.SetValue(self.camera.ExposureTime.Value)
                        self.exposure = self.camera.ExposureTime.Value

                        self.gain_slider.SetMax(self.camera.Gain.Max)
                        self.gain_slider.SetMin(self.camera.Gain.Min)
                        self.gain_slider.SetValue(self.camera.Gain.Value)
                        self.gain = self.camera.Gain.Value

                        self.framerate_slider.SetMax(self.camera.AcquisitionFrameRate.Max)
                        self.framerate_slider.SetMin(self.camera.AcquisitionFrameRate.Min)
                        self.framerate_slider.SetValue(self.camera.AcquisitionFrameRate.Value)
                        self.framerate = self.camera.AcquisitionFrameRate.Value

                        self.connect_btn.SetLabel("Disconnect")
                        self.refresh_btn.Disable()
                        self.cam_combo.Disable()
                        self.camera_connected = True

                        self.AllocateMemory()
                        self.EnableGUI(True)
                        return

        else:
            self.StopPreview()
            self.camera.Close()
            self.connect_btn.SetLabel("Connect")
            self.refresh_btn.Enable()
            self.cam_combo.Enable()
            self.camera_connected = False
            self.EnableGUI(False)
            return

    def OnCloseWindow(self, event):

        self.StopPreview()
        self.capture_on = False
        self.StopCapture()

        if self.camera_connected is True:
            self.camera.Close()

        self.Destroy()
        # print("Closing BaslerGUI")
        return

    def OnEnableAutoExposure(self, event):
        if self.camera_connected is True:
            self.auto_exposure_on = self.set_auto_exposure.GetValue()
            if self.auto_exposure_on is True:
                self.camera.ExposureAuto.SetValue("Continuous")
                self.exposure_slider.Disable()
            else:
                self.camera.ExposureAuto.SetValue("Off")
                self.exposure_slider.SetValue(self.camera.ExposureTime.Value)
                self.exposure_slider.Enable()

    def OnEnableAutoGain(self, event):
        if self.camera_connected is True:
            self.auto_gain_on = self.set_auto_gain.GetValue()
            if self.auto_gain_on is True:
                self.camera.GainAuto.SetValue("Continuous")
                self.gain_slider.Disable()
            else:
                self.camera.GainAuto.SetValue("Off")
                self.gain_slider.SetValue(self.camera.Gain.Value)
                self.gain_slider.Enable()

    def OnRefreshList(self, event):

        if self.camera_connected is False:
            self.selected_camera = 0
            self.cam_combo.Clear()
            tlFactory = pylon.TlFactory.GetInstance()
            devices = tlFactory.EnumerateDevices()
            if len(devices) == 0:
                wx.MessageBox('No camera device. Please provide input device!', 'Warning',
                          wx.OK | wx.ICON_WARNING)
                raise pylon.RuntimeException("No camera present.")

            self.cameras_list.clear()
            for device in devices:
                self.cam_combo.Append(device.GetModelName() +
                                      "_" + device.GetSerialNumber())
                self.cameras_list.append({"name": device.GetModelName(),
                                          "serial": device.GetSerialNumber()})
            self.cam_combo.SetSelection(self.selected_camera)

    def OnPreview(self, event):
        if self.camera_connected is True:
            if self.preview_on is True:
                self.StopPreview()
            else:
                self.StartPreview()
    
    ###############################
    def OnSaveImage(self, event):
        if self.camera_connected is True:
            self.SaveImage()
    ###############################

    def OnCapture(self, event):
        if self.current_step == 0:
            if self.capture_on is False:
                self.StartCapture()
                self.capture_btn.SetLabel("Capture STOP")
            else:
                self.capture_on = False
                self.current_step = 0
                self.StopCapture()
                self.capture_btn.SetLabel("Capture START")
                self.current_state.SetLabel("Current state: idle")
                self.connect_btn.Enable()
                self.StartPreview()
        else:
            self.current_step = 0
            self.StopCapture()
            self.capture_btn.SetLabel("Capture START")
            self.current_state.SetLabel("Current state: idle")
            self.connect_btn.Enable()

    def FramerteSliderScroll(self, event):
        obj = event.GetEventObject()
        val = obj.GetValue()
        self.framerate = val
        if self.camera_connected is True:
            self.camera.AcquisitionFrameRate.SetValue(self.framerate)
            resulting_framerate = self.camera.ResultingFrameRate.GetValue()
            if (resulting_framerate != self.framerate):
                self.framerate = resulting_framerate
                self.framerate_slider.SetValue(self.framerate)

    def ExposureSliderScroll(self, event):
        obj = event.GetEventObject()
        val = obj.GetValue()
        self.exposure = val
        if self.camera_connected is True:
            self.camera.ExposureTime.SetValue(self.exposure)
            resulting_framerate = self.camera.ResultingFrameRate.GetValue()
            if (resulting_framerate != self.framerate):
                self.framerate = resulting_framerate
                self.framerate_slider.SetValue(self.framerate)

    def OnAutoIndex(self, event):
        if self.camera_connected is True:
            self.auto_index_on = self.auto_index.GetValue()
            if self.auto_index_on is True:
                self.index_ctrl.Disable()
                self.current_index = int(self.index_ctrl.GetValue())
            else:
                self.index_ctrl.Enable()

    def ContrastSliderScroll(self, event):
        obj = event.GetEventObject()
        val = obj.GetValue()
        self.max_contrast = val

    def GraySliderScroll(self, event):
        obj = event.GetEventObject()
        val = obj.GetValue()
        self.min_gray_val = val

    def GainSliderScroll(self, event):
        obj = event.GetEventObject()
        val = obj.GetValue()
        self.gain = val
        if self.camera_connected is True:
            self.camera.Gain.SetValue(self.gain)

    def OnLascaCombo(self, event):
        current_selection = self.lasca_combo.GetSelection()
        filter_size = int(2*(current_selection+2) - 1)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (filter_size, filter_size)).astype(np.float32)
        self.kernel /= np.sum(self.kernel)

    def OnCamCombo(self, event):
        self.selected_camera = self.cam_combo.GetSelection()

    def OnModeCombo(self, event):
        self.selected_mode = self.mode_combo.GetSelection()

    def OnCapModeCombo(self, event):
        self.capture_mode = self.capmode_combo.GetSelection()

    def OnSelectFolder(self, event):
        dlg = wx.DirDialog(None, "Choose input directory", "",
                           wx.DD_DEFAULT_STYLE | wx.DD_DIR_MUST_EXIST)
        dlg.ShowModal()
        self.exportfolder_ctrl.SetValue(dlg.GetPath())

    def GetHistogram(self, image):
        hist_full = cv2.calcHist([image], [0], None, [256], [0, 256])
        max_val = np.max(hist_full)
        if max_val > 0:
            hist_full = (hist_full / np.max(hist_full))*100
        else:
            hist_full = np.zeros((256, 1))
        return hist_full

    def DrawHistogram(self, image, size, bcg_color, bin_color):
        histogram_data = self.GetHistogram(image)
        histogram_image = np.ones((256, 256, 3), np.uint8)*240
        R, G, B = bcg_color
        histogram_image[:, :, 0] = B
        histogram_image[:, :, 1] = G
        histogram_image[:, :, 2] = R

        for column in range(0, len(histogram_data)):
            column_height = int(np.floor((histogram_data[column]/100)*256))
            if column_height > 1:
                R, G, B = bin_color
                color = (B, G, R)
                cv2.line(histogram_image, (column, 255),
                         (column, 255-column_height), color, 1)

        resized = cv2.resize(histogram_image, size, interpolation=cv2.INTER_AREA)
        return resized

    def OnAppendDate(self, event):
        self.append_date_flag = self.append_date.GetValue()

    def OnEnableRoi(self, event):
        self.roi_on = self.set_roi.GetValue()
        if self.roi_on is True:
            self.roi_x_ctrl.Enable()
            self.roi_y_ctrl.Enable()
            self.roi_width_ctrl.Enable()
            self.roi_height_ctrl.Enable()
        else:
            self.roi_x_ctrl.Disable()
            self.roi_y_ctrl.Disable()
            self.roi_width_ctrl.Disable()
            self.roi_height_ctrl.Disable()

    def OnSetRoiX(self, event):
        new_roi_x = self.roi_x_ctrl.GetValue()
        if (new_roi_x + self.roi_width) < self.frame_width:
            self.roi_x = new_roi_x
        else:
            self.roi_x_ctrl.SetValue(self.roi_x)

    def OnSetRoiY(self, event):
        new_roi_y = self.roi_y_ctrl.GetValue()
        if (new_roi_y + self.roi_height) < self.frame_height:
            self.roi_y = new_roi_y
        else:
            self.roi_y_ctrl.SetValue(self.roi_y)

    def OnSetRoiWidth(self, event):
        new_roi_width = self.roi_width_ctrl.GetValue()
        if (self.roi_x + new_roi_width) < self.frame_width:
            self.roi_width = new_roi_width
        else:
            self.roi_width_ctrl.SetValue(self.roi_width)

    def OnSetRoiHeight(self, event):
        new_roi_height = self.roi_height_ctrl.GetValue()
        if (self.roi_y + new_roi_height) < self.frame_height:
            self.roi_height = new_roi_height
        else:
            self.roi_height_ctrl.SetValue(self.roi_height)

    def StartPreview(self):
        self.preview_on = True
        self.preview_thread_obj = threading.Thread(target=self.preview_thread)
        self.preview_thread_obj.start()
        self.preview_timer.Start(100, oneShot=True)
        self.preview_btn.SetLabel("Preview STOP")

    def StopPreview(self):
        self.preview_on = False
        if self.preview_thread_obj.is_alive() is True:
            self.preview_thread_obj.join()
        self.preview_timer.Stop()
        self.preview_btn.SetLabel("Preview START")

    def preview_thread(self):
        self.camera.StartGrabbing(pylon.GrabStrategy_OneByOne)
        self.previous_time = int(round(time.time() * 1000))

        while self.preview_on is True:
            if self.camera.IsGrabbing():
                grabResult = self.camera.RetrieveResult(5000,
                                                        pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    current_time = int(round(time.time() * 1000))
                    if ((current_time - self.previous_time) > 20):
                        self.lock.acquire()
                        self.frame = grabResult.GetArray()
                        self.lock.release()
                        self.previous_time = current_time
                else:
                    print("Error: Memory Temp", grabResult.ErrorCode)
                grabResult.Release()

        self.camera.StopGrabbing()

    def StartCapture(self):
        self.StopPreview()
        self.capture_thread_obj = threading.Thread(target=self.capture_thread)
        self.capture_thread_obj.start()
        self.EnableGUI(False)
        self.connect_btn.Disable()
        self.capture_btn.Enable()
        self.capture_status_timer.Start(200, oneShot=True)

    def StopCapture(self):
        if self.capture_thread_obj.is_alive() is True:
            self.capture_thread_obj.join()
        self.EnableGUI(True)
        self.capture_status_timer.Stop()
        self.capture_sequence_timer.Stop()

    def capture_thread(self):
        self.capture_on = True
        self.last_capture_time = time.time()
        sequence_length = int(self.sequence_ctrl.GetValue())
        video_length = float(self.framescap_ctrl.GetValue())
        frames_to_capture = int(video_length * self.framerate)
        interval_length = float(self.interval_ctrl.GetValue())

        output_path = []
        output_file_name = self.exportfile_ctrl.GetValue()
        output_folder_name = self.exportfolder_ctrl.GetValue()
        if len(output_folder_name) > 0:
            output_path = output_folder_name + "\\" + output_file_name
        else:
            output_path = output_file_name

        if len(output_file_name) <= 1:
            wx.MessageBox('Please provide output file name!', 'Warning',
                          wx.OK | wx.ICON_WARNING)
            self.capture_on = False
            self.current_step = sequence_length
            return

        if len(output_folder_name) <= 1:
            wx.MessageBox('Please provide output folder!', 'Warning',
                          wx.OK | wx.ICON_WARNING)
            self.capture_on = False
            self.current_step = sequence_length
            return

        if self.append_date_flag is True:
            if self.capture_mode != 2 and self.capture_mode != 4:
                output_path = output_path + "_" + time.strftime("%Y%m%d_%H%M%S")

        if self.auto_index_on is True:
            if self.capture_mode != 2 and self.capture_mode != 4:
                output_path = output_path + "_" + str(self.current_index)
                self.current_index += 1

        if self.auto_index_on is False and self.append_date_flag is False:
            if sequence_length > 1:
                if self.capture_mode != 2:
                    wx.MessageBox('Turn on auto indexing or append date to' +
                                  ' file name when capturing sequence!',
                                  'Warning', wx.OK | wx.ICON_WARNING)
                    self.capture_on = False
                    self.current_step = sequence_length
                    return

        if len(output_path) <= 4:
            wx.MessageBox('Invalid name for data output file!',
                          'Warning', wx.OK | wx.ICON_WARNING)
            self.capture_on = False
            self.current_step = sequence_length
            return

        if sequence_length < 1:
            wx.MessageBox('Invalid length of measurement sequence! Minimum' +
                          ' required value is 1.',
                          'Warning', wx.OK | wx.ICON_WARNING)
            self.capture_on = False
            return
        if sequence_length > 1:
            if(video_length > interval_length):
                wx.MessageBox('Interval length should be greater than video length',
                              'Warning', wx.OK | wx.ICON_WARNING)
                self.capture_on = False
                return

        if frames_to_capture < 1:
            wx.MessageBox('Invalid number of frames to capture! Minimum' +
                          ' required value is 5 frames.',
                          'Warning', wx.OK | wx.ICON_WARNING)
            self.capture_on = False
            self.current_step = sequence_length
            return

        width = 0
        height = 0
        roi_min_row = 0
        roi_max_row = self.frame_height - 1
        roi_min_col = 0
        roi_max_col = self.frame_width - 1
        if self.roi_on is True:
            roi_min_row = self.roi_y
            roi_max_row = self.roi_y + self.roi_height
            roi_min_col = self.roi_x
            roi_max_col = self.roi_x + self.roi_width
            width = self.roi_width
            height = self.roi_height
        else:
            width = self.frame_width
            height = self.frame_height

        if self.capture_mode == 3:
            frames_to_capture = 1

        buffer = np.zeros((frames_to_capture, height, width), np.uint8)
        self.camera.StartGrabbingMax(frames_to_capture, pylon.GrabStrategy_OneByOne)

        current_date_and_time = str(datetime.datetime.now())

        print(f'Capturing video started at: {current_date_and_time}')

        captured_frames = 0
        while captured_frames < frames_to_capture:
            if self.camera.IsGrabbing():
                grabResult = self.camera.RetrieveResult(5000,
                                                        pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    if self.roi_on is True:
                        buffer[captured_frames, :, :] = grabResult.GetArray()[roi_min_row:roi_max_row, 
                                                                              roi_min_col:roi_max_col].copy()
                    else:
                        buffer[captured_frames, :, :] = grabResult.GetArray().copy()
                    captured_frames += 1
                else:
                    print("Error: ",
                          grabResult.ErrorCode)
                grabResult.Release()
            if self.capture_on is False:
                self.camera.StopGrabbing()
                self.current_step = sequence_length
                return

        self.camera.StopGrabbing()
        print(f'Capturing finished after grabbing {captured_frames} frames')

        if self.process_thread_obj is not None:
            if self.process_thread_obj.is_alive():
                self.process_thread_obj.join()

        self.process_thread_obj = threading.Thread(target=self.process_data, args=(
            buffer.copy(), self.capture_mode, output_path, current_date_and_time))
        self.process_thread_obj.start()

        self.capture_on = False

    def process_data(self, buffer, capture_mode, output_path, current_date_and_time):

        frames_to_capture, height, width = buffer.shape
        date_time = time.strftime("%Y%m%d_%H%M%S")
        if capture_mode == 0:
            video_output_path = output_path + ".mp4"
            print(f'Writing raw output video to {output_path}')
            video_writer = cv2.VideoWriter(video_output_path,
                                           cv2.VideoWriter_fourcc(*'RGBA'),
                                           self.framerate,
                                           (width, height), isColor=False)
            for frame_index in range(0, frames_to_capture):
                video_writer.write(buffer[frame_index, :, :])

            video_writer.release()
            print("Video data written")

        if capture_mode == 1:
            output_path = output_path + ".mp4"
            print(f'Writing LASCA output video to {output_path}')
            video_writer = cv2.VideoWriter(output_path,
                                           cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                                           self.framerate,
                                           (width, height))
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)).astype(np.float32)
            kernel /= np.sum(kernel)
            for frame_index in range(0, frames_to_capture):
                img = buffer[frame_index, :, :].copy().astype(np.float32, copy=False)
                mean_img = cv2.filter2D(img, ddepth=cv2.CV_32F, kernel=kernel)
                mean_img_sq = np.multiply(mean_img, mean_img)
                sq = np.multiply(img, img)
                sq_img_mean = cv2.filter2D(sq, ddepth=cv2.CV_32F, kernel=kernel)
                std = cv2.subtract(sq_img_mean, mean_img_sq)
                std[std <= 0] = 0.000001
                cv2.sqrt(std, dst=std)
                mask = mean_img < 5
                mean_img[mask] = 10000000.00
                cv2.pow(mean_img, power=-1.0, dst=mean_img)

                LASCA = cv2.multiply(std, mean_img, scale=255, dtype=cv2.CV_8U)
                LASCA = 255 - LASCA
                cv2.filter2D(LASCA, dst=LASCA, ddepth=cv2.CV_8U, kernel=kernel)
                im_color = cv2.applyColorMap(LASCA, colormap=cv2.COLORMAP_JET)
                video_writer.write(im_color)

            video_writer.release()

        if capture_mode == 2 or capture_mode == 4:

            file_name = output_path.split('\\')[-1]
            file_name = file_name + "_" + date_time

            if capture_mode == 4:
                video_output_path = output_path + "_" + date_time + ".mp4"
                print(f'Writing raw output video to {video_output_path}')
                video_writer = cv2.VideoWriter(video_output_path, cv2.VideoWriter_fourcc(*'RGBA'), self.framerate,
                                               (width, height), isColor=False)
                for frame_index in range(0, frames_to_capture):
                    video_writer.write(buffer[frame_index, :, :])
                video_writer.release()

            output_path = output_path + ".csv"
            print(f'Writing output data to {output_path}')

            mean_slasca = 0
            mean_tlasca = 0
            inertia_moment_2 = 0

            buffer_array = np.zeros((width * height, frames_to_capture),
                                    dtype=np.float32, order='C')
            for frame_index in range(0, frames_to_capture):
                buffer_array[:, frame_index] = buffer[frame_index, :, :] \
                    .flatten(order='C')  \
                    .astype(np.float32)
                mean_slasca += np.std(buffer[frame_index, :, :]) / \
                    np.mean(buffer[frame_index, :, :])

            mean_slasca /= frames_to_capture
            mean_tlasca = np.mean(np.std(buffer_array, axis=1) /
                                  np.mean(buffer_array, axis=1))

            buffer_array = buffer_array.astype(np.int32)
            result = graycomatrix(buffer_array, [1], [0], normed=False,
                                  levels=256, symmetric=True)
            com = result[:, :, 0, 0] / frames_to_capture
            distance_matrix = np.tile(np.arange(256), (256, 1))
            distance_matrix = np.abs(distance_matrix - distance_matrix.T) * (np.sqrt(2) / 2)
            inertia_moment_2 = np.sum(com*distance_matrix)
            #
            max_lag = frames_to_capture // 3
            samples = 0
            correlations = np.zeros(max_lag)
            time_scale = np.zeros(max_lag)
            for i in range(0, frames_to_capture-max_lag, 5):
                samples = samples + 1
                for lag in range(0, max_lag):
                    out = np.corrcoef(buffer_array[:, i], buffer_array[:, i + lag])
                    correlations[lag] = correlations[lag] + out.item((0, 1))

            for lag in range(0, max_lag):
                time_scale[lag] = (1/self.framerate)*lag

            correlations = correlations / samples
            def fitfunc(x, a, b): return np.exp(-1*((x/a)**b))

            popt, pcov = curve_fit(fitfunc,
                                   xdata=time_scale,
                                   ydata=correlations,
                                   maxfev=6000,
                                   p0=[1, 1],
                                   bounds=((0.001, 0.001), (1000, 1000)))

            a, b = popt

            file = open(output_path, 'a', newline='', encoding='UTF8')
            writer = csv.writer(file)

            data = [self.current_step, date_time, file_name,
                    mean_slasca, mean_tlasca, inertia_moment_2, a, b,
                    self.framerate, self.exposure, self.gain]
            writer.writerow(data)
            writer = None
            file.close()
            print(f'Data saved in output file: {output_path}')

        if capture_mode == 3:
            output_path = output_path + ".tif"
            print(f'Writing output image to {output_path}')
            cv2.imwrite(output_path, buffer[0, :, :])

        print('\n')

    def capture_status(self, evt):
        if self.capture_on is True:
            self.capture_status_timer.Start(200, oneShot=True)
            self.current_state.SetLabel("Current status: capturing data!")
            return
        else:
            sequence_length = int(self.sequence_ctrl.GetValue())
            if sequence_length == 1:
                self.current_state.SetLabel("Current state: idle")
                self.EnableGUI(True)
                self.connect_btn.Enable()
                self.capture_btn.SetLabel("Capture START")
                self.StartPreview()
                return
            else:
                self.current_step += 1
                if sequence_length > self.current_step:
                    self.capture_btn.SetLabel("Capture STOP")
                    correction = int(time.time() - self.last_capture_time)
                    self.time_to_next = int(self.interval_ctrl.GetValue()) - correction
                    time_to_next_str = str(datetime.timedelta(seconds=self.time_to_next))
                    self.current_state.SetLabel(
                        f"Current status: step {self.current_step} out of" +
                        f"{sequence_length}" +
                        "time to next: " + time_to_next_str)
                    self.capture_sequence_timer.Start(1000, oneShot=True)
                    self.capture_btn.Enable()
                    self.preview_btn.Enable()
                    self.StartPreview()
                    return
                else:
                    if self.index_ctrl.GetValue() == '':
                        self.index_ctrl.SetValue(str(1))
                        self.current_index = 1
                    else:
                        self.current_index = int(self.index_ctrl.GetValue())
                    self.current_step = 0
                    self.capture_btn.SetLabel("Capture START")
                    self.current_state.SetLabel("Current state: idle")
                    self.EnableGUI(True)
                    self.connect_btn.Enable()
                    self.StartPreview()
                    return
        return

    def count_elapsed(self, evt):
        self.time_to_next -= 1
        time_to_next_str = str(datetime.timedelta(seconds=self.time_to_next))
        sequence_length = int(self.sequence_ctrl.GetValue())
        self.current_state.SetLabel(
            f"Current status: step {self.current_step} out of" +
            f"{sequence_length}" +
            "time to next: " + time_to_next_str)
        if self.time_to_next > 0:
            self.capture_sequence_timer.Start(1000, oneShot=True)
        else:
            self.StartCapture()


if __name__ == '__main__':
    app = wx.App()
    ex = BaslerGuiWindow(None)
    app.MainLoop()