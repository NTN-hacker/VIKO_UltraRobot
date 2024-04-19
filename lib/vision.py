import os
import os.path as osp
import importlib
import sys
from config import config as CFG


try:
    import numpy as np
    import numpy.core.multiarray
    import cv2
    import torch
    import matplotlib.pyplot as plt
    from pypylon import pylon as py
    import threading
    import time
    from datetime import datetime
    from PIL import Image
    from  segment_anything import sam_model_registry, SamPredictor, SamAutomaticMaskGenerator
    from  mobile_sam import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
    from roboflow import Roboflow
    import supervision as sv
except ImportError:
    print('Vision module bindings requires "numpy", "cv2", "torch", "matplotlib.pyplot", "pypylon", "threading", "time", "pillow", "sam", "datetime" package.')
    print('Install it via command:')
    print('    pip install numpy/cv2/matplotlib/pypylon/threading/time/pillow/datetime/roboflow/supervision')
    print('Depend on with your cuda version, this case: pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121')
    print('To install model, we nees install from repo and checkpoint: pip install git+https://github.com/facebookresearch/segment-anything.git')
    print('                                                         or pip install git+https://github.com/ChaoningZhang/MobileSAM.git')
    raise


# TODO

class VisionModule():
    def __init__(self) -> None:
        #Init Transform Layer instance
        tl_factory = py.TlFactory.GetInstance()
        devices    = tl_factory.EnumerateDevices()
        for device in devices:
            print(device.GetModelName(), device.GetSerialNumber())
            self.model_name = device.GetModelName()
        
        self.camera = py.InstantCamera()
        self.camera.Attach(tl_factory.CreateDevice(devices[0]))
        self.runningStatus       = False
        self.thread              = None
        self.image_list          = list()
        self.countImagesIntoGrab = 2
        self.model               = None
        self.MODEL_CONFIG        = CFG.MODEL['SAM']
        self.PATH_OUTPUT         = f'Record_{str(datetime.now())[:10]}'
        if not osp.exists(self.PATH_OUTPUT):
            os.mkdir(self.PATH_OUTPUT)
        self.outputDir   = self.PATH_OUTPUT
        self.filename    = None
    def _start_(self):
        self.camera.Open()
        self.settingParameter()

    def _getImage_(self):
        """
        The funciton use to get images from camera
        """

        self.camera.PixelFormat = "RGB8"
        #count frame
        self.camera.StartGrabbingMax(self.countImagesIntoGrab)

        while self.camera.IsGrabbing():
            grab = self.camera.RetrieveResult(2000, py.TimeoutHandling_Return)

            if grab.GrabSucceeded():
                serial_number = '_'.join([self.camera.GetDeviceInfo().GetModelName(),
                            self.camera.GetDeviceInfo().GetSerialNumber()])
                print('Serial No:%s' %serial_number) 
                imageDict = {}
                imageDict['filename'] = datetime.now().strftime("%Y%m%d-%H%M%S%f-{}.png".format(serial_number))
                imageDict['image'] = grab.GetArray()
            else:
                print("Error: ", grab.ErrorCode, grab.ErrorDescription)
            
            grab.Release()
            self.runningStatus = True

            #Push to queue Image List
            self.image_list.append(imageDict)
        
        self.runningStatus = False

    
    def settingParameter(self):
    ########Trigger 
        self.camera.Width= 2448
        self.camera.Height=2048
        self.camera.ExposureTime = 4000

    def thread_grab(self, target):
        self.thread = threading.Thread(target= target)
        self.thread.start()
    
    def save_image(self):
        totalTime = 0
        # while not self.runningStatus:
        #     time.sleep(0.001)
        # while True:
        #     if not self.runningStatus and len(self.image_list) == 0:
        #         break
        #     if len(self.image_list) == 0:
        #         continue
        #     else:
        imageDict = self.image_list.pop(0)
        img = imageDict['image'] 
        self.filename = imageDict['filename']
        startTime = time.time()  
        im = Image.fromarray(img)
        im.save('%s/%s' % (self.outputDir, self.filename))
        totalTime = totalTime + (time.time() - startTime)
        print('Total time for saving: %f' % totalTime)
        return im
    
    def _end_(self):
        self.camera.Close()
    
    def load_model(self):
        """
        This function to get bounding box
        """

        sam = sam_model_registry[self.MODEL_CONFIG['MODEL_TYPE']](checkpoint= self.MODEL_CONFIG['WEIGHT'])
        sam.to(device= CFG.DEVICE)        
        self.model = SamAutomaticMaskGenerator(sam)

    
    def _getCoordinate_(self):
        image_arr = self.image_list[-1]['image']

        # assert len(image_arr.shape) == 2, "image should be 3D and Color space is RGB"
        img_cvt = cv2.cvtColor(image_arr, cv2.COLOR_BGR2RGB)
        marks = self.model.generate(img_cvt)
        #example
        coordinate = marks[0]['bbox']
        print(len(marks))
        x, y, w, h = coordinate
        # img_crop = img_cvt[y: y+h, x:x+w,  :]
        
        #display image after segment
        img_cvt_draw = img_cvt.copy()
        cv2.circle(img_cvt, (x, y), color = (255, 0, 0), radius = 50, thickness = 20)
        text = f'({x}, {y})'
        cv2.putText(img = img_cvt, org = (x, y), fontFace = 1, fontScale = 5, text = text, color = (125, 255, 255), thickness = 10)
        cv2.rectangle(img_cvt, (x, y), (x + w, y + h), (0, 255, 125), thickness = 20)

        fig = plt.imshow(img_cvt)
        plt.savefig('%s/Box_%s' % (self.outputDir, self.filename))
        return marks
    
    def _getCoordinateWeld_(self, img) -> tuple:
        rf = Roboflow(api_key="JtRFLNmuxFdQiNLXfFJj")
        project = rf.workspace().project("weld-detection-slz4d")
        model = project.version(1).model

        image = img.copy()

        result = model.predict(img, confidence=50, overlap=50).json()

        detections = sv.Detections.from_roboflow(result)

        coordinate = detections.xyxy[0]

        x, y, x2, y2 = coordinate

        cv2.rectangle(img = image, pt1= (int(x), int(y)), pt2= (int(x2), int(y2)), color= (0, 255, 0), thickness= 1)

        cv2.imwrite(f'{self.outputDir}/Weld_{self.filename}', image)
        
        return coordinate
    


                