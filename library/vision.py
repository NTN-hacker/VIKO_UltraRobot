import os
import os.path as osp
import importlib
import sys
from config import config as CFG
from library import viko_lib as lib


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
    from segment_anything_hq import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
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
        self.MODEL_CONFIG        = CFG.MODEL['HQ_SAM']
        self.PATH_OUTPUT         = f'data/AI/Record_{str(datetime.now())[:10]}'
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
        coordinate = list([])
        # assert len(image_arr.shape) == 2, "image should be 3D and Color space is RGB"
        img_cvt = cv2.cvtColor(image_arr, cv2.COLOR_BGR2RGB)
        img_cvt_cp = img_cvt.copy()
        marks = self.model.generate(img_cvt)
        #example
        for mark in marks:
            if (1200000 < mark['area'] < 2000000):# and (mark['bbox'][2]*mark['bbox'][3] < 1450000):
                coordinate = mark['bbox']
                print(len(marks))
                x, y, w, h = coordinate
                x, y, w, h = int(x), int(y), int(w), int(h)
                # img_crop = img_cvt[y: y+h, x:x+w,  :]
                
                #display image after segment
                cv2.circle(img_cvt_cp, (x, y), color = (255, 0, 0), radius = 50, thickness = 20)
                text = f'({x}, {y})'
                cv2.putText(img = img_cvt_cp, org = (x, y), fontFace = 1, fontScale = 5, text = text, color = (125, 255, 255), thickness = 10)
                cv2.rectangle(img_cvt_cp, (x, y), (x + w, y + h), (0, 255, 125), thickness = 20)
                # cv2.rectangle(img_cvt, ())
                fig = plt.imshow(img_cvt_cp)
                plt.savefig('%s/Box_%s' % (self.outputDir, self.filename))
                self.img = img_cvt_cp.copy()
                coordinate = [x, y, w, h]
                print(mark)
                break

        x, y, w, h = coordinate
        self.img_split = img_cvt[ y: y+h, x:x+w]
        cv2.namedWindow("Point detect", cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Point detect', 600, 600)
        cv2.imshow('Point detect', self.img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print(coordinate)
        return coordinate
    
    def _getCoordinateWeld_(self, img, coordinate_ori) -> tuple:
        image = img.copy()
        try: 
            x_ori, y_ori, w, h = coordinate_ori
            if w<h:
                x = (2*x_ori + w)//2 
                y = y_ori
                x2 = (2*x_ori + w)//2 
                y2 = y_ori+h 
            else:
                x = x_ori
                y = (2*y_ori + h)//2 
                x2 = x_ori + w
                y2 = (2*y_ori + h)//2 
        except: 
            rf = Roboflow(api_key="JtRFLNmuxFdQiNLXfFJj")
            project = rf.workspace().project("weld-detection-slz4d")
            model = project.version(1).model           

            result = model.predict(img, confidence=50, overlap=50).json()
            detections = sv.Detections.from_roboflow(result)
            print(detections)
            coordinate = detections.xyxy[0]
            x, y, x2, y2 = coordinate
        # except:
        
        cv2.rectangle(img = image, pt1= (int(0), int(h//2 -70)), pt2= (int(w), int(h//2 + 90)), color= (0, 255, 0), thickness= 1)
        cv2.rectangle(img = self.img, pt1= (int(x), int(y-70)), pt2= (int(x2), int(y2+90)), color= (0, 255, 0), thickness= 1)
        cv2.circle(self.img, (x, y), color = (255, 0, 0), radius = 10, thickness = 5)
        cv2.namedWindow("Weld detect", cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Weld detect', 600, 600)
        cv2.imshow('Weld detect', self.img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        cv2.imwrite(f'{self.outputDir}/Weld_{self.filename}.png', image)
        # x_ori, y_ori, w, h = coordinate_ori
        # self.img[y_ori:y_ori + h, x_ori:x_ori + w] = image
        cv2.imwrite(f'{self.outputDir}/Weld_Detected.png', self.img)

        coordinate_out = [[x, y], [x2, y2]]

        return coordinate_out
    
    def backgroundSubtraction(self) -> list:
        #Background Image
        img_bg = cv2.imread('lib/BackGround.png', cv2.IMREAD_COLOR)
        img_bg = cv2.cvtColor(img_bg, cv2.COLOR_BGR2RGB)
        #Foreground Image
        imageDict = self.image_list.pop(0)
        img_obj = imageDict['image'] 
        self.filename = imageDict['filename']
        
        #Background Subtraction
        fmask = lib.main(img_obj, img_bg)
        cv2.imwrite(osp.join(self.outputDir, f'Mask_{self.filename}'), fmask)
    
        #Post Processing
        fg, coordinates = lib.boudingBox(img_obj, fgMask= fmask)
        cv2.imwrite(osp.join(self.outputDir, f'Foreground_{self.filename}'), fg)
        # print(f'The information of object {coordinate}, {area}')
        # # Crop Working Space
        # x, y, w, h = coordinate
        # img_crop = img_obj[y: y+h, x: x+w]
        # print(f'Time to process step 1 {datetime.datetime.now() - start_time}')
        # # Test save
        # cv2.imwrite(f'data/boudingbox/{osp.basename(obj_path)}', img_crop)



        #Get coordinates 
        # coordinates = list([])
        # print(coordinates)

        return coordinates
    
    def _getCircle_(self) -> list:
        imageDict = self.image_list.pop(0)
        img_obj = imageDict['image'] 
        self.filename = imageDict['filename']

        x, y = lib.get_point(img_obj)
        
        # lib.save_csv(self.filename, x, y)

        lis = [x, y]
        return lis
    


                