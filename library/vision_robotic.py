import os
import os.path as osp
import importlib
import sys
from config import config as CFG
from library import viko_lib as lib
import sys
sys.path.append(
    "D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot"
)

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
    from ultralytics import YOLO
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
        self.model               = None
        self.MODEL_CONFIG        = CFG.MODEL['YOLOV9']
        self.PATH_OUTPUT         = f'data/AI/Record_{str(datetime.now())[:10]}'
        if not osp.exists(self.PATH_OUTPUT):
            os.mkdir(self.PATH_OUTPUT)
        self.outputDir   = self.PATH_OUTPUT
        self.filename    = None
    
    
    def load_model(self, model):
        """
        This function to get bounding box
        """
        if self.MODEL_CONFIG == CFG.MODEL['YOLOV9']:
            self.model = model
        else:
            sam = sam_model_registry[self.MODEL_CONFIG['MODEL_TYPE']](checkpoint= self.MODEL_CONFIG['WEIGHT'])
            sam.to(device= CFG.DEVICE)        
            self.model = SamAutomaticMaskGenerator(sam)
    
    def _getCoordinateTest_(self, image):
        coordinate = list([])
        img_cvt_cp = image.copy()
        
        if self.MODEL_CONFIG == CFG.MODEL['YOLOV9']:
            img_resize = cv2.resize(img_cvt_cp, (640, 640), interpolation= cv2.INTER_CUBIC)
            dict_re = self.model.predict(img_resize, save=True, conf=0.65)
            
            img_re = dict_re[0][0].orig_img

            mask = dict_re[0].masks.data.detach().cpu().numpy()
            mask = cv2.normalize(mask, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
            mask = lib.convert_mask(mask)
            cv2.imshow("Mask", mask)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            #display 1
            out = np.hstack([img_re, mask])
            cv2.imshow('Mapping', out)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            #display 2
            coordinate_re = dict_re[0].masks.xy[0]

            img_cp = img_re.copy()
            pts = np.zeros((len(coordinate_re), 2))

            for idx, point in enumerate(coordinate_re):
                cv2.circle(img_cp, (int(point[0]), int(point[1])), 1, (255, 0, 0), 1)
                pts[idx] = [int(point[0]), int(point[1])]
            pts = np.array(pts, np.int32)

            id = 0
            cv2.fillPoly(img_cp, pts=[pts], color=(0, 0, 100 + int(id) * 100, 20))
            cv2.imshow("Image after filly", img_cp)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            #save data
            labels = lib.save_data_predict(dict_re, img_cp)

            # define point
            x_ratio = 2048/640
            y_ratio = 2448/640

            #point end
            coordinate_end, length = lib.find_point_end(pts)
            print(coordinate_end)
            coordinate = [[int(pts[0][0]* y_ratio) , int(pts[0][1]* x_ratio)], 
                          [int(coordinate_end[0]* y_ratio), int(coordinate_end[1]* x_ratio)]]
            cv2.circle(img_cp, (int(pts[0][0]), int(pts[0][1])), color = (255, 0, 0), radius = 10, thickness = 5)
            cv2.circle(img_cp, (int(coordinate_end[0]), int(coordinate_end[1])), color = (255, 255, 0), radius = 10, thickness = 5)
            cv2.imshow("Image after define", img_cp)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return coordinate 
    
    def _getCoordinateSingleobject_(self, image):
        
            if self.MODEL_CONFIG == CFG.MODEL['YOLOV9']:
                #predict
                img_resize = cv2.resize(image, (640, 640), interpolation= cv2.INTER_CUBIC)
                dict_re = self.model.predict(img_resize, save=False, conf=0.65)   

                #image after predict
                plot = dict_re[0].plot() 

                #coordinates
                transformed_coordinates = lib.transform_coordinates(dict_re, tag = 'single')
                print('transformed_coordinates ', transformed_coordinates)

                # img_re = lib.draw_coordinates_on_image(image, transformed_coordinates)
                
                #save data
                # label_out = lib.save_data_scan(dict_re, plot, transformed_coordinates)

                # cv2.imshow("Image after define", cv2.resize(img_re, (600, 600), cv2.INTER_CUBIC))
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

            return transformed_coordinates    

    def _getCoordinateMultiobject_(self, image):
    
        if self.MODEL_CONFIG == CFG.MODEL['YOLOV9']:
            #predict
            img_resize = cv2.resize(image, (640, 640), interpolation= cv2.INTER_CUBIC)
            dict_re = self.model.predict(img_resize, save=False, conf=0.65)   

            #image after predict
            plot = dict_re[0].plot() 

            #coordinates
            transformed_coordinates = lib.transform_coordinates(dict_re)
            print('transformed_coordinates ', transformed_coordinates)

            img_re = lib.draw_coordinates_on_image(image, transformed_coordinates)
            
            #save data
            label_out = lib.save_data_scan(dict_re, plot, transformed_coordinates)

            cv2.imshow("Image after define", cv2.resize(img_re, (600, 600), cv2.INTER_CUBIC))
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return transformed_coordinates 
    
def getCoordinates(model, image, flag=True):
    vision = VisionModule()
    vision.load_model(model)
    coordinate_list = vision._getCoordinateSingleobject_(image)
    return coordinate_list