import os.path as osp
import numpy as np
import cv2
import matplotlib.pyplot as plt
from datetime import datetime
import pandas as pd
import os
import sys
sys.path.append(
    "D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot"
)
from config import config as CFG
from library import viko_lib as lib
from scipy.linalg import lstsq
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter


MAX_AREA = 2048*2448
#Preprocessing
def preprocessing_image(img):
    #convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    gray = cv2.multiply(gray, 1.5)
    
    #blur remove noise
    blured1 = cv2.medianBlur(gray,3)
    blured2 = cv2.medianBlur(gray,47)
    divided = np.ma.divide(blured1, blured2).data
    normed = np.uint8(255*divided/divided.max())
    
    #Threshold image
    th, threshed = cv2.threshold(normed, 0, 255,cv2.THRESH_OTSU+  cv2.THRESH_BINARY)
    
    return threshed

#Blur
def blur_color_img(img, kernel_width=5, kernel_height=5, sigma_x=2, sigma_y=2):
    #increse brightness for foreground
    
    img = np.copy(img) # we don't modify the original image
    img[:,:,0] = cv2.GaussianBlur(img[:,:,0], ksize=(kernel_width, kernel_height), sigmaX=sigma_x, sigmaY=sigma_y)
    img[:,:,1] = cv2.GaussianBlur(img[:,:,1], ksize=(kernel_width, kernel_height), sigmaX=sigma_x, sigmaY=sigma_y)
    img[:,:,2] = cv2.GaussianBlur(img[:,:,2], ksize=(kernel_width, kernel_height), sigmaX=sigma_x, sigmaY=sigma_y)
    return img


#background subtraction
def background_subtraction(fg_img, bg_img, diff_threshold=150):
    fg_img = blur_color_img(fg_img)
    bg_img = blur_color_img(bg_img)
    mask = fg_img - bg_img
    mask = np.abs(mask)
    mask = np.mean(mask, axis=2, keepdims=False)
    mask[mask>diff_threshold] = 255
    mask[mask >= diff_threshold] = 0
    mask = mask.astype(np.uint8)
    mask = cv2.medianBlur(mask, 7)
    return mask

#excute
def main(foreground_img, background_img):
    fg_img = (foreground_img) # [h, w, 3]
    bg_img = (background_img) # [h, w, 3]
    mask = background_subtraction(fg_img, bg_img)
    new_fg = np.zeros([fg_img.shape[0], fg_img.shape[1], 4]) # png image --> has 4-dims instead of 3-dims like color image
    new_fg[:,:,:3] = fg_img
    new_fg[:,:,3] = mask
    return mask

#Post Processing
#####################################################################TEMP#######################################################
# def boudingBox(fg_cropImageRoi, fgMask):
#     fg = fg_cropImageRoi.copy()
#     contours, hierarchy = cv2.findContours(image=fgMask, mode=cv2.RETR_EXTERNAL, 
#                                        method=cv2.CHAIN_APPROX_NONE)
#     # draw contours on the original image
#     cv2.drawContours(image=fg, contours=contours, contourIdx=-1, 
#                     color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    
#     print(contours)
#     cv2.imshow('None approximation', fg)
#     cv2.waitKey(0)
#     cv2.imwrite('test_contour.jpg', fg)
#     cv2.destroyAllWindows()
#     return fg

######################################################################=======###################################################
def boudingBox(fg_cropImageRoi, fgMask):
    coordinate = list()
    fg = fg_cropImageRoi.copy()
    font = cv2.FONT_HERSHEY_COMPLEX
    contours, hierarchy = cv2.findContours(image=fgMask, mode=cv2.RETR_EXTERNAL, 
                                       method=cv2.CHAIN_APPROX_NONE)
    for cnt in contours : 
    
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True) 

        area = cv2.contourArea(cnt)

        if (0.3*MAX_AREA < area < MAX_AREA/2.5):
            print(area)
    
            cv2.drawContours(fg, [approx], 0, (0, 0, 255), 5)              
        
            n = approx.ravel()  
            i = 0

            for j in n : 
                if(i % 2 == 0): 
                    x = n[i] 
                    y = n[i + 1] 
        
                    string = str(x) + " " + str(y)  
        
                    if(i == 0): 
                        cv2.putText(fg, string, (x, y), 
                                        font, 2, (255, 0, 0))  
                    else: 
                        cv2.putText(fg, string, (x, y),  
                                font, 2, (0, 255, 0))  
                    
                    coordinate.append([x, y])
                i = i + 1
    return fg, coordinate

def get_point(img):
    x, y = 0., 0.
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # apply binary thresholding
    ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, 
                                        method=cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        # draw contours on the original image
        image_copy = img.copy()
        area = cv2.contourArea(contour)
        # print(area)
        if 6000 < area < 10000:
            cv2.drawContours(image=image_copy, contours=contour, contourIdx=-1, 
                            color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = float(M['m10'] / M['m00'])
                cy = float(M['m01'] / M['m00'])
                x, y, = cx, cy
                print(f"Coordinates of the center: ({cx}, {cy})")
            else:
                print("Could not find the center coordinates.")
        
    return x, y

def save_csv(filename, x, y):
    try:
        df = pd.read_csv('TestCircle.csv')
    except FileNotFoundError:
        df = pd.DataFrame()
        Filename = list()
        X = list()
        Y = list()

    

    Filename.append(filename)
    X.append(x)
    Y.append(y)
    
    data = {'Filename': Filename,
                'X': X,
                'Y': Y}
    
    df = pd.DataFrame(data)
    df.to_csv('TestCircle.csv')
    return True

def convert_mask(mask):
    """
    Use for Yolo
    """
    color_image = np.zeros((640, 640, 3), dtype=np.uint8)
    color_image[:, :, 0] = mask[0]  
    color_image[:, :, 1] = mask[0]  
    color_image[:, :, 2] = mask[0]  
    return color_image
  
def find_point_end(pts: np.array):
    points = pts

    start_point = points[0]

    distances = np.linalg.norm(points - start_point, axis=1)

    max_distance_index = np.argmax(distances)

    max_distance_point = points[max_distance_index]

    return max_distance_point, distances[max_distance_index]

def find_centerLine(dict_re, index_obj):

    img_re = dict_re[0].orig_img

    mask_test = dict_re[0].masks.data.detach().cpu().numpy()[index_obj]
    mask = cv2.normalize(mask_test, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)

    #edge detection
    high_threshold = 200
    low_threshold = 100
    edges = cv2.Canny(mask, low_threshold, high_threshold)

    # nếu len(unique(x) < len(unique(y)))
    # y is first
    if (len(np.unique(np.nonzero(edges)[0])) < len(np.unique(np.nonzero(edges)[1]))):
        y, x = np.nonzero(edges)
        flag = False
    else:
        flag = True
        x, y = np.nonzero(edges)

    coefficient_matrix = np.column_stack((x, np.ones_like(x))) # y = x.a + 1.b

    y_true = np.array(y) 

    coeffs, _, loss, _ = lstsq(coefficient_matrix, y_true)

    a, b = coeffs

    start_end = np.linspace(min(x), max(x), 2)

    y_start = start_end[0]*a + b
    y_end = start_end[1]*a + b

    # get coordinate start, end (by image, y horizontal - x vertical)
    if flag == True:
        coordinateXStart = y_start
        coordinateXEnd = y_end
        coordinateYStart = start_end[0]
        coordinateYEnd = start_end[1]
    else:
        coordinateXStart = start_end[0]
        coordinateXEnd = start_end[1]
        coordinateYStart = y_start
        coordinateYEnd = y_end
    
    # print(flag)

    print(F'coordinateXStart ', coordinateXStart)
    print(F'coordinateYStart ', coordinateYStart)
    
    # cv2.line(img_re, (int(coordinateXStart), int(coordinateYStart)), (int(coordinateXEnd), int(coordinateYEnd)), color = (255, 0, 0), thickness = 1)

    # cv2.imshow('test', mask)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


    # cv2.imshow('test', edges)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # return coordinates

    ## view and display
    coordinateStart = [int(coordinateXStart), int(coordinateYStart)]
    coordinateEnd   = [int(coordinateXEnd), int(coordinateYEnd)]

    ## real
    coordinateStartReal = [int(coordinateXStart*CFG.Y_RATIO), int(coordinateYStart*CFG.X_RATIO)]
    coordinateEndReal   = [int(coordinateXEnd*CFG.Y_RATIO), int(coordinateYEnd*CFG.X_RATIO)]
    
    ## merge
    coordinateView = [coordinateStart, coordinateEnd]
    coordinateReal = [coordinateStartReal, coordinateEndReal] if coordinateStartReal[1] < coordinateEndReal[1] \
                                                            else [coordinateEndReal, coordinateStartReal]

    return coordinateView, coordinateReal

def getLabels(results):
    """
    Save data predict when infer
    """
    labels = []

    for idx, result in enumerate(results):
        boxes = result.boxes  
        for box in boxes:
            label = int(box.cls.item()) 
            confidence = float(box.conf.item()) 
            labels.append(label)
    return labels

def draw_coordinates_on_image(image, coordinates):
    """
    Draws points and lines on the image for given coordinates.
    """
    colors = [
        (255, 0, 0),  
        (0, 255, 0),  
        (0, 0, 255),  
        (255, 255, 0),  
        (255, 0, 255),  
        (0, 255, 255),  
    ]
    
    for idx, (start, end) in enumerate(coordinates):
        color = colors[idx % len(colors)]

        cv2.circle(image, tuple(start), radius=5, color=color, thickness=3)
        cv2.circle(image, tuple(end), radius=5, color=color, thickness=3)

        cv2.line(image, tuple(start), tuple(end), color=color, thickness=2)
    
    return image

def transform_coordinates(dict_re) -> list:
    """
    Transforms the start and end coordinates based on the provided ratios.
    """

    listCoordinateView = list([])
    listCoordinateReal = list([])
    listModelWeld = list([])
    
    # list_label = lib.getLabels(dict_re)
    # indices = [idx for idx, label in enumerate(list_label) if label == 1]
    container_pairs = getWeldModelMulti(dict_re= dict_re)
    for obj in container_pairs:
        # print(obj[0][0])
        # print(obj[1][1])
        coordinateView, coordinateReal = lib.find_centerLine(dict_re= dict_re, index_obj = obj[0][0])
        listCoordinateView.append(coordinateView)
        listCoordinateReal.append(coordinateReal)
        listModelWeld.append(obj[1][1])
    
    # mac dinh chi 1 mau voi 1 bouding box
    # listModelWeld = np.unique(listModelWeld).tolist()
    
    #View image all weld
    current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    img_re = draw_coordinates_on_image(dict_re[0].orig_img, listCoordinateView)    
    # cv2.imwrite(f'laser/image_{current_time}_{CFG.CONTAINER_SIZE}l-{CFG.EXPOSURE_TIME}e-{CFG.IDLE_TIME}i.jpg', img_re)
    # cv2.imwrite('result_temp.png', img_re)
    # cv2.imshow('Image Re', img_re)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return listCoordinateReal, listModelWeld, img_re


    # if tag == 'multi':

    #     coordinate_re = dict_re[0].masks.xy
    #      #NEW MODEL: 1
    #     extract_points = lambda idx: np.array(coordinate_re[idx], dtype=np.int32)
    #     find_end_points = lambda pts: (lib.find_point_end(pts)[0].astype('uint'), pts[0].astype('uint'))
    #     coordinates_end, coordinates_start = zip(*map(lambda idx: find_end_points(extract_points(idx)), indices))
    #     transform_coordinate = lambda start, end: [
    #         [int(start[0] * CFG.Y_RATIO), int(start[1] * CFG.X_RATIO)],
    #         [int(end[0] * CFG.Y_RATIO), int(end[1] * CFG.X_RATIO)]
    #     ]
    #     return list(map(lambda pair: transform_coordinate(*pair), zip(coordinates_start, coordinates_end)))
    
    # elif tag == 'single':
    #     from scipy.linalg import lstsq

    #     img_re = dict_re[0][0].orig_img

    #     #get mask
    #     # index = 1
    #     indices = [idx for idx, label in enumerate(list_label) if label == 1]
    #     mask_test = dict_re[0].masks.data.detach().cpu().numpy()[indices[0]]
    #     mask = cv2.normalize(mask_test, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)

    #     #edge detection
    #     high_threshold = 200
    #     low_threshold = 100

    #     edges = cv2.Canny(mask, low_threshold, high_threshold)

    #     cv2.imshow("Image after edge detection", cv2.resize(edges, (600, 600), cv2.INTER_CUBIC))
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()

    #     #get coordinate
    #     x_coords, y_coords = np.nonzero(edges)

    #     A = np.column_stack((x_coords, np.ones_like(x_coords)))
    #     b = np.array(y_coords)

    #     coeffs, _, _, _ = lstsq(A, b)

    #     a, b = coeffs

    #     print("Đường trung tâm: y =  {:.2f}*x + {:.2f}".format(a, b))

    #     y_line = np.linspace(min(x_coords), max(x_coords), 100) 
    #     x_line = a * y_line + b 

    #     cv2.line(img_re, (int(x_line[0]), int(y_line[0])), (int(x_line[-1]), int(y_line[-1])), color = (255, 0, 0), thickness = 1)

    #     coordinate_re = [[[int(x_line[0]* CFG.Y_RATIO), int(y_line[0]* CFG.X_RATIO)], [int(x_line[-1]* CFG.Y_RATIO), int(y_line[-1]* CFG.X_RATIO)]]]

    #     cv2.imshow("Image after define", cv2.resize(img_re, (600, 600), cv2.INTER_CUBIC))
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()

    #     return coordinate_re

    # else:
    #     print('No signal')
    
def getWeldModel(dict_re) -> str:
    """
    Label for single object
    """
    label_conf_model_weld = []
    dict_model_weld = CFG.MODEL_WELD
    for idx, result in enumerate(dict_re):
        boxes = result.boxes  
        for box in boxes:
            label = int(box.cls.item()) 
            confidence = float(box.conf.item())
            if (label != 1) and (label != 3) and confidence > CFG.CONF_MODEL_WELD: #1: weld, 3: other
    
                label_conf_model_weld.append([dict_model_weld[label], confidence])

    shape = label_conf_model_weld[0][0]
    return shape

def getWeldModelMulti(dict_re) -> list:
    label_box_model_weld = []
    dict_model_weld = CFG.MODEL_WELD
    for idx, result in enumerate(dict_re):
        boxes = result.boxes  
        for index, box in enumerate(boxes):
            label = int(box.cls.item()) 
            confidence = float(box.conf.item())
            box = box.xyxy.detach().cpu().numpy().tolist()[0]
            if (label != 3) and confidence > CFG.CONF_MODEL_WELD: #3: other
                label_box_model_weld.append([index, dict_model_weld[label], confidence, box])
    print(f'Label: ', label_box_model_weld)
    containing_pairs = findContainingPairs(label_box_model_weld)
    print(f'Pair: ', containing_pairs)

    return containing_pairs

def save_data_scan(results, image_re, coordinates):
    """
    Save scan evaluation when infer.
    """
    output_dir = 'runs/segment/result/image'
    os.makedirs(output_dir, exist_ok=True)
    labels = []
    confs = []
    new_rows = []

    csv_file_path = 'runs/segment/result/scan.csv'
    if os.path.exists(csv_file_path):
        df = pd.read_csv(csv_file_path)
    else:
        df = pd.DataFrame(columns=[
            'Timestamp', 'ImageName', 'OutputPath', 'Label', 'Confidence', 
            'CoordinateStart', 'CoordinateEnd', 'ScanLength'
        ])

    timestamp = datetime.now().strftime("%Y-%m-%d")
    
    for idx, result in enumerate(results):
        boxes = result.boxes  
        for box in boxes:
            label = int(box.cls.item()) 
            confidence = float(box.conf.item()) 
            labels.append(label)
            confs.append(confidence)
            
            image_name = f"sample_{len(df) + len(new_rows) + 1}.png"
            output_path = os.path.join(output_dir, timestamp, image_name)
            
            # Create directory for current date if it doesn't exist
            os.makedirs(os.path.join(output_dir, timestamp), exist_ok=True)
            
            cv2.imwrite(output_path, image_re)

        # Calculate the number of scans and scan lengths
        for idx_m, (start, end) in enumerate(coordinates):
            scan_length = np.linalg.norm(np.array(start) - np.array(end))

            new_rows.append({
                'Timestamp': timestamp,
                'ImageName': image_name,
                'OutputPath': output_path,
                'Label': labels[idx_m],
                'Confidence': confs[idx_m],
                'CoordinateStart': start,
                'CoordinateEnd': end,
                'ScanLength': scan_length
            })

    new_df = pd.DataFrame(new_rows)
    df = pd.concat([df, new_df], ignore_index=True)
    df.to_csv(csv_file_path, index=False)

# def isContained(outer, inner):
#     x, y, x_w, y_h = outer
#     x_, y_, x_w_, y_h_ = inner
#     return (x < x_ and y < y_ and (x_w > x_w_ or y_h > y_h_))

def check_intersection(polygon1, polygon2):
    """
    Update to replace 
    """
    # print(polygon1)
    intersection = list()
    for point in polygon2:
        result = cv2.pointPolygonTest(np.array(polygon1, dtype=np.float32), (float(point[0]), float(point[1])), measureDist=False)
        # if point inside return 1
        # if point outside return -1
        # if point on the contour return 0
        intersection.append(result)
  
    internal = 0
    external = 0

    for value in intersection:
        if value == -1:
            external += 1
        elif value == 1:
            internal += 1
        elif value == 0:
            internal += 1

    # print(internal)

    if internal >= 3:
        return True
    return False

def findContainingPairs(data):
    containing_pairs = list([])

    for label_item in data:
        idx, label, confidence, box = label_item
        if label in ['0_degree', '30_degree', '90_degree']:
            x1, y1, x2, y2 = box
            label_rect = (x1, y1, x2, y2)

            for weld_item in data:
                weld_idx, weld_label, weld_confidence, weld_box = weld_item
                if weld_label == 'weld':
                    weld_x1, weld_y1, weld_x2, weld_y2 = weld_box
                    weld_rect = (weld_x1, weld_y1, weld_x2, weld_y2)

                    object = list(label_rect)
                    weld = list(weld_rect)
                    object = np.array([[object[0] - CFG.PIXEL_UNION, object[1]], [object[2] + CFG.PIXEL_UNION, object[1]], 
                                       [object[2] + CFG.PIXEL_UNION, object[3] + CFG.PIXEL_UNION], 
                                       [object[0] - CFG.PIXEL_UNION, object[3] + CFG.PIXEL_UNION]]).astype(int)  #config because some case the coordinate weld higher object
                    weld = np.array([[weld[0], weld[1]], [weld[2], weld[1]], [weld[2], weld[3]], [weld[0], weld[3]]]).astype(int)

                    if check_intersection(object, weld):
                        containing_pairs.append([weld_item, label_item])

    return containing_pairs

def count_defects(defects):
    return dict(Counter(defects))

def save_lidar_data(lidar_data, filename):
    with open(filename, 'w') as file:
        for point in lidar_data:
            x, z = point
            file.write(f"{x}, {z}\n")

def convert_to_grayscale_image(z):
        Z_processed = list()
        for value in z:
            Z_processed.append(value[1])
        Z_processed = np.array(Z_processed).reshape(-1, CFG.RESOLUTION_X_LASER)

        non_zero_values = Z_processed[Z_processed != 0]
        mean_value = np.mean(non_zero_values)

        # Thay thế các giá trị 0 bằng giá trị trung bình
        Z_processed[Z_processed == 0] = mean_value
        min_val = np.min(Z_processed)
        max_val = np.max(Z_processed)

        normalized_Z = (Z_processed - min_val) / (max_val - min_val)
        gray_image = np.stack((normalized_Z), axis=-1)
        # gray_image = cv2.resize(gray_image, (640, 640), cv2.INTER_CUBIC)
        image_rgb = (gray_image * 255).astype(np.uint8)
        image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)

        laser_img = apply_shading(image_rgb)

        # Path to the directory
        data_dir = 'C:/Robdat'

        # Delete all files in the target directory
        for filename in os.listdir(data_dir):
            file_path = os.path.join(data_dir, filename)
            if os.path.isfile(file_path):
                os.remove(file_path)

        current_time = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        cv2.imwrite(f'laser/experimental/laser_{current_time}.png', laser_img)   
        import struct

        length_weld = len(Z_processed)
        with open(f'C:/Robdat/data__{length_weld}__{current_time}.dat', 'wb') as bin:
            for raw in Z_processed:
                bin_data = struct.pack(f'{len(raw)}f', *raw)
                bin.write(bin_data)

        return Z_processed, laser_img

def apply_shading(image_rgb):
        grad_x, grad_y, _ = np.gradient(image_rgb)
        slope = np.pi / 2. - np.arctan(np.sqrt(grad_x ** 2 + grad_y ** 2))
        aspect = np.arctan2(-grad_y, grad_x)

        #     azimuth = CFG.AZIMUTH
        #     altitude = CFG.ALTITUDE
        azimuth = 315  # angle between the light source and north, in degrees
        altitude = 45

        azimuth_rad = np.radians(azimuth)
        altitude_rad = np.radians(altitude)

        shaded = (np.sin(altitude_rad) * np.sin(slope) +
        np.cos(altitude_rad) * np.cos(slope) * np.cos(azimuth_rad - np.pi / 2. - aspect))

        shaded = (shaded - shaded.min()) / (shaded.max() - shaded.min())
        img_shaded = (shaded * 255).astype(np.uint8)
        # image_rgb_shaded = cv2.cvtColor(img_shaded, cv2.COLOR_BGR2RGB)

        # print('shape ', image_rgb_shaded.shape)

        return img_shaded

def run_inspection(model, img, conf):
    results = model.predict(source=img, conf=conf) #CFG.CONF_ACC    
    plot = results[0].plot()
    return plot



