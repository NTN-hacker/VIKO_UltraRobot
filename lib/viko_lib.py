import os.path as osp
import numpy as np
import cv2
import matplotlib.pyplot as plt
import datetime
import pandas as pd

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
    ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY_INV)

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, 
                                        method=cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        # draw contours on the original image
        image_copy = img.copy()
        area = cv2.contourArea(contour)
        # print(area)
        if 0 < area < 10000:
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
            
  