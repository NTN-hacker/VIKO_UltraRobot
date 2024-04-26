import os.path as osp
import numpy as np
import cv2
import matplotlib.pyplot as plt
import datetime

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
def background_subtraction(fg_img, bg_img, diff_threshold=200):
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
def boudingBox(fg_cropImageRoi, fgMask):
    fg = fg_cropImageRoi.copy()
    contours, hierarchy = cv2.findContours(image=fgMask, mode=cv2.RETR_EXTERNAL, 
                                       method=cv2.CHAIN_APPROX_NONE)
    # draw contours on the original image
    cv2.drawContours(image=fg, contours=contours, contourIdx=-1, 
                    color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    
    print(contours)
    cv2.imshow('None approximation', fg)
    cv2.waitKey(0)
    cv2.imwrite('test_contour.jpg', fg)
    cv2.destroyAllWindows()
    return fg