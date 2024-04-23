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
    # blured1 = cv2.medianBlur(img,3)
    # blured2 = cv2.medianBlur(img,51)
    # img = cv2.GaussianBlur(img, (5, 5), 0)
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
    # cv2.imwrite('mask.jpg', mask)
    # cv2.imwrite('test1.png', new_fg)
    # display_image(mask)
    return mask

#Post Processing
def boudingBox(fg_cropImageRoi, fgMask):
    fg = fg_cropImageRoi.copy()
    cordinate = 0
    area_get = 0
    #You can choose type connectivity with value about from 4 to 8
    connectivity = 4
    output = cv2.connectedComponentsWithStats(fgMask, connectivity, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
    for i in  range(0, numLabels):
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]

        if (0.3*MAX_AREA < area < MAX_AREA)  and (w < fg.shape[0]) and (h < fg.shape[1]):
            cv2.rectangle(fg, (x, y), (x + w, y + h), (0, 255, 0), 10)
            cordinate = [x, y, w, h]
            text = f'({x}, {y})'
            cv2.putText(fg, text, color = (0, 255, 125), org = (x, y-50), fontFace= 1, fontScale= 3, thickness= 5, lineType= 1)

            text2 = f'({x+w}, {y+h})'
            cv2.putText(fg, text2, color = (0, 255, 125), org = (x+w, y+h+50), fontFace= 1, fontScale= 3, thickness= 5, lineType= 1)
            area_get = area
    
    # cv2.imshow('Roi Image 2' , fg)
    # time = datetime.datetime.now()
    # cv2.imwrite("test_re.png", fg)
    return cordinate, fg, area_get

if __name__ == '__main__':

    start_time = datetime.datetime.now()
    # Background
    img_bg = cv2.imread('data\\data_2024_04_20\\9.png', cv2.IMREAD_COLOR)
    # Object
    obj_path = 'data\\data_2024_04_20\\1_vertical.png'
    img_obj = cv2.imread(obj_path , cv2.IMREAD_COLOR)
    # Mask
    fmask = main(img_obj, img_bg)
    cv2.imwrite(f'data/mask/{osp.basename(obj_path)}', fmask)
    # Return Object
    coordinate, fg, area = boudingBox(img_obj, fgMask= fmask)
    cv2.imwrite(f'data/fg/{osp.basename(obj_path)}', fg)
    print(f'The information of object {coordinate}, {area}')
    # Crop Working Space
    x, y, w, h = coordinate
    img_crop = img_obj[y: y+h, x: x+w]
    print(f'Time to process step 1 {datetime.datetime.now() - start_time}')
    # Test save
    cv2.imwrite(f'data/boudingbox/{osp.basename(obj_path)}', img_crop)



