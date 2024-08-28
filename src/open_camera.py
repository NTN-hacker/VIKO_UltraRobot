import cv2

from pypylon import pylon
import cv2,time
from datetime import datetime
import os


def onCameraGrabbed():
    # conecting to the first available camera
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

    # Grabing Continusely (video) with minimal delay
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
    converter = pylon.ImageFormatConverter()

    # converting to opencv bgr format
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    if camera.IsGrabbing():
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        if grabResult.GrabSucceeded():
            # Access the image data
            image1 = converter.Convert(grabResult)
            image = image1.GetArray()
            image = cv2.resize(image, (2448, 2048), fx=0.8366, fy=1, interpolation=cv2.INTER_LINEAR)

            current_time = datetime.now()
            filename = f"data\Robotic\Distance_Z_Axis\Image{current_time.strftime('%Y-%m-%d_%H-%M-%S')}.png"
            cv2.imwrite(filename, image)
        

        grabResult.Release()
        image = cv2.resize(image , (640,640))
        # cv2.imshow("Image", image)
        # if cv2.waitKey(20) != -1:
        #     break  # Wait indefinitely until a key is pressed

    # Releasing the resource    
    camera.StopGrabbing()
    return filename


    # cv2.destroyAllWindows()