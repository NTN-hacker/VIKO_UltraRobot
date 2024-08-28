import cv2

from pypylon import pylon
import cv2,time

# conecting to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabing Continusely (video) with minimal delay
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
converter = pylon.ImageFormatConverter()

# converting to opencv bgr format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# used to record the time when we processed last frame
prev_frame_time = 0
# used to record the time at which we processed current frame
new_frame_time = 0

index = 0
while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data
        image1 = converter.Convert(grabResult)
        image = image1.GetArray()
        image = cv2.resize(image, (2448, 2048), fx=0.8366, fy=1, interpolation=cv2.INTER_LINEAR)
        cv2.imwrite('data/AI/Record_2024-05-31/test.png', image)
    grabResult.Release()
    image = cv2.resize(image , (640,640))
    cv2.imshow("Image", image)
    if cv2.waitKey(20) != -1:
        break  # Wait indefinitely until a key is pressed

    
    
# Releasing the resource    
camera.StopGrabbing()

cv2.destroyAllWindows()