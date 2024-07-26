import cv2
from pypylon import pylon
from datetime import datetime

# Connecting to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabing Continuously (video) with minimal delay
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
converter = pylon.ImageFormatConverter()

# Converting to OpenCV BGR format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Used to record the time when we processed last frame
prev_frame_time = 0
# Used to record the time at which we processed current frame
new_frame_time = 0

index = 0
while camera.IsGrabbing():
    index+= 1
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data
        image1 = converter.Convert(grabResult)
        image = image1.GetArray()
        image = cv2.resize(image, (2448, 2048), fx=0.8366, fy=1, interpolation=cv2.INTER_LINEAR)
        
        # Show image
        image_resized = cv2.resize(image , (640, 640))
        cv2.imshow("Image", image_resized)
        
        # Get current time
        new_frame_time = datetime.now()

        # Save image when 'a' is pressed
        key = cv2.waitKey(1)
        if key == ord('a'):
            cv2.imwrite(f'data/AI/Record_{datetime.now().strftime("%Y-%m-%d")}/image_{index}_{index}.png', image)

    grabResult.Release()

    # Break loop if any key is pressed
    if cv2.waitKey(1) != -1:
        break

# Releasing the resource    
camera.StopGrabbing()
cv2.destroyAllWindows()
