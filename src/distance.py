import numpy as np
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
from robotic import (rf_camera2base_matrix, rf_camera2flange_non_matrix, setFrame, rf_camera2flange_matrix)


RDK = Robolink()
# robot = RDK.AddFile("E:\\Install-software\\RoboDK\\Library\\Motoman-GP8.robot")
robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("No robot selected or available")


RUN_ON_ROBOT = True

if RDK.RunMode() != RUNMODE_SIMULATE:
    RUN_ON_ROBOT = False

if RUN_ON_ROBOT:
    
    # Connect to the robot using default IP
    success = robot.Connect()  # Try to connect once````
    #success robot.ConnectSafe() # Try to connect multiple times
    status, status_msg = robot.ConnectedState()
    print(status)
    print(status_msg)
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        print(status_msg)
        raise Exception("Failed to connect: " + status_msg)

    # This will set to run the API programs on the robot and the simulator (online programming)
    RDK.setRunMode(RUNMODE_RUN_ROBOT)

robot.ConnectedState()
# print("ConnectedState:", robot.ConnectedState(), "\n")
# robot.setPoseFrame(setFrame)
robot.setPoseFrame(robot.PoseFrame())

print(f"robot.PoseFrame():{robot.PoseFrame()}")
# robot.setPoseTool(rf_camera2flange_matrix)
robot.setPoseTool(robot.PoseTool())

# print(f"robot.PoseTool():{robot.PoseTool()}")
robot.setRounding(5)  # Set the rounding parameter
robot.setSpeed(10, 10)  # Set linear speed in mm/s
robot.setSpeedJoints(10)

def cameraPos(data: Mat, mat_camera2base):
    
    # robot.MoveJ(rf_camera2base_matrix)
    print(f'rf_camera2base_non_matrix:{data}')
    print(f'mat_camera2base:{mat_camera2base}')

# cameraPos(rf_camera2flange_non_matrix, rf_camera2base_matrix)

def convertCoordinates(res_height, res_width, pixel_x, pixel_y):                    

    sensor_height = res_height * 3.5 / 1000 #cfg
    sensor_width = res_width * 3.5 / 1000 #cfg
    sensor_size = np.array([sensor_height, sensor_width])
    print("sensor_size:", sensor_size, "\n")

    xpixel_center = res_width / 2
    ypixel_center = res_height / 2
    xpixel_2center = pixel_x - xpixel_center
    ypixel_2center = pixel_y - ypixel_center
    print("xpixel2center:", xpixel_2center, "\n")

    x_2_camera = xpixel_2center * sensor_size[1] / (2 * xpixel_center)
    y_2_camera = ypixel_2center * sensor_size[1] / (2 * ypixel_center)

    print("x2camera:", x_2_camera, "\n")
    print("y2camera:", y_2_camera, "\n")

    return x_2_camera, y_2_camera

def height(x1, y1, x2, y2, o1o2, focal_length):
    
    distance = o1o2 * focal_length / (abs(x1) + abs(x2))
    print("distance:", distance, "\n")

    x1_real = x1 * distance / focal_length
    y1_real = y1 * distance / focal_length
    target1 = np.array([x1_real, y1_real])

    x2_real = x2 * distance / focal_length
    y2_real = y2* distance / focal_length
    target2 = np.array([x2_real, y2_real])

    return target1, target2


pixel_x1 = 1128
pixel_y1 = 140

pixel_x2 = 1698
pixel_y2 = 142

o1o2 = 33.12 * 2
res_width = 2448
res_height = 2048
focal_length = 16
x1, y1 = convertCoordinates(res_height, res_width, pixel_x1, pixel_y1)
x2, y2 = convertCoordinates(res_height, res_width, pixel_x2, pixel_y2)

target1, target2 = height(x1, y1, x2, y2, o1o2, focal_length)
print(f'target1:{target1}, target2:{target2}')