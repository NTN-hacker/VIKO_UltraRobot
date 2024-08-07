"""
config file
"""

# import torch

# TODO


########################################################### ROBOTIC ###################################################################
PIXEL_SIZE = 3.45  ## unit: micrometer
LINEAR_SPEEDS = [30, 30]
JOINT_SPEEDS = [20, 40] ### Not over 100
FOCAL_LENGTH = 16
RESOLUTION_X = 2448  # DEFAULT RESOLUTION
RESOLUTION_Y = 2048  # DEFAULT RESOLUTION
HORIZONTAL_BASELINE = 55  # 65: best value with object is black dot
VERTICAL_BASELINE = 450  # best value with object is black dot
SAFE_DISTANCE = 565
ROTATE_OY_LASER = 40
ROTATE_OX_LASER = 0  ### Negative degrees
ERROR_POS = 3

HOME_LASER = 611
DISTANCE_LASER2OBJECT = 135
DISTANCE_CAMERA2OBJECT = 651

TEST_TARGET = [[1878, 598], [470, 1162]]
########################################################## AI MODEL ###################################################################


DEVICE = "cuda"  # if torch.cuda.is_available() else "cpu"
MODEL = {
    "SAM": {
        "WEIGHT": "D:\\nhan\\viko\src\sam_vit_h_4b8939.pth",  # Path of your checkpoint
        "MODEL_TYPE": "vit_h",
    },
    "LightWeight_SAM": {
        "WEIGHT": "D:\\nhan\\viko\src\weight\mobile_sam.pt",  # Path of your checkpoint
        "MODEL_TYPE": "vit_t",
    },
    "SEGMENT_WELD": {
        "API_KEY": "JtRFLNmuxFdQiNLXfFJj",  # your api key
        "PROJECT_NAME": "weld-detection-slz4d",
        "CONFIDENT": 50,
        "OVERLAP": 50,
    },
    "HQ_SAM": {
        "WEIGHT": "D:\\nhan\\viko\src\weight\sam_hq_vit_tiny.pth",
        "MODEL_TYPE": "vit_tiny",
    },
    "YOLOV9": {
        "WEIGHT": "weight\\best.onnx"
        # "WEIGHT": 'D:\\nhan\\viko\src\weight\\Weld_Identification_1.pt'
    },
    "INSPECTION": {
        "WEIGHT": "weight\\inspection.onnx"
    }
}

########################################################## COORDINATE #######################################################################
X_RATIO = 2048 / 640
Y_RATIO = 2448 / 640


######################################################### LABEL #############################################################################
MODEL_WELD = {0: "0_degree", 1: "weld", 2: "90_degree", 3: "other", 4: "30_degree"}
MODEL_INSPECTION = {0: 'air-hole', 1: 'bite-edge', 2: 'broken-arc', 3: 'crack', 4: 'hollow-bead', 5: 'overlap', 6: 'slag-inclusion', 7: 'unfused'}


CONF_MODEL_WELD = 0.6
PIXEL_UNION = 5 #Chấp nhận lệch 5 pixel khi xác định hai obj trùng. Sử dụng trong trường hợp mối hàn nằm trên khung obj

######################################################### LASER #############################################################################
PATH_LASER_PROGRAM = 'E:\\Quan\\AutoRoboticInspection-V1\\VIKO_UltraRobot\\src\laser\\TranferData\\bin\Release\\net8.0\\win-x64\\publish\\TranferData.exe'
# Define the azimuth (direction) and altitude (angle) of the light source
AZIMUTH = 315  # angle between the light source and north, in degrees
ALTITUDE = 45  # angle above the horizon
RESOLUTION_Y_LASER = 0.1 #mm
FREQUENCY = 650 #Hz

# EXPOSURE_TIME = 1000 #us
# IDLE_TIME = 3900 #us
# SLEEP = 0.1
# CONTAINER_SIZE = 2000 #lines
# TIME_SCAN = 200/0.1/650 




###### optimize speed #######

# # Define the azimuth (direction) and altitude (angle) of the light source
# AZIMUTH = 315  # angle between the light source and north, in degrees
# ALTITUDE = 45  # angle above the horizon

# EXPOSURE_TIME = 1000 #us
# IDLE_TIME = 3900 #us
# SLEEP = 0.1
# NO_PROFILES = 650
# RESOLUTION_LASER_Y = 100 #(um)
# TIME_SCAN = 200000/100/650
# CONTAINER_SIZE = 2000 # to save

