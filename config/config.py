"""
config file
"""

# import torch

# TODO


########################################################### ROBOTIC ###################################################################
PIXEL_SIZE = 3.45  ## unit: micrometer
LINEAR_SPEEDS = [30, 100]
JOINT_SPEEDS = [20, 40] ### Not over 100
FOCAL_LENGTH = 16
RESOLUTION_X = 2448  # DEFAULT RESOLUTION
RESOLUTION_Y = 2048  # DEFAULT RESOLUTION
HORIZONTAL_BASELINE = 55  # 65: best value with object is black dot
VERTICAL_BASELINE = 450  # best value with object is black dot
SAFE_DISTANCE = 500
ROTATE_OY_LASER = 40
ROTATE_OX_LASER = 0    ### Negative degrees
OX_POS_LASER_POSITIVE = 110
OX_POS_LASER_NEGATIVE = 80
OY_POS_LASER = 130   ### NOT USED
ERROR_POS = 3

DISTANCE_LASER2OBJECT = 135
DISTANCE_CAMERA2OBJECT = 569

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
        "WEIGHT": "E:\\Quan\\AutoRoboticInspection-V1\\VIKO_UltraRobot\\weight\\best.onnx"
        # "WEIGHT": 'D:\\nhan\\viko\src\weight\\Weld_Identification_1.pt'
    },
    "INSPECTION": {
        "WEIGHT": "E:\\Quan\\AutoRoboticInspection-V1\\VIKO_UltraRobot\\weight\\inspection.onnx"
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
PATH_LASER_PROGRAM = 'C:\\Users\\Admin\\Downloads\\scanCONTROL-Windows-SDK-4-1-1\\scanCONTROL Windows SDK 4.1.1\\C# SDK\\examples\\bin_x64\\Release\\ContainerMode.exe'

EXPOSURE_TIME = 5000 #us
IDLE_TIME = 3900 #us
CONTAINER_SIZE = 2000 #lines
# TIME_SCAN = (EXPOSURE_TIME + IDLE_TIME) * CONTAINER_SIZE / 185000 #s
Denom = 100000

TIME_SCAN = (EXPOSURE_TIME + IDLE_TIME) * CONTAINER_SIZE / 100000 #s
SLEEP = TIME_SCAN + 1 #s

# Define the azimuth (direction) and altitude (angle) of the light source
AZIMUTH = 315  # angle between the light source and north, in degrees
ALTITUDE = 45  # angle above the horizon