"""
config file
"""

# import torch

# TODO


########################################################### ROBOTIC ###################################################################
PIXEL_SIZE = 3.45  ## unit: micrometer
LINEAR_SPEEDS = [50, 100]
JOINT_SPEEDS = [20, 80]
FOCAL_LENGTH = 16
RESOLUTION_X = 2448  # DEFAULT RESOLUTION
RESOLUTION_Y = 2048  # DEFAULT RESOLUTION
HORIZONTAL_BASELINE = 55  # 65: best value with object is black dot
VERTICAL_BASELINE = 450  # best value with object is black dot
SAFE_DISTANCE = 600

DISTANCE_LASERtoOBJECT = 135
DISTANCE_CAMERA2OBJECT = 569


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
        "WEIGHT": "D:\\nhan\\viko\\VIKO_UltraRobot\\weight\\best.pt"
        # "WEIGHT": 'D:\\nhan\\viko\src\weight\\Weld_Identification_1.pt'
    },
}

########################################################## COORDINATE #######################################################################
X_RATIO = 2048 / 640
Y_RATIO = 2448 / 640


######################################################### LABEL #############################################################################
MODEL_WELD = {0: "0_degree", 1: "weld", 2: "90_degree", 3: "other", 4: "30_degree"}

######################################################### LABEL #############################################################################
CONF_MODEL_WELD = 0.6
