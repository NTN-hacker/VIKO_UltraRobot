"""
config file
"""
# import torch

# TODO


########################################################### ROBOTIC ###################################################################
PIXEL_SIZE = 3.45
SPEEDS = [100, 50]
FOCAL_LENGTH = 16
RESOLUTION_X = 2448 #DEFAULT RESOLUTION
RESOLUTION_Y = 2048 #DEFAULT RESOLUTION
DISTANCE_O1O2 = 60
########################################################## AI MODEL ###################################################################



DEVICE = "cuda" #if torch.cuda.is_available() else "cpu"
MODEL = {
    'SAM' : 
    {
        'WEIGHT' : 'D:\\nhan\\viko\src\sam_vit_h_4b8939.pth', #Path of your checkpoint
        'MODEL_TYPE' : 'vit_h'
    },

    'LightWeight_SAM' : 
    {
        'WEIGHT' : 'D:\\nhan\\viko\src\weight\mobile_sam.pt', #Path of your checkpoint
        'MODEL_TYPE' : 'vit_t'
    },

    'SEGMENT_WELD': 
    {
        'API_KEY': 'JtRFLNmuxFdQiNLXfFJj', # your api key
        'PROJECT_NAME': 'weld-detection-slz4d',
        'CONFIDENT': 50,
        'OVERLAP': 50
    }
}
