"""
config file
"""
########################################################### ROBOTIC ###################################################################
PIXEL_SIZE = 3.45
SPEEDS = [100, 50]
FOCAL_LENGTH = 16
RESOLUTION_X = 2048 #DEFAULT RESOLUTION
RESOLUTION_Y = 2448 #DEFAULT RESOLUTION
DISTANCE_2OBJECT = 480

########################################################## AI MODEL ###################################################################

WEIGHT = 'D:\\nhan\\viko\src\sam_vit_h_4b8939.pth' #Path of your checkpoint
MODEL_TYPE = 'vit_h'
DEVICE = 'cuda:0'
