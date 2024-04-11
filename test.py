import numpy as np


def convertCoordinates(focal_length, res_height, res_width, pixel_x, pixel_y, distance):

    sensor_height = res_height * 3.45 / 1000
    sensor_width = res_width * 3.45 / 1000
    sensor_size = np.array([sensor_height, sensor_width])

    hFOV = (distance * sensor_size[0]) / focal_length
    vFOV = (distance * sensor_size[1]) / focal_length
    print("hFOV:", hFOV, "\n", "vFOV:", vFOV, "\n")

    spatial_x = pixel_x * hFOV / res_width
    spatial_y = pixel_y * vFOV / res_height
    print("spatial_x:", spatial_x, "\n", "spatial_y:", spatial_y, "\n")

    return spatial_x, spatial_y


convertCoordinates(16, 2448, 2048, 720, 1280, 300)
