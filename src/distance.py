import numpy as np


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