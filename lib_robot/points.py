import numpy as np
from robotic import x_target, x_target_02, y_target, y_target_02, createPoint
from config import config as CFG
def trajectory(x, y):
    ## create the y = ax + b
    a, b = np.polyfit(x, y, 1)
    return a, b

x = [x_target, x_target_02]
y = [y_target, y_target_02]
a, b = trajectory(x, y)
print(f'a:{a}, b:{b}')

def points_tra(a, b, x_target, x_target_02) -> float:
    i = 0
    step = int(abs(x_target_02 - x_target)/10)
    new_x = np.zeros(10)
    new_y = np.zeros(10)
    for i in range(10):
        if i == 0:
            new_x[i] = x_target + step
            new_y[i] = a * new_x[i] + b
        else:
            new_x[i] = new_x[i-1] + step
            new_y[i] = a * new_x[i] + b
        
        target, joint = createPoint(new_x[i], new_y[i], CFG.DISTANCE_2OBJECT, i)
        
        if i == 8:
            print(f'joint:{joint}')

    print(f'newx:{new_x}, newy:{new_y}')
    

    return new_x, new_y

new_x, new_y = points_tra(a, b, x_target, x_target_02)