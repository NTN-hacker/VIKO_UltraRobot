import numpy as np

def rotPos(trans_matrix):
    roll = np.arctan2(trans_matrix[2, 1], trans_matrix[2, 2])
    pitch = np.arctan2(
        -trans_matrix[2, 0], np.sqrt(trans_matrix[2, 1] ** 2 + trans_matrix[2, 2] ** 2)
    )
    yaw = np.arctan2(trans_matrix[1, 0], trans_matrix[0, 0])

    rot = np.array([roll, pitch, yaw])
    pos = trans_matrix[:3, 3]

    return rot, pos