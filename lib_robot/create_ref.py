import numpy as np

def createRef(translation, rotation):
    """
    Create a transformation matrix for a new reference frame.

    Args:
    - translation: A 3-element list or array representing the translation [Tx, Ty, Tz].
    - rotation: A 3-element list or array representing the rotation angles [Rx, Ry, Rz] in radians.

    Returns:
    - transformation_matrix: A 4x4 transformation matrix representing the new reference frame.
    """
    # Translation matrix
    translation_matrix = np.array(
        [
            [1, 0, 0, translation[0]],
            [0, 1, 0, translation[1]],
            [0, 0, 1, translation[2]],
            [0, 0, 0, 1],
        ]
    )

    # Rotation matrices (assuming XYZ Euler angles)
    rotation_x = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(rotation[0]), -np.sin(rotation[0]), 0],
            [0, np.sin(rotation[0]), np.cos(rotation[0]), 0],
            [0, 0, 0, 1],
        ]
    )

    rotation_y = np.array(
        [
            [np.cos(rotation[1]), 0, np.sin(rotation[1]), 0],
            [0, 1, 0, 0],
            [-np.sin(rotation[1]), 0, np.cos(rotation[1]), 0],
            [0, 0, 0, 1],
        ]
    )

    rotation_z = np.array(
        [
            [np.cos(rotation[2]), -np.sin(rotation[2]), 0, 0],
            [np.sin(rotation[2]), np.cos(rotation[2]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    # Combine translation and rotation
    transformation_matrix = np.dot(
        translation_matrix, np.dot(rotation_x, np.dot(rotation_y, rotation_z))
    )

    return transformation_matrix