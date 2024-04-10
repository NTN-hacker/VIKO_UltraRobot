from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import numpy as np
import time


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


def rotPos(trans_matrix):
    roll = np.arctan2(trans_matrix[2, 1], trans_matrix[2, 2])
    pitch = np.arctan2(
        -trans_matrix[2, 0], np.sqrt(trans_matrix[2, 1] ** 2 + trans_matrix[2, 2] ** 2)
    )
    yaw = np.arctan2(trans_matrix[1, 0], trans_matrix[0, 0])

    rot = np.array([roll, pitch, yaw])
    pos = trans_matrix[:3, 3]

    return rot, pos


def drawPolygon(target_rf, n_sides, R):

    for i in range(n_sides + 1):
        ang = i * 2 * pi / n_sides  # angle: 0, 60, 120, ...
        # target_matrix = TxyzRxyz_2_Pose(pos_i)
        # -----------------------------
        # Movement relative to the reference frame
        # Create a copy of the target
        target_draw_1 = Mat(target_rf)
        print(f"target_i: {target_draw_1}")
        pos_draw_i = target_draw_1.Pos()
        pos_draw_i[0] = pos_draw_i[0] + R * cos(ang)
        pos_draw_i[1] = pos_draw_i[1] + R * sin(ang)
        target_draw_1.setPos(pos_draw_i)
        print("Moving to target %i: angle %.1f" % (i, ang * 180 / pi))
        # print(str(Pose_2_TxyzRxyz(target_i)))
        robot.setSpeed(0)
        robot.MoveL(target_draw_1)
    return


def intialTarget(x, y, z):
    # target_pos_initial = np.array([200, 200, 200, 1])
    target_pos_initial = np.array([x, y, z, 1])

    target_pose = np.array(
        [
            [1, 0, 0, target_pos_initial[0]],
            [0, 1, 0, target_pos_initial[1]],
            [0, 0, 1, target_pos_initial[2]],
            [0, 0, 0, 1],
        ]
    )
    return target_pose


def changeJoint(rf_camera2base, rf_surface2base):

    rf_base2camera = np.linalg.inv(rf_camera2base)
    print("ref_base2camera:", rf_base2camera, "\n")
    print("ref_surface2base:", rf_surface2base, "\n")
    rf_surface2camera = np.dot(rf_base2camera, rf_surface2base)
    print("ref_surface2camera:", rf_surface2camera, "\n")

    rot, pos = rotPos(rf_surface2camera)
    print("rot:", rot, "\n", "pos:", pos, "\n")

    return


def createPoint(x_st, y_st, z_st):

    x_axis = x_st
    y_axis = y_st
    z_axis = z_st

    target_points = [x_axis, y_axis, z_axis]
    count_i = i = j = 0
    for i in range(10):
        count_i = i
        if i == 0:
            x_axis = x_axis
            for j in range(20):
                y_axis = y_axis - 10
                z_axis = z_axis
                target_points = [[x_axis, y_axis, z_axis]]
                j = j + 1

        elif i > 0 and i % 2 != 0:
            x_axis = x_axis + 10
            target_points.append([x_axis, y_axis, z_axis])
            for j in range(20):
                y_axis = y_axis + 10
                z_axis = z_axis
                target_points.append([x_axis, y_axis, z_axis])
                j = j + 1

        elif i > 0 and i % 2 == 0:
            x_axis = x_axis + 10
            target_points.append([x_axis, y_axis, z_axis])
            for j in range(20):
                y_axis = y_axis - 10
                z_axis = z_axis
                target_points.append([x_axis, y_axis, z_axis])
                j = j + 1

        i = i + 1

    target_pose = []
    print(
        "len(target_points):", len(target_points), "\n", "count_i:", str(count_i), "\n"
    )
    for i in range(len(target_points)):

        # print("target_points:", target_points[i], "\n")
        target_posei = intialTarget(
            target_points[i][0], target_points[i][1], target_points[i][2]
        )
        target_pose.append(target_posei)
        # print("target_posei]:", target_pose, "\n", "len(target_pose):", len(target_pose), "\n")

    # create an empty 2D array
    all_pos_target = np.empty((0, 3))
    all_target_matrix = []
    # all_pos_target = []
    for i in range(len(target_pose)):

        rf_target2camera = np.dot(rf_surface2camera, target_pose[i])
        rf_target2base = np.dot(rf_camera2base, rf_target2camera)
        # print("target_ref_base:", rf_target2base, "\n")
        rot_target, pos_target = rotPos(rf_target2base)

        # print("rot_target:", rot_target, "\n", "pos_target:", pos_target, "\n")

        target_none_matrix = np.concatenate((pos_target, rot_target), axis=0)
        # print("target_none_matrix:", target_none_matrix, "\n")

        # convert to pose
        target_matrix = TxyzRxyz_2_Pose(target_none_matrix)
        all_target_matrix.append(target_matrix)

        # print("rot_target:", rot_target, "\n", "pos_target:", pos_target, "\n")
        pos_target = np.array([pos_target])
        all_pos_target = np.concatenate((all_pos_target, pos_target), axis=0)
        all_pos_target = all_pos_target.tolist()
        # print("type of all_pos_target:", type(all_pos_target), "\n")

    # print("pos_target:", all_pos_target, "\n")
    print("all_target_matrix:", len(all_target_matrix), "\n")
    # RDK.AddCurve(all_pos_target)

    speeds = [50, 10]
    count = 0
    first_target_point = all_target_matrix[0]
    robot.setSpeed(speeds[1])
    robot.MoveL(first_target_point)
    for i in range(1, len(all_target_matrix)):
        if count % 20 == 0:
            group_index = count // 20
            if group_index % 2 == 0:
                robot.setSpeed(speeds[0])
            else:
                robot.setSpeed(speeds[1])

        robot.MoveL(all_target_matrix[i])
        count += 1

    return all_pos_target, all_target_matrix


""" # forward kinematics
def transformation_matrix(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Các góc xoay của các khớp (radian)
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = np.radians(-90)
theta6 = np.radians(90)

# Tạo ma trận biến đổi cho từng khớp
T1 = transformation_matrix(0, 0, 0, theta1)
T2 = transformation_matrix(0, 0, 0, theta2)
T3 = transformation_matrix(0, 0, 0, theta3)
T4 = transformation_matrix(0, 0, 0, theta4)
T5 = transformation_matrix(0, 0, 0, theta5)
T6 = transformation_matrix(0, 0, 0, theta6)

# Tích các ma trận biến đổi
T_final = np.dot(np.dot(np.dot(np.dot(np.dot(T1, T2), T3), T4), T5), T6)

# Trục hệ tọa độ của mỗi khớp
axes = {
    'Khớp 1': T1[:3, :3],
    'Khớp 2': T2[:3, :3],
    'Khớp 3': T3[:3, :3],
    'Khớp 4': T4[:3, :3],
    'Khớp 5': T5[:3, :3],
    'Khớp 6': T6[:3, :3]
}

for k, v in axes.items():
    print(k + ':')
    print(v)
    print()

# Vị trí của khâu tác động cuối cùng (cột 4 của ma trận biến đổi)
position = T_final[:3, 3]
# Hướng của khâu tác động cuối cùng (các phần tử của cột 1, 2, 3 của ba cột đầu tiên)
orientation = T_final[:3, :3]

print("Vị trí của khâu tác động cuối cùng:", position)
print("Hướng của khâu tác động cuối cùng:", orientation)

# Các góc xoay của các khớp (radian)
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = np.radians(-90)
theta6 = np.radians(90)
"""
RDK = Robolink()
# robot = RDK.AddFile("E:\\Install-software\\RoboDK\\Library\\Motoman-GP8.robot")
robot = RDK.ItemUserPick("Yaskawa GP8 Base", ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("No robot selected or available")

RDK.Connect()
# reference frame from flange to base
pos_flange2base = [380, 0, 305]  # Translation vector [Tx, Ty, Tz]
rot_flange2base = [
    np.radians(-180),
    np.radians(0),
    np.radians(0),
]  # Rotation angles [Rx, Ry, Rz] in radians

# Create the transformation matrix for the new rf flange to base
rf_flage2base = createRef(pos_flange2base, rot_flange2base)
print("rf_flage2base:", rf_flage2base, "\n")

# reference frame camera to flange
pos_camera2flange = [0, 0, 200]  # Translation vector [Tx, Ty, Tz]
rot_camera2flange = [
    np.radians(0),
    np.radians(0),
    np.radians(0),
]  # Rotation angles [Rx, Ry, Rz] in radians

# Create the transformation matrix for the new rf camera to flange
rf_camera2flange = createRef(pos_camera2flange, rot_camera2flange)
print("ref_camera2flange:", rf_camera2flange, "\n")

rf_camera2flange_non_matrix = np.concatenate((pos_camera2flange, rot_camera2flange))
rf_camera2flange_matrix = TxyzRxyz_2_Pose(rf_camera2flange_non_matrix)
print("ref_camera2flange_test:", rf_camera2flange_matrix, "\n")

# reference frame surface to base
pos_surface2camera = [0, 0, 300]  # Translation vector [Tx, Ty, Tz]
rot_surface2camera = [0, 0, 0]  # Rotation angles [Rx, Ry, Rz] in radians

# Create the transformation matrix for the new rf surface to base
rf_surface2camera = createRef(pos_surface2camera, rot_surface2camera)
print("rf_surface2camera:", rf_surface2camera, "\n")

# reference frame camera to base
rf_camera2base = np.dot(rf_flage2base, rf_camera2flange)
print("ref_camera2base:", rf_camera2base, "\n")

rot_camera2base, pos_camera2base = rotPos(rf_camera2base)
# print(
#     "rot_camera2base:", rot_camera2base, "\n", "pos_camera2base:", pos_camera2base, "\n"
# )
rot_camera2base_non_matrix = np.concatenate((pos_camera2base, rot_camera2base))
print("rot_camera2base_non_matrix:", rot_camera2base_non_matrix, "\n")
rf_camera2base_matrix = TxyzRxyz_2_Pose(rot_camera2base_non_matrix)

rf_surface2base = np.dot(rf_camera2base, rf_surface2camera)
print("ref_surface2base:", rf_surface2base, "\n")

# initial begin position robot
target_pose = intialTarget(0, -50, 0)
print("target_pose:", target_pose, "\n")
print("rf_surface2camera:", rf_surface2camera, "\n")
rf_target2camera = np.dot(rf_surface2camera, target_pose)
print("rf_target2camera:", rf_target2camera, "\n")

# Apply the transformation to the target point
rf_target2base = np.dot(rf_camera2base, rf_target2camera)
print("target_ref_base:", rf_target2base, "\n")
rot_target, pos_target = rotPos(rf_target2base)

# combine position and rotation
target_none_matrix = np.concatenate((pos_target, rot_target))
# print("target_none_matrix:", target_none_matrix, "\n")

# convert to pose
target_matrix = TxyzRxyz_2_Pose(target_none_matrix)
print("target_matrix:", target_matrix, "\n")

robot.setPoseFrame(robot.PoseFrame())
# print(f"robot.PoseFrame():{robot.PoseFrame()}")
robot.setPoseTool(rf_camera2flange_matrix)
print(f"robot.PoseTool():{robot.PoseTool()}")
robot.setRounding(5)  # Set the rounding parameter
robot.setSpeed(10, 10)  # Set linear speed in mm/s
robot.setSpeedJoints(10)

robot.MoveJ(rf_camera2base_matrix)
start = time.time()
drawPolygon(rf_camera2base_matrix, 200, 0)
end = time.time()
print("time:", end - start)
robot.MoveJ(target_matrix)
# print("target_matrix:", target_matrix, "\n")

target_points, all_target_matrix = createPoint()

# robot.MoveJ(fixed_target_matrix)
# robot.setJoints(current_joint_values)

## draw the polygon

robot.MoveJ(rf_camera2base_matrix)
# current_joint_values = robot.Joints()
# print("current_joint_values:", current_joint_values, "\n")
robot.setJoints([0, 0, 0, 0, -90, 0])
# RDK.ShowRoboDK()
RDK.setRunMode(RUNMODE_RUN_ROBOT)
