## NOTES:
# M = transformation matrix
# P_world = P_local * M_(local_to_world)
# M_(world_to_local) = M_(local_to_world)⁻1
# P_local = p_world * M_(world_to_local)
# https://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points
# M is computed as Translation * rotation
# http://www.inf.ed.ac.uk/teaching/courses/cg/lectures/cg3_2013.pdf
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


def np_swap_values(array, index1, index2):
    array[[index1, index2]] = array[[index2, index1]]
    return array


def test():
    pass


def get_rotation_from_compass(compass_data: float, degrees=True):
    """
    Compass is giving only yaw information
    :param degrees: If the compass data is in degrees
    :type compass_data: giving the yaw rotation.
    :return:
    """
    return get_3d_rotation_matrix_from_yaw_pitch_roll(compass_data, 0, 0, degrees=degrees)


def get_3d_rotation_matrix_from_yaw_pitch_roll(yaw: float = 0.0,
                                               pitch: float = 0.0,
                                               roll: float = 0.0,
                                               order='zyx',
                                               degrees=True,
                                               rounding_digits=12,
                                               verbose=False):
    """
    :param yaw: rotation around z-axis
    :param pitch: rotation around y-axis
    :param roll: rotation around z-axis
    :param degrees: if the rotation is in degrees
    :param rounding_digits: How many digits to take into count
    :param verbose: If we should print the result
    :return: combined rotation
    """

    # # Here is the math, but scipy has made it easy for us.

    yaw_radians = degree_to_radians(yaw)
    pitch_radians = degree_to_radians(pitch)
    roll_radians = degree_to_radians(roll)

    # Rotation around x axis (roll):
    x_rotation = np.zeros((3, 3), dtype=np.float)
    x_rotation[0][0] = 1
    x_rotation[1][1] = np.cos(roll_radians)
    x_rotation[1][2] = np.sin(roll_radians)
    x_rotation[2][1] = -np.sin(roll_radians)
    x_rotation[2][2] = np.cos(roll_radians)

    # Rotation around y axis (pitch):
    y_rotation = np.zeros((3, 3), dtype=np.float)
    y_rotation[0][0] = np.cos(pitch_radians)
    y_rotation[0][2] = -np.sin(pitch_radians)
    y_rotation[1][1] = 1
    y_rotation[2][0] = np.sin(pitch_radians)
    y_rotation[2][2] = np.cos(pitch_radians)
    # Rotation around z axis (yaw):
    z_rotation = np.zeros((3, 3), dtype=np.float)
    z_rotation[0][0] = np.cos(yaw_radians)
    z_rotation[0][1] = np.sin(yaw_radians)
    z_rotation[1][0] = -np.sin(yaw_radians)
    z_rotation[1][1] = np.cos(yaw_radians)
    z_rotation[2][2] = 1

    rotation_matrix = np.dot(np.dot(z_rotation, y_rotation), x_rotation)
    if verbose:
        print("rotation matrix around x-axis:\n", str(x_rotation))
        print('rotation matrix around y-axis:\n', str(y_rotation))
        print("rotation matrix around z-axis:\n", str(z_rotation))
        print("rotation matrix:\n", str(rotation_matrix))
    rm = rotation_matrix
    rm = rm.round(decimals=12)
    # if degrees:
    #     if yaw < 0:
    #         yaw += 360
    #     if pitch < 0:
    #         pitch += 360
    #     if roll < 0:
    #         roll += 360

    r = R.from_euler(order, [yaw, pitch, roll], degrees=degrees)

    rotation_matrix = np.array(r.as_matrix())
    rotation_matrix_rounded = np.round(rotation_matrix, decimals=rounding_digits)
    # d1 = R.from_matrix(rotation_matrix).as_euler('zyx', degrees=True)
    # r_test = R.from_matrix(rm)
    # d = r_test.as_euler("zyx", degrees=True)
    # d2 = r.as_euler("zyx", degrees=True)
    if verbose:
        print("Rotation matrix: \n", str(rotation_matrix_rounded))
    return r, rotation_matrix_rounded


def get_3d_rotation_matrix_from_yaw_pitch_roll_ue4(yaw: float = 0.0,
                                                   pitch: float = 0.0,
                                                   roll: float = 0.0,
                                                   order='zyx',
                                                   degrees=True,
                                                   rounding_digits=12,
                                                   verbose=False):
    """
    :param yaw: rotation around z-axis
    :param pitch: rotation around y-axis
    :param roll: rotation around z-axis
    :param degrees: if the rotation is in degrees
    :param rounding_digits: How many digits to take into count
    :param verbose: If we should print the result
    :return: combined rotation
    """

    # # Here is the math, but scipy has made it easy for us.

    yaw_radians = degree_to_radians(yaw)
    pitch_radians = degree_to_radians(pitch)
    roll_radians = degree_to_radians(roll)

    # Rotation around x axis (roll):
    x_rotation = np.zeros((3, 3), dtype=np.float)
    x_rotation[0][0] = 1
    x_rotation[1][1] = np.cos(roll_radians)
    x_rotation[1][2] = -np.sin(roll_radians)
    x_rotation[2][1] = np.sin(roll_radians)
    x_rotation[2][2] = np.cos(roll_radians)

    # Rotation around y axis (pitch):
    y_rotation = np.zeros((3, 3), dtype=np.float)
    y_rotation[0][0] = np.cos(pitch_radians)
    y_rotation[0][2] = np.sin(pitch_radians)
    y_rotation[1][1] = 1
    y_rotation[2][0] = -np.sin(pitch_radians)
    y_rotation[2][2] = np.cos(pitch_radians)
    # Rotation around z axis (yaw):
    z_rotation = np.zeros((3, 3), dtype=np.float)
    z_rotation[0][0] = np.cos(yaw_radians)
    z_rotation[0][1] = np.sin(yaw_radians)
    z_rotation[1][0] = -np.sin(yaw_radians)
    z_rotation[1][1] = np.cos(yaw_radians)
    z_rotation[2][2] = 1

    rotation_matrix = np.dot(np.dot(x_rotation, y_rotation), z_rotation)
    if verbose:
        print("rotation matrix around x-axis:\n", str(x_rotation))
        print('rotation matrix around y-axis:\n', str(y_rotation))
        print("rotation matrix around z-axis:\n", str(z_rotation))
        print("rotation matrix:\n", str(rotation_matrix))
    rm = rotation_matrix
    rm = rm.round(decimals=12)
    return x_rotation, y_rotation, z_rotation


def get_rotation_matrix(theta: float, ux: float, uy: float, uz: float, verbose=False):
    """
    NOT TESTED
    visualisation on http://www.inf.ed.ac.uk/teaching/courses/cg/lectures/cg3_2013.pdf slide 28
    :param theta: rotation in degrees
    :param ux: unit-value
    :param uy: unit-value
    :param uz: unit-value
    :return:
    """
    assert ux + uy + uz == 1
    c = np.cos(theta)
    s = np.sin(theta)

    rotation_matrix = np.zeros((4, 4), dtype=np.float)
    rotation_matrix[0][0] = ux * ux * (1 - c) + c
    rotation_matrix[0][1] = uy * ux * (1 - c) + uz * s
    rotation_matrix[0][2] = uz * ux * (1 - c) - uy * s

    rotation_matrix[1][0] = uz * ux * (1 - c) - uz * s
    rotation_matrix[1][1] = uz * ux * (1 - c) + c
    rotation_matrix[1][2] = uy * uz * (1 - c) + ux * s

    rotation_matrix[2][0] = ux * uz * (1 - c) + uy * s
    rotation_matrix[2][1] = uy * uz * (1 - c) - ux * s
    rotation_matrix[2][2] = uz * uz * (1 - c) + c

    rotation_matrix[3][3] = 1
    if verbose:
        print("Rotation matrix:\n", str(rotation_matrix))
    # TODO: test later if needed
    # r = R.from_rotvec(theta * np.array([ux, uy, uz]))
    # print(str(np.array(r.as_matrix())))

    return rotation_matrix


def find_transformation_matrix(translation: np.array, rotation: np.array):
    """
    Combines translation with rotations
    :param translation: array([ x ,  y ,  z])
    :param rotation: array([[ 0.51456517, -0.25333656,  0.81917231],
                            [ 0.16196059,  0.96687621,  0.19727939],
                            [-0.8420163 ,  0.03116053,  0.53855136]])
    :return: array([[ 0.51456517, -0.25333656,  0.81917231,  1.        ],
                    [ 0.16196059,  0.96687621,  0.19727939,  2.        ],
                    [-0.8420163 ,  0.03116053,  0.53855136,  0.5       ],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]])
    """
    transformation_matrix = np.vstack((np.hstack((rotation, translation[:, None])), [0, 0, 0, 1]))
    ## TEST:
    z = np.zeros((3, 1))
    R = np.append(rotation, z, axis=1)
    R = np.vstack((R, [0, 0, 0, 1]))
    R = np.vstack((np.hstack((rotation, np.array([[0], [0], [0]]))), [0, 0, 0, 1]))
    T = np.vstack(
        (np.hstack((np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.reshape(translation, (3, 1)))), [0, 0, 0, 1]))

    # return  Translate then rotate
    # return np.matmul(R, T)
    # testing = np.matmul(T, R)
    # return rotate then translate
    return transformation_matrix


def transform():
    pass


def degree_to_radians(degree):
    return (degree * np.pi) / 180


def radians_to_degree(radians):
    return (radians * 180) / np.pi


def hconcat_resize_min(im_list, interpolation=cv2.INTER_CUBIC):
    h_min = min(im.shape[0] for im in im_list)
    im_list_resize = [cv2.resize(im, (int(im.shape[1] * h_min / im.shape[0]), h_min), interpolation=interpolation)
                      for im in im_list]
    return cv2.hconcat(im_list_resize)


def vconcat_resize_min(im_list, interpolation=cv2.INTER_CUBIC):
    w_min = min(im.shape[1] for im in im_list)
    im_list_resize = [cv2.resize(im, (w_min, int(im.shape[0] * w_min / im.shape[1])), interpolation=interpolation)
                      for im in im_list]
    return cv2.vconcat(im_list_resize)


if __name__ == '__main__':
    get_3d_rotation_matrix_from_yaw_pitch_roll(yaw=90, pitch=90, roll=90, verbose=True)
