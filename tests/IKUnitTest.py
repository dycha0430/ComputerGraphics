from MotionViewer import *
import pytest
import Util as util
from LimbIK.LimbIK import LimbIK
import numpy as np

###########################################################
@pytest.fixture
def limb_ik():
    limb_ik = LimbIK("")

    val = np.sqrt(2) / 2

    # X rotation
    limb_ik.key_joints['a'].transformation_matrix = ((np.array([
        1, 0, 0, 0,
        0, val, val * -1, 0,
        0, val, val, 0,
        0, 0, 0, 1
    ])).reshape(4, 4)).T

    # Translate x by -4
    limb_ik.key_joints['b'].transformation_matrix = ((np.array([
        1, 0, 0, -4,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ])).reshape(4, 4)).T

    # Z rotation
    limb_ik.key_joints['c'].transformation_matrix = (np.array([
        val, val * -1, 0, 0,
        val, val, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]).reshape(4, 4)).T

    # Y rotation
    limb_ik.key_joints['t'].transformation_matrix = (np.array([
        val, 0, val, 0,
        0, 1, 0, 0,
        val * -1, 0, val, 0,
        0, 0, 0, 1
    ]).reshape(4, 4)).T

    return limb_ik


# get_global_position 의 시작점을 4, 0, 0, 1로 한 것 기준.
def test_get_global_position_is_correct(limb_ik):
    global_pos = limb_ik.key_joints['c'].get_global_position(4)
    assert global_pos[0] == np.sqrt(8)
    assert global_pos[1] == np.sqrt(8)
    assert global_pos[2] == 0

    global_pos = limb_ik.key_joints['b'].get_global_position(4)
    assert global_pos[0] == 0
    assert global_pos[1] == 0
    assert global_pos[2] == 0


def test_get_local_axis_is_correct(limb_ik):
    global_axis = [-1, -1, 0, 0]
    local_axis = limb_ik.key_joints['c'].get_local_axis(global_axis)
    assert local_axis[0] < 0
    assert local_axis[1] == 0
    assert local_axis[2] == 0


def test_get_degree_between_vectors():
    vec1 = np.array([2, 2, 0])
    vec2 = np.array([-1, 2, 0])
    degree = util.get_degree_between_vectors(vec1, vec2)
    assert degree == 45 + np.rad2deg(np.arctan(0.5))


def test_get_normal_vector():
    vec1 = np.array([2, 2, 0])
    vec2 = np.array([-1, 2, 0])
    normal_vector = util.get_normal_vector(vec1, vec2)
    assert normal_vector[0] == 0
    assert normal_vector[1] == 0
    assert normal_vector[2] == 1


def test_get_degree_between_triangle():
    degree = util.get_degree_between_triangle(3, 3, 3)
    degree = round(degree, 2)
    assert degree == 60
    degree = util.get_degree_between_triangle(2, 5, 6)
    degree = round(degree, 2)
    assert degree == 110.49

def test_is_valid_triangle():
    is_valid = util.is_valid_triangle(3, 4, 5)
    assert is_valid == True
    is_valid = util.is_valid_triangle(3, 10, 4)
    assert is_valid == False
