from MotionViewer import *
import pytest

###########################################################
@pytest.fixture
def renderer():
    renderer = BvhRenderer("")

    val = np.sqrt(2) / 2

    # X rotation
    renderer.key_joints['a'].transformation_matrix = ((np.array([
        1, 0, 0, 0,
        0, val, val * -1, 0,
        0, val, val, 0,
        0, 0, 0, 1
    ])).reshape(4, 4)).T

    # Translate x by -4
    renderer.key_joints['b'].transformation_matrix = ((np.array([
        1, 0, 0, -4,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ])).reshape(4, 4)).T

    # Z rotation
    renderer.key_joints['c'].transformation_matrix = (np.array([
        val, val * -1, 0, 0,
        val, val, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]).reshape(4, 4)).T

    # Y rotation
    renderer.key_joints['t'].transformation_matrix = (np.array([
        val, 0, val, 0,
        0, 1, 0, 0,
        val * -1, 0, val, 0,
        0, 0, 0, 1
    ]).reshape(4, 4)).T

    return renderer


# get_global_position 의 시작점을 4, 0, 0, 1로 한 것 기준.
def test_get_global_position_is_correct(renderer):
    global_pos = renderer.key_joints['c'].get_global_position(4)
    assert global_pos[0] == np.sqrt(8)
    assert global_pos[1] == np.sqrt(8)
    assert global_pos[2] == 0

    global_pos = renderer.key_joints['b'].get_global_position(4)
    assert global_pos[0] == 0
    assert global_pos[1] == 0
    assert global_pos[2] == 0


def test_get_global_axis_is_correct(renderer):
    renderer.key_end_effector = 3
    global_axis = renderer.get_global_axis()

    assert global_axis[0] == -1 * global_axis[1]
    assert -1 * global_axis[1] == global_axis[2]
    assert global_axis[2] == global_axis[0]
    assert global_axis[0] < 0


def test_get_local_axis_is_correct(renderer):
    global_axis = renderer.get_global_axis()
    local_axis = renderer.get_local_axis()

    print("global")
    print(global_axis)
    print("local")
    print(local_axis)
    assert global_axis[0] == local_axis[0]
    assert global_axis[1] == local_axis[1]
    assert global_axis[2] == local_axis[2]

def test_get_degree_between_vectors(renderer):
    vec1 = np.array([2, 2, 0])
    vec2 = np.array([-1, 2, 0])
    degree = renderer.get_degree_between_vectors(vec1, vec2)
    assert degree == 45 + np.rad2deg(np.arctan(0.5))

def test_get_normal_vector(renderer):
    vec1 = np.array([2, 2, 0])
    vec2 = np.array([-1, 2, 0])
    normal_vector = renderer.get_normal_vector(vec1, vec2)
    assert normal_vector[0] == 0
    assert normal_vector[1] == 0
    assert normal_vector[2] == 1

def test_get_degree_between_triangle(renderer):
    degree = renderer.get_degree_between_triangle(3, 3, 3)
    degree = round(degree, 2)
    assert degree == 60
    degree = renderer.get_degree_between_triangle(2, 5, 6)
    degree = round(degree, 2)
    assert degree == 110.49
