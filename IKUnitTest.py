from MotionViewer import *
import pytest

###########################################################
@pytest.fixture
def renderer():
    renderer = BvhRenderer("")

    val = np.sqrt(2) / 2

    # X rotation
    renderer.transformation_matrices.append((np.array([
        1, 0, 0, 0,
        0, val, val * -1, 0,
        0, val, val, 0,
        0, 0, 0, 1
    ])).reshape(4, 4))

    # Z rotation
    renderer.transformation_matrices.append(np.array([
        val, val * -1, 0, 0,
        val, val, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]).reshape(4, 4))

    # Y rotation
    renderer.transformation_matrices.append(np.array([
        val, 0, val, 0,
        0, 1, 0, 0,
        val * -1, 0, val, 0,
        0, 0, 0, 1
    ]).reshape(4, 4))

    # Translate x by -4
    renderer.transformation_matrices.append((np.array([
        1, 0, 0, -4,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ])).reshape(4, 4))

    return renderer


# get_global_position 의 시작점을 4, 0, 0, 1로 한 것 기준.
def test_get_global_position_is_correct(renderer):
    global_pos = renderer.get_global_position(1)
    assert(np.linalg.det(renderer.transformation_matrices[0]) == 1)
    assert global_pos[0] == np.sqrt(8)
    assert global_pos[1] == np.sqrt(8)
    assert global_pos[2] == 0

    global_pos = renderer.get_global_position(3)
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

