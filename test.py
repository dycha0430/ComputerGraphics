import numpy as np
from MotionViewer import *

def hi():
    val = np.sqrt(2) / 2

    renderer.transformation_matrices.append(np.array([
        val, val * -1, 0, 0,
        val, val, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]).reshape(4, 4))

    global_pos = renderer.get_global_position(0)
    print("global!!! ", global_pos)
    assert global_pos[0] == 0
    assert global_pos[1] == 3
    assert global_pos[2] == 0

renderer = BvhRenderer("")

hi()