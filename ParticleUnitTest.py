import ParticleSystem
import pytest
import Util
import numpy as np


def test_add_two_vectors():
    vec1 = np.array([1, 2, 3])
    vec2 = np.array([3, 2, 1])

    ret = Util.add_vectors(vec1, vec2)
    assert(ret[0] == 4)
    assert(ret[1] == 4)
    assert(ret[2] == 4)


def test_scale_vector():
    vec = np.array([4, 8, 16])
    delta = 0.25

    ret = Util.scale_vector(vec, delta)
    assert(ret[0] == 1)
    assert(ret[1] == 2)
    assert(ret[2] == 4)


def test_detect_collision():
    # In collision space and moving inside
    pos = np.array([1, 0.009, 3])
    velocity = np.array([1, -1, 1])
    ret = Util.detect_collision(pos, velocity)
    assert(ret == True)

    # Outside collision space and moving inside
    pos = np.array([0, 0.011, 10])
    velocity = np.array([1, -1, 1])
    ret = Util.detect_collision(pos, velocity)
    assert (ret == False)

    # Inside collision space and moving outside
    pos = np.array([1, 0.009, 3])
    velocity = np.array([1, 0.5, 1])
    ret = Util.detect_collision(pos, velocity)
    assert (ret == False)

    # Outside collision space and moving outside
    pos = np.array([-10, 0.011, 3])
    velocity = np.array([1, 0.1, 1])
    ret = Util.detect_collision(pos, velocity)
    assert (ret == False)
