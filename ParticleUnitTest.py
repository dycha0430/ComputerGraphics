import ParticleSystem
import pytest
import Util
import numpy as np
from Collider import *


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


@pytest.fixture
def collider():
    collider = PlaneCollider(np.array([0, 0, 0]), np.array([0, 1, 0]))
    return collider


def test_detect_collision(collider):
    # In collision space and moving inside
    pos = np.array([1, 0.00001, 3])
    velocity = np.array([1, -1, 1])
    ret = collider.detect_collision(pos, velocity)
    assert(ret == CollisionType.COLLIDE)

    pos = np.array([1, 0.00001, 3])
    velocity = np.array([1, -0.00001, 1])
    ret = collider.detect_collision(pos, velocity)
    assert (ret == CollisionType.CONTACT)

    # Outside collision space and moving inside
    pos = np.array([0, 0.01, 10])
    velocity = np.array([1, -1, 1])
    ret = collider.detect_collision(pos, velocity)
    assert (ret == CollisionType.NOTHING)

    # Inside collision space and moving outside
    pos = np.array([1, 0.00001, 3])
    velocity = np.array([1, 0.5, 1])
    ret = collider.detect_collision(pos, velocity)
    assert (ret == CollisionType.NOTHING)

    # Outside collision space and moving outside
    pos = np.array([-10, 0.01, 3])
    velocity = np.array([1, 0.1, 1])
    ret = collider.detect_collision(pos, velocity)
    assert (ret == CollisionType.NOTHING)


def test_response_collision(collider):
    velocity = np.array([-3, -1, 3])
    counter_velocity = collider.response_collision(velocity)
    assert (counter_velocity[0] == -3)
    assert (counter_velocity[1] == 0.2) # assume that k_r is 0.2
    assert (counter_velocity[2] == 3)