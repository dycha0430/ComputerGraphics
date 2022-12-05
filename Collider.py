from abc import *
from enum import Enum
import numpy as np
import Util


class CollisionType(Enum):
    COLLIDE = 0
    CONTACT = 1
    NOTHING = 2


class Collider(metaclass=ABCMeta):
    def __init__(self):
        pass

    @abstractmethod
    def detect_collision(self, pos, velocity):
        pass

    @abstractmethod
    def response_collision(self, velocity):
        pass

    @abstractmethod
    def apply_friction_force(self, particle):
        pass


class PlaneCollider(Collider, metaclass=ABCMeta):
    def __init__(self, point, norm_vec):
        self.norm_vec = norm_vec / np.linalg.norm(norm_vec)
        self.point = point

    def detect_collision(self, pos, velocity):
        eps = 0.0001
        N = self.norm_vec
        P = self.point
        diff = np.subtract(pos, P)

        NdotV = np.inner(N, velocity)
        if np.inner(diff, N) < eps and NdotV <= 0:
            if np.abs(NdotV) < eps:
                return CollisionType.CONTACT
            return CollisionType.COLLIDE

        return CollisionType.NOTHING

    def response_collision(self, velocity):
        k_r = 0.6

        v_n = np.inner(self.norm_vec, velocity) * self.norm_vec
        v_t = np.subtract(velocity, v_n)

        counter_velocity = np.add(v_t, -k_r * v_n)
        threshold = 0.0001
        if np.linalg.norm(counter_velocity) < threshold:
            counter_velocity = np.array([0, 0, 0])
        return counter_velocity

    def apply_friction_force(self, particle):
        coefficient = 0.3
        v_t = particle.v
        f_n = np.inner(particle.f, self.norm_vec)
        friction_force = -coefficient * Util.get_vector_length(f_n) * v_t / Util.get_vector_length(v_t)
        particle.accumulate_force(friction_force)


