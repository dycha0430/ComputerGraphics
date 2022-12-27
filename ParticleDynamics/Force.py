import numpy as np
from abc import *


class Force(metaclass=ABCMeta):
    def __init__(self):
        pass

    @abstractmethod
    def apply_force(self, delta_t):
        pass


class GravityForce(Force, metaclass=ABCMeta):
    def __init__(self):
        super().__init__()
        self.affected_particles = []
        self.G = np.array([0, -9.8, 0])

    def add_particle(self, particle):
        self.affected_particles.append(particle)

    def apply_force(self, delta_t):
        for particle in self.affected_particles:
            particle.accumulate_force(particle.m * self.G * delta_t * delta_t)


class SpringForce(Force, metaclass=ABCMeta):
    def __init__(self):
        super().__init__()
        self.particles1 = []
        self.particles2 = []
        self.rest_lengths = []
        self.kd = 50
        self.ks = 10

    def get_spring_num(self):
        return len(self.particles1)

    def remove_spring(self, particle):
        if particle in self.particles1:
            idx = self.particles1.index(particle)
            self.particles1.pop(idx)
            self.particles2.pop(idx)
            self.rest_lengths.pop(idx)
        elif particle in self.particles2:
            idx = self.particles2.index(particle)
            self.particles1.pop(idx)
            self.particles2.pop(idx)
            self.rest_lengths.pop(idx)

    def add_spring(self, particle1, particle2):
        self.particles1.append(particle1)
        self.particles2.append(particle2)
        self.rest_lengths.append(np.linalg.norm(particle1.x - particle2.x))

    def set_kd(self, kd):
        self.kd = kd

    def set_ks(self, ks):
        self.ks = ks

    def apply_force(self, delta_t):
        for i in range(self.get_spring_num()):
            p1 = self.particles1[i]
            p2 = self.particles2[i]
            delta_x = p1.x - p2.x
            len_delta_x = np.linalg.norm(delta_x)
            delta_v = p1.v - p2.v
            r = self.rest_lengths[i]
            f1 = self.ks * (len_delta_x - r) + self.kd * (np.inner(delta_v, delta_x) / len_delta_x)
            f1 = -1 * f1
            f1 = f1 * delta_x / len_delta_x
            f1 *= delta_t
            f2 = -1 * f1
            self.particles1[i].accumulate_force(f1)
            self.particles2[i].accumulate_force(f2)
