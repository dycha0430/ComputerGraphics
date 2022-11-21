import numpy as np
from abc import *


class Force(metaclass=ABCMeta):
    def __init__(self):
        pass

    @abstractmethod
    def compute_force(self):
        pass


class GravityForce(Force, metaclass=ABCMeta):
    def __init__(self, particles):
        super().__init__()
        self.affected_particles = particles
        self.G = np.array([0, -0.0000098, 0]) # ..?

    def compute_force(self):
        for particle in self.affected_particles:
            particle.accumulate_force(particle.m * self.G)


class SpringForce(Force, metaclass=ABCMeta):
    def __init__(self, particles1, particles2, rest_lengths, num_spring):
        super().__init__()
        self.particles1 = particles1
        self.particles2 = particles2
        self.rest_lengths = rest_lengths
        self.num_spring = num_spring
        self.kd = 0.00000001
        self.ks = 0.00001

    def compute_force(self):
        for i in range(self.num_spring):
            p1 = self.particles1[i]
            p2 = self.particles2[i]
            delta_x = p1.x - p2.x
            len_delta_x = np.linalg.norm(delta_x)
            delta_v = p1.v - p2.v
            r = self.rest_lengths[i]
            r = 1.5
            f1 = self.ks * (len_delta_x - r) + self.kd * (np.inner(delta_v, delta_x) / len_delta_x)
            f1 = -1 * f1
            f1 = f1 * delta_x / len_delta_x
            f2 = -1 * f1
            self.particles1[i].accumulate_force(f1)
            self.particles2[i].accumulate_force(f2)


class FrictionForce(Force, metaclass=ABCMeta):
    def __init__(self):
        pass

    def compute_force(self):
        pass
