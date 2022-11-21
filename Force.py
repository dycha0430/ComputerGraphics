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
    def __init__(self, particles1, particles2, num_particle):
        super().__init__()
        self.particles1 = particles1
        self.particles2 = particles2
        self.num_particle = num_particle
        self.kd = 1
        self.ks = 1

    def compute_force(self):
        for i in range(self.num_particle):
            p1 = self.particles1[i]
            p2 = self.particles2[i]
            delta_x = np.subtract(p1.x, p2.x)
            len_delta_x = np.linalg.norm(delta_x)
            delta_v = np.subtract(p1.v, p2.v)
            r = 0 # ??? rest length of spring 이 모지..
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
