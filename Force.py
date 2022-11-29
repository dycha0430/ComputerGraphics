import numpy as np
from abc import *


class Force(metaclass=ABCMeta):
    def __init__(self):
        pass

    @abstractmethod
    def compute_force(self, delta_t):
        pass


class GravityForce(Force, metaclass=ABCMeta):
    def __init__(self):
        super().__init__()
        self.affected_particles = []
        self.G = np.array([0, -9.8, 0])

    def init_particles(self, particles):
        self.affected_particles = particles

    def add_particle(self, particle):
        self.affected_particles.append(particle)

    def compute_force(self, delta_t):
        for particle in self.affected_particles:
            particle.accumulate_force(particle.m * self.G * delta_t * delta_t)


class SpringForce(Force, metaclass=ABCMeta):
    def __init__(self):
        super().__init__()
        self.particles1 = []
        self.particles2 = []
        self.rest_lengths = []
        self.num_spring = 0
        self.kd = 50
        self.ks = 100000

    def remove_spring(self):
        self.particles1.pop(0)
        self.particles2.pop(0)
        self.rest_lengths.pop(0)
        self.num_spring -= 1

    def add_spring(self, particle1, particle2, is_pointer=False):
        if is_pointer:
            self.particles1.insert(0, particle1)
            self.particles2.insert(0, particle2)
            self.rest_lengths.insert(0, np.linalg.norm(particle1.x - particle2.x))
        else:
            self.particles1.append(particle1)
            self.particles2.append(particle2)
            self.rest_lengths.append(np.linalg.norm(particle1.x - particle2.x))
        self.num_spring += 1

    def init_spring(self, particles1, particles2, rest_lengths, num_spring):
        self.particles1 = particles1
        self.particles2 = particles2
        self.rest_lengths = rest_lengths
        self.num_spring = num_spring

    def set_kd(self, kd):
        self.kd = kd

    def set_ks(self, ks):
        self.ks = ks

    def compute_force(self, delta_t):
        for i in range(self.num_spring):
            p1 = self.particles1[i]
            p2 = self.particles2[i]
            delta_x = p1.x - p2.x
            len_delta_x = np.linalg.norm(delta_x)
            delta_v = p1.v - p2.v
            r = self.rest_lengths[i]
            f1 = self.ks * (len_delta_x - r) + self.kd * (np.inner(delta_v, delta_x) / len_delta_x)
            f1 = -1 * f1
            f1 = f1 * delta_x / len_delta_x
            f1 *= delta_t * delta_t
            f2 = -1 * f1
            self.particles1[i].accumulate_force(f1)
            self.particles2[i].accumulate_force(f2)


class FrictionForce(Force, metaclass=ABCMeta):
    def __init__(self):
        self.particles = []
        pass

    def init_particles(self, particles):
        self.particles = particles

    def compute_force(self, delta_t):
        pass
