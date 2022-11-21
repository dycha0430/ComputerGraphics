from Renderer import Renderer
from OpenGL.GL import *
from abc import *
import numpy as np
from ParticleSystem import ParticleSystem


class ParticleRenderer(Renderer, metaclass=ABCMeta):
    def __init__(self):
        self.num_particle = 8
        self.particle_system = ParticleSystem(self.num_particle)
        self.spring_indeces1 = None
        self.spring_indeces2 = None
        self.create_mass_spring_model()

    def render(self):
        # state = self.particle_system.get_state()
        positions = self.particle_system.get_positions()
        # print(positions)
        glColor3ub(255, 255, 0)
        glPointSize(10.0)
        glBegin(GL_POINTS)
        for pos in positions:
            glVertex3fv(np.array([pos[0], pos[1], pos[2]]))
        glEnd()
        glBegin(GL_LINES)
        for i in range(len(self.spring_indeces1)):
            pos1 = positions[self.spring_indeces1[i]]
            pos2 = positions[self.spring_indeces2[i]]
            glVertex3fv(np.array([pos1[0], pos1[1], pos1[2]]))
            glVertex3fv(np.array([pos2[0], pos2[1], pos2[2]]))

        glEnd()
        glColor3ub(255, 255, 255)

    def create_mass_spring_model(self):
        height = 5
        particles = []
        particles.append([0, 0+height, 0])
        particles.append([1, 0+height, 0])
        particles.append([0, 0+height, 1])
        particles.append([1, 0+height, 1])
        particles.append([0, 1 + height, 0])
        particles.append([1, 1 + height, 0])
        particles.append([0, 1 + height, 1])
        particles.append([1, 1 + height, 1])
        # particles = np.append(particles, np.array([0, 0, 0]))
        # particles = np.append(particles, np.array([0, 1, 0]))
        # particles = np.append(particles, np.array([0, 0, 1]))
        # particles = np.append(particles, np.array([0, 1, 1]))
        self.spring_indeces1 = np.array([0, 0, 1, 1, 2, 0, 1, 2, 3, 4, 4, 5, 5, 6, 1, 1, 3, 0, 4])
        self.spring_indeces2 = np.array([1, 2, 2, 3, 3, 4, 5, 6, 7, 5, 6, 6, 7, 7, 4, 7, 6, 6, 7])

        self.particle_system.init_particles(particles)
        self.particle_system.init_springs(self.spring_indeces1, self.spring_indeces2, len(self.spring_indeces1))
