from Renderer import Renderer
from OpenGL.GL import *
from abc import *
import numpy as np
from ParticleSystem import ParticleSystem


class ParticleRenderer(Renderer, metaclass=ABCMeta):
    def __init__(self):
        self.particle_system = ParticleSystem(1)

        # For test
        src = np.array([1, 3, 1])
        self.particle_system.init_particles(src)

    def render(self):
        # state = self.particle_system.get_state()
        pos = self.particle_system.get_positions()
        glColor3ub(255, 255, 0)
        glPointSize(10.0)
        glBegin(GL_POINTS)
        glVertex3fv(np.array([pos[0], pos[1], pos[2]]))
        glEnd()
        glColor3ub(255, 255, 255)

    def create_mass_spring_model(self):
        pass
