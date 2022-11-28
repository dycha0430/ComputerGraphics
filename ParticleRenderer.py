from Renderer import Renderer
from OpenGL.GL import *
from abc import *
import numpy as np
from ParticleSystem import ParticleSystem


class ParticleRenderer(Renderer, metaclass=ABCMeta):
    def __init__(self):
        self.num_particle = 8
        self.square_particle_system = ParticleSystem()
        self.spring_indeces1 = None
        self.spring_indeces2 = None
        self.rendering_square = False
        self.particle_system = ParticleSystem()
        self.rendering_particle_system = False
        self.pointer = np.array([0, 0, 0])

        self.init_mass_spring_model()

    def render(self):
        self.render_pointer()
        self.render_particle_system()
        if self.rendering_square:
            self.render_square()

    def simulate_particle(self, time_stamp):
        if self.rendering_square:
            self.square_particle_system.update_state(time_stamp)
        if self.rendering_particle_system:
            self.particle_system.update_state(time_stamp)

    def set_render_particle_system(self):
        if not self.rendering_particle_system:
            self.particle_system.init_particles()
        self.rendering_particle_system = not self.rendering_particle_system

    def render_pointer(self):
        glColor3ub(0, 255, 0)
        glPointSize(10)
        glBegin(GL_POINTS)
        glVertex3fv(self.pointer)
        glEnd()
        glColor3ub(255, 255, 255)

    def move_pointer(self, offset):
        self.pointer = np.array([self.pointer[0] + offset[0], self.pointer[1] + offset[1], self.pointer[2] + offset[2]])

    def add_particle(self):
        self.particle_system.add_particle(self.pointer, 1)

    def render_particle_system(self):
        positions = self.particle_system.get_positions()
        glColor3ub(0, 102, 0)
        glPointSize(10.0)
        glBegin(GL_POINTS)
        for pos in positions:
            glVertex3fv(np.array([pos[0], pos[1], pos[2]]))
        glEnd()
        glColor3ub(255, 255, 255)





    def change_integration_method(self, method):
        self.square_particle_system.change_integration_method(method)

    def set_spring_kd(self, kd):
        self.square_particle_system.set_spring_kd(kd)

    def set_spring_ks(self, ks):
        self.square_particle_system.set_spring_ks(ks)

    def set_rendering_square(self):
        if not self.rendering_square:
            self.square_particle_system.init_particles()
        self.rendering_square = not self.rendering_square


    def render_square(self):
        positions = self.square_particle_system.get_positions()
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

    def init_mass_spring_model(self):
        height = 2
        particles = []
        particles.append([0, 0+height, 0])
        particles.append([1, 0+height, 0])
        particles.append([0, 0+height, 1])
        particles.append([1, 0+height, 1])
        particles.append([0, 1 + height, 0])
        particles.append([1, 1 + height, 0])
        particles.append([0, 1 + height, 1])
        particles.append([1, 1 + height, 1])

        for particle in particles:
            self.square_particle_system.add_particle(particle, 2)

        self.spring_indeces1 = np.array([0, 0, 1, 1, 2, 0, 1, 2, 3, 4, 4, 5, 5, 6, 1, 1, 3, 0, 4, 0, 0, 2, 3, 2])
        self.spring_indeces2 = np.array([1, 2, 2, 3, 3, 4, 5, 6, 7, 5, 6, 6, 7, 7, 4, 7, 6, 6, 7, 3, 5, 4, 5, 7])

        for i in range(len(self.spring_indeces1)):
            self.square_particle_system.add_spring(self.spring_indeces1[i], self.spring_indeces2[i])
