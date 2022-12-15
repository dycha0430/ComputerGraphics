from Renderer import Renderer
from OpenGL.GL import *
from abc import *
import numpy as np
from ParticleSystem import ParticleSystem


class ParticleRenderer(Renderer):
    def __init__(self):
        # For Cube Test
        self.square_particle_system = ParticleSystem()
        self.rendering_square = False
        self.square_particle_system.add_plane_collider(np.array([0, 0, 0]), np.array([0, 1, 0]))

        # For particle system made by user input
        self.particle_system = ParticleSystem()
        self.rendering_particle_system = False
        self.clicked_particle_idx = -1
        self.selected_particles_idx = []
        self.particle_system.add_plane_collider(np.array([0, 0, 0]), np.array([0, 1, 0]))

        self.pointer = np.array([0, 0, 0])
        self.pointer_linked_particle_idx = 0

        self.bvh_joint = np.array([0, 0, 0])
        self.bvh_linked_particle_idx = 0

        self.init_mass_spring_model()

    def change_move_mode(self, select_particle=False):
        ret = True
        if select_particle:
            self.pointer_linked_particle_idx = self.selected_particles_idx[0]
            ret = self.particle_system.change_move_mode(selected_particle=self.selected_particles_idx[0])
        else:
            self.pointer_linked_particle_idx = 0
            ret = self.particle_system.change_move_mode()
        return ret

    def move_bvh_joint(self, position):
        self.bvh_joint = position
        self.particle_system.set_motion_connected_particle(self.bvh_joint)

    def change_motion_connected_mode(self, select_particle=False):
        if select_particle:
            self.bvh_linked_particle_idx = self.selected_particles_idx[0]
            self.particle_system.change_motion_connected_mode(selected_particle=self.selected_particles_idx[0])
        else:
            self.bvh_linked_particle_idx = 0
            self.particle_system.change_motion_connected_mode()

    def change_integration_method(self, method):
        self.particle_system.change_integration_method(method)
        self.square_particle_system.change_integration_method(method)

    def set_spring_kd(self, kd):
        self.square_particle_system.set_spring_kd(kd)

    def set_spring_ks(self, ks):
        self.square_particle_system.set_spring_ks(ks)

    def render(self):
        self.render_pointer()
        self.render_particle_system()
        if self.rendering_square:
            self.render_square()

    def click_particle(self, idx):
        self.clicked_particle_idx = idx

    def select_particle(self, idx):
        self.selected_particles_idx.append(idx)

    def remove_selected_particle(self, idx):
        self.selected_particles_idx.pop(idx)

    def clear_selected_particle_list(self):
        self.selected_particles_idx.clear()

    def contains_particle(self, idx):
        if idx in self.selected_particles_idx:
            return True
        return False

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
        self.particle_system.set_pointer_particle(self.pointer)

    def add_particle(self):
        self.particle_system.add_particle(self.pointer, 1)

    def add_spring(self):
        idx1 = self.selected_particles_idx[0]
        idx2 = self.selected_particles_idx[1]
        self.particle_system.add_spring(idx1, idx2)

    def render_particle_system(self):
        positions = self.particle_system.get_positions()
        glColor3ub(51, 255, 255)
        glPointSize(10.0)
        glBegin(GL_POINTS)
        for i, pos in enumerate(positions):
            if i == self.clicked_particle_idx:
                glColor3ub(255, 51, 255)
            glVertex3fv(np.array([pos[0], pos[1], pos[2]]))
            if i == self.clicked_particle_idx:
                glColor3ub(51, 255, 255)
        glEnd()

        glBegin(GL_LINES)
        for i in range(self.particle_system.spring_force.num_spring):
            pos1 = self.particle_system.spring_force.particles1[i].x
            pos2 = self.particle_system.spring_force.particles2[i].x
            glVertex3fv(np.array([pos1[0], pos1[1], pos1[2]]))
            glVertex3fv(np.array([pos2[0], pos2[1], pos2[2]]))
        glEnd()

        glColor3ub(255, 255, 255)












    # Cube particle model rendering for test.
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
        for i in range(self.square_particle_system.spring_force.num_spring):
            pos1 = self.square_particle_system.spring_force.particles1[i].x
            pos2 = self.square_particle_system.spring_force.particles2[i].x
            glVertex3fv(np.array([pos1[0], pos1[1], pos1[2]]))
            glVertex3fv(np.array([pos2[0], pos2[1], pos2[2]]))
        glEnd()

        glColor3ub(255, 255, 255)

    def init_mass_spring_model(self):
        height = 2
        particles = [[0, 0 + height, 0], [1, 0 + height, 0], [0, 0 + height, 1], [1, 0 + height, 1], [0, 1 + height, 0],
                     [1, 1 + height, 0], [0, 1 + height, 1], [1, 1 + height, 1]]

        for particle in particles:
            self.square_particle_system.add_particle(particle, 2)

        square_spring_indices1 = np.array([0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 6])
        square_spring_indices2 = np.array([1, 2, 3, 4, 5, 6, 7, 2, 3, 4, 5, 6, 7, 3, 4, 5, 6, 7, 4, 5, 6, 7, 5, 6, 7, 6, 7, 7])

        for i in range(len(square_spring_indices1)):
            self.square_particle_system.add_spring(square_spring_indices1[i], square_spring_indices2[i])
