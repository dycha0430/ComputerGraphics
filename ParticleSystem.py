import numpy as np
import Util
from Force import GravityForce, SpringForce
from Collider import Collider, PlaneCollider, CollisionType
from typing import List


class ParticleSystem:
    def __init__(self):
        self.particles = []
        self.forces = []
        self.spring_force = SpringForce()
        self.gravity_force = GravityForce()
        self.forces.append(self.spring_force)
        self.forces.append(self.gravity_force)
        self.integration_method = 0

        self.pointer_linked_mode = False
        self.pointer_particle = Particle()

        self.bvh_linked_mode = False
        self.bvh_linked_particle = Particle()

        self.colliders: List[Collider] = []

    def add_plane_collider(self, point, norm_vec):
        self.colliders.append(PlaneCollider(point, norm_vec))

    def set_pointer_particle_pos(self, pos):
        self.pointer_particle.x = pos

    def set_bvh_linked_particle_pos(self, pos):
        self.bvh_linked_particle.x = pos

    def change_pointer_linked_mode(self, selected_particle):
        if len(self.particles) == 0:
            return False
        self.pointer_linked_mode = not self.pointer_linked_mode
        if self.pointer_linked_mode:
            self.spring_force.add_spring(self.particles[selected_particle], self.pointer_particle)
        else:
            self.spring_force.remove_spring(self.pointer_particle)
        return True

    def change_bvh_linked_mode(self, selected_particle=0):
        if len(self.particles) == 0:
            return False
        self.bvh_linked_mode = not self.bvh_linked_mode
        if self.bvh_linked_mode:
            self.spring_force.add_spring(self.particles[selected_particle], self.bvh_linked_particle)
        else:
            self.spring_force.remove_spring(self.bvh_linked_particle)
        return True

    def change_integration_method(self, method):
        self.integration_method = method

    def set_spring_kd(self, kd):
        self.spring_force.set_kd(kd)

    def set_spring_ks(self, ks):
        self.spring_force.set_ks(ks)

    def init_particles(self):
        for particle in self.particles:
            particle.x = particle.initial_x
            particle.v = np.array([0, 0, 0])
            particle.f = np.array([0, 0, 0])

    def add_particle(self, pos, weight):
        new_particle = Particle(np.array(pos), weight)
        self.particles.append(new_particle)
        self.gravity_force.add_particle(new_particle)

    def add_spring(self, index1, index2):
        self.spring_force.add_spring(self.particles[index1], self.particles[index2])

    def get_positions(self):
        dst = []
        for particle in self.particles:
            dst.append([particle.x[0], particle.x[1], particle.x[2]])

        return dst

    def get_state(self):
        dst = np.array([])
        for particle in self.particles:
            dst = np.append(dst, particle.x[0])
            dst = np.append(dst, particle.x[1])
            dst = np.append(dst, particle.x[2])
            dst = np.append(dst, particle.v[0])
            dst = np.append(dst, particle.v[1])
            dst = np.append(dst, particle.v[2])

        return dst

    def set_state(self, src):
        for i in range(len(self.particles)):
            self.particles[i].x = np.array([src[i * 6], src[i * 6 + 1], src[i * 6 + 2]])
            self.particles[i].v = np.array([src[i * 6 + 3], src[i * 6 + 4], src[i * 6 + 5]])

    def calculate_derivative(self, delta_t):
        self.clear_forces()

        for force in self.forces:
            force.apply_force(delta_t)

        self.apply_friction_force()

        dst = np.array([])
        for particle in self.particles:
            dst = np.append(dst, particle.v[0])
            dst = np.append(dst, particle.v[1])
            dst = np.append(dst, particle.v[2])
            dst = np.append(dst, particle.f[0] / particle.m)
            dst = np.append(dst, particle.f[1] / particle.m)
            dst = np.append(dst, particle.f[2] / particle.m)

        return dst

    def clear_forces(self):
        for particle in self.particles:
            particle.f = np.array([0, 0, 0])

    def euler_step(self, delta_t: float):
        tmp1 = self.calculate_derivative(delta_t)
        Util.scale_vector(tmp1, delta_t)
        tmp2 = self.get_state()
        tmp2 = Util.add_vectors(tmp1, tmp2)
        self.set_state(tmp2)

    def semi_implicit_euler_step(self, delta_t: float):
        tmp1 = self.calculate_derivative(delta_t)
        Util.scale_vector(tmp1, delta_t)
        tmp2 = self.get_state()
        tmp3 = Util.add_vectors(tmp1, tmp2)
        self.set_state(tmp3)

        tmp4 = self.calculate_derivative(delta_t)

        tmp4 = self.process_stopped_particle(tmp4)
        Util.scale_vector(tmp4, delta_t)
        tmp5 = Util.add_vectors(tmp2, tmp4)
        self.set_state(tmp5)

    def update_state(self, delta_t):
        if self.integration_method == 0:
            self.semi_implicit_euler_step(delta_t)
        else:
            self.euler_step(delta_t)

        self.process_collision()

    def process_collision(self):
        for particle in self.particles:
            for collider in self.colliders:
                ret = collider.detect_collision(particle.x, particle.v)
                if ret == CollisionType.COLLIDE:
                    particle.v = collider.response_collision(particle.v)

    def apply_friction_force(self):
        for particle in self.particles:
            for collider in self.colliders:
                ret = collider.detect_collision(particle.x, particle.v)
                if ret == CollisionType.CONTACT:
                    collider.apply_friction_force(particle)

    def process_stopped_particle(self, derivative):
        for i, particle in enumerate(self.particles):
            for collider in self.colliders:
                ret = collider.detect_collision(particle.x, particle.v)
                if ret == CollisionType.CONTACT:
                    norm_v = collider.get_normal_component(np.array([derivative[i*6], derivative[i*6+1], derivative[i*6+2]]))
                    derivative[i*6] -= norm_v[0]
                    derivative[i * 6+1] -= norm_v[1]
                    derivative[i * 6+2] -= norm_v[2]
                    norm_f = collider.get_normal_component(np.array([derivative[i*6+3], derivative[i*6+4], derivative[i*6+5]]))
                    derivative[i * 6 + 3] -= norm_f[0]
                    derivative[i * 6 + 4] -= norm_f[1]
                    derivative[i * 6 + 5] -= norm_f[2]

        return derivative



class Particle:
    def __init__(self, x=np.array([0, 0, 0]), m=10):
        self.m: float = m
        self.initial_x = x
        self.x = x
        self.v = np.array([0, 0, 0])
        self.f = np.array([0, 0, 0])

    def accumulate_force(self, force):
        self.f = np.add(self.f, force)
