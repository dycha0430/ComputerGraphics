import numpy as np
import Util
from Force import GravityForce, SpringForce


class ParticleSystem:
    def __init__(self):
        self.num = 0
        self.time: float = 0
        self.particles = []
        self.forces = []
        self.spring_force = SpringForce()
        self.gravity_force = GravityForce()
        self.forces.append(self.spring_force)
        self.forces.append(self.gravity_force)
        self.integration_method = 0

        self.move_mode = False
        self.pointer_particle = Particle()

    def set_pointer_particle(self, pointer):
        self.pointer_particle.x = pointer

    def change_move_mode(self, selected_particle=0):
        self.move_mode = not self.move_mode
        if self.num == 0:
            return
        if self.move_mode:
            self.spring_force.add_spring(self.particles[selected_particle], self.pointer_particle, is_pointer=True)
        else:
            self.spring_force.remove_spring()

    def change_integration_method(self, method):
        self.integration_method = method

    def set_spring_kd(self, kd):
        self.spring_force.set_kd(kd)

    def set_spring_ks(self, ks):
        self.spring_force.set_ks(ks)

    def add_particle_with_no_gravity(self, particle, weight):
        self.num += 1
        new_particle = Particle(particle, weight)
        self.particles.append(new_particle)

    def init_particles(self):
        for particle in self.particles:
            particle.x = particle.initial_x
            particle.v = np.array([0, 0, 0])
            particle.f = np.array([0, 0, 0])

    def add_particle(self, particle, weight):
        self.num += 1
        new_particle = Particle(np.array(particle), weight)
        self.particles.append(new_particle)
        self.gravity_force.add_particle(new_particle)

    def add_spring(self, index1, index2):
        self.spring_force.add_spring(self.particles[index1], self.particles[index2])

    def get_dims(self):
        return 6 * self.num

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
        for i in range(self.num):
            self.particles[i].x = np.array([src[i * 6], src[i * 6 + 1], src[i * 6 + 2]])
            self.particles[i].v = np.array([src[i * 6 + 3], src[i * 6 + 4], src[i * 6 + 5]])

    def calculate_derivative(self, delta_t):
        self.clear_forces()
        # self.compute_forces()

        for force in self.forces:
            force.compute_force(delta_t)

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
        self.time += delta_t

    def semi_implicit_euler_step(self, delta_t: float):
        tmp1 = self.calculate_derivative(delta_t)
        Util.scale_vector(tmp1, delta_t)
        tmp2 = self.get_state()
        tmp3 = Util.add_vectors(tmp1, tmp2)
        self.set_state(tmp3)

        self.process_stopped_particle()
        tmp4 = self.calculate_derivative(delta_t)
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
            ret = Util.detect_collision(particle.x, particle.v)
            if not ret:
                continue
            particle.v = Util.response_collision(particle.v)

    def process_stopped_particle(self):
        for particle in self.particles:
            ret = Util.detect_collision(particle.x, particle.v)
            if not ret:
                continue
            after_v = Util.response_collision(particle.v)
            if after_v[0] == 0 and after_v[1] == 0 and after_v[2] == 0:
                particle.v = after_v


class Particle:
    def __init__(self, x=np.array([0, 0, 0]), m=1):
        self.m: float = m
        self.initial_x = x
        self.x = x
        self.v = np.array([0, 0, 0])
        self.f = np.array([0, 0, 0])

    def accumulate_force(self, force):
        self.f = np.add(self.f, force)
