import numpy as np
import Util
from Force import GravityForce, SpringForce

class ParticleSystem:
    def __init__(self, num):
        self.num = num
        self.time: float = 0
        self.particles = []
        for i in range(num):
            self.particles.append(Particle())
        self.forces = []
        self.forces.append(GravityForce(self.particles))
        # append more ...

    def init_particles(self, src, weights):
        for i in range(self.num):
            self.particles[i].x = np.array(src[i])
            self.particles[i].m = weights[i]

    def init_springs(self, indeces1, indeces2, num):
        particles1 = []
        particles2 = []
        rest_lengths = []

        for i in range(num):
            p1 = self.particles[indeces1[i]]
            p2 = self.particles[indeces2[i]]
            particles1.append(p1)
            particles2.append(p2)
            rest_lengths.append(np.linalg.norm(p1.x - p2.x))

        self.forces.append(SpringForce(particles1, particles2, rest_lengths, num))

    def get_dims(self):
        return 6 * self.num

    def get_positions(self):
        dst = []
        for particle in self.particles:
            dst.append([particle.x[0],  particle.x[1],  particle.x[2]])
            # dst = np.append(dst, np.array())
            # dst = np.append(dst, particle.x[0])
            # dst = np.append(dst, particle.x[1])
            # dst = np.append(dst, particle.x[2])

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

    # lecture node c 6페이지 그림 보고 함..
    # 거기에 각 force 객체들은 그것이 영향을 끼칠 particle들을 가리키고 있다고 함.
    # 그리고 각 영향을 받는 particle에 주어질 force를 어떻게
    # 계산하는지도 알고 있다고 함. (apply_force 함수)
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
        # collision 때매 v를 0으로 만들어도, 여기서 두 번 caculate_derivative 하면서
        # 가속도를 v에 더하고, v를 x에 더하게 되면서 x가 계속 조금씩 변함..
        # 어케 해결해야할지 모르겠음.
        tmp4 = self.calculate_derivative(delta_t)
        Util.scale_vector(tmp4, delta_t)
        tmp5 = Util.add_vectors(tmp2, tmp4)
        self.set_state(tmp5)

    def update_state(self, delta_t):
        # self.euler_step(delta_t)
        self.semi_implicit_euler_step(delta_t)
        for particle in self.particles:
            ret = Util.detect_collision(particle.x, particle.v)
            if not ret:
                continue
            particle.v = Util.response_collision(particle.v)



class Particle:
    def __init__(self):
        self.m: float = 1
        self.x = np.array([0, 0, 0])
        self.v = np.array([0, 0, 0])
        self.f = np.array([0, 0, 0])

    def accumulate_force(self, force):
        self.f = np.add(self.f, force)
