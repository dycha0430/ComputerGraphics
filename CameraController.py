import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *

class CameraController:
    def __init__(self):
        # Projection type : Perspective(True), Orthogonal(False)
        self.projection_type = True
        self.target = np.array([0., 0., 0.])
        self.u = np.array([1, 0, 0])
        self.v = np.array([0, 1, 0])
        self.w = np.array([0, 0, 1])
        self.azimuth = 45.
        self.elevation = 36.
        self.cur_xpos = 0
        self.cur_ypos = 0
        self.zooming = 0
        self.aspect = 1.0
        self.viewport_w = 800
        self.viewport_h = 800

    def set_viewport_size(self, width, height):
        self.viewport_w = width
        self.viewport_h = height
        self.aspect = float(width / height)

    def flip_projection(self):
        self.projection_type = not self.projection_type

    def change_orbit(self, xpos, ypos):
        self.azimuth += self.cur_xpos - xpos
        self.elevation -= self.cur_ypos - ypos

    def change_panning(self, xpos, ypos):
        self.target += 0.03 * (self.cur_xpos - xpos) * self.u
        self.target += 0.03 * (ypos - self.cur_ypos) * self.v

    def set_cur_pos(self, xpos, ypos):
        self.cur_xpos = xpos
        self.cur_ypos = ypos

    def zoom(self, yoffset):
        # zoom in 하면 (0, 1), zoom out하면 (0, -1)
        self.zooming += yoffset

    def init_viewport(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_NORMALIZE)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glLoadIdentity()

        glViewport(0, 0, self.viewport_w, self.viewport_h)
        if self.projection_type:
            gluPerspective(45, self.aspect, 2, 200)
        else:
            glOrtho(-10, 10, -10, 10, -50, 50)

        distance = 15
        r_azimuth = np.radians(self.azimuth)
        r_elevation = np.radians(self.elevation)
        tmp = distance * np.cos(r_elevation)
        camera = np.array([self.target[0] + tmp * np.sin(r_azimuth), self.target[1] + distance * np.sin(r_elevation),
                           self.target[2] + tmp * np.cos(r_azimuth)])
        Ma = np.array([[np.cos(r_azimuth), 0., np.sin(r_azimuth)],
                       [0., 1., 0.],
                       [-np.sin(r_azimuth), 0., np.cos(r_azimuth)]])
        Me = np.array([[1., 0., 0.],
                       [0., np.cos(r_elevation), np.sin(r_elevation)],
                       [0., -np.sin(r_elevation), np.cos(r_elevation)]])
        M = Ma @ Me
        self.u = M @ np.array([1., 0., 0.])
        self.v = M @ np.array([0., 1., 0.])
        self.w = M @ np.array([0., 0., 1.])
        self.u = self.u / np.sqrt(np.dot(self.u, self.u))
        self.v = self.v / np.sqrt(np.dot(self.v, self.v))
        self.w = self.w / np.sqrt(np.dot(self.w, self.w))
        camera -= self.zooming * self.w

        Mv = np.array([[self.u[0], self.u[1], self.u[2], -np.dot(self.u, camera)],
                       [self.v[0], self.v[1], self.v[2], -np.dot(self.v, camera)],
                       [self.w[0], self.w[1], self.w[2], -np.dot(self.w, camera)],
                       [0, 0, 0, 1]])
        glMultMatrixf(Mv.T)