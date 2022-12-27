from OpenGL.GL import *
import numpy as np
from abc import *


class Renderer(metaclass=ABCMeta):
    @abstractmethod
    def render(self):
        pass


class BackgroundRenderer(Renderer):
    def render(self):
        glBegin(GL_LINES)
        line = 20
        for i in range(-line, line + 1):
            glVertex3fv(np.array([i, 0, -line]))
            glVertex3fv(np.array([i, 0, line]))
            glVertex3fv(np.array([-line, 0, i]))
            glVertex3fv(np.array([line, 0, i]))
        glEnd()

        self.render_axis()

    def render_plane(self):
        glBegin(GL_TRIANGLES)

    def render_axis(self):
        glColor3ub(255, 0, 0)
        glBegin(GL_LINES)
        glVertex3fv(np.array([0, 0, 0]))
        glVertex3fv(np.array([1, 0, 0]))
        glEnd()

        glColor3ub(0, 255, 0)
        glBegin(GL_LINES)
        glVertex3fv(np.array([0, 0, 0]))
        glVertex3fv(np.array([0, 1, 0]))
        glEnd()

        glColor3ub(0, 0, 255)
        glBegin(GL_LINES)
        glVertex3fv(np.array([0, 0, 0]))
        glVertex3fv(np.array([0, 0, 1]))
        glColor3ub(255, 255, 255)
        glEnd()
