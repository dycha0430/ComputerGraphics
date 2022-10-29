from OpenGL.GL import *
import numpy as np
from abc import *


class Renderer(metaclass=ABCMeta):
    @abstractmethod
    def render(self):
        pass


class BvhRenderer(Renderer, metaclass=ABCMeta):
    def __init__(self, bvh_motion):
        self.bvh_motion = bvh_motion
        self.cur_frame = 0
        self.rendering_frame = 0
        self.posture_idx = 0
        self.joint_idx = 0
        self.animating_mode = False

        self.key_frame = -1
        self.pointer = (0, 0, 0)
        self.transformation_matrices = []
        self.key_end_effector = -1

    def get_bvh(self):
        return self.bvh_motion

    def set_bvh(self, bvh_motion):
        self.bvh_motion = bvh_motion
        self.cur_frame = 0
        self.key_frame = -1
        self.animating_mode = False

    def get_frame_num(self):
        if self.bvh_motion is not None:
            return self.bvh_motion.get_frame_num()
        return 0

    def set_next_frame(self):
        if self.animating_mode:
            self.cur_frame += 1
            if self.cur_frame >= self.bvh_motion.get_frame_num(): self.cur_frame = 0

    def start_or_stop_animation(self):
        self.animating_mode = not self.animating_mode

    def set_key_frame(self, frame):
        if self.bvh_motion is None:
            return

        self.key_frame = frame[0]
        if self.get_frame_num() <= self.key_frame:
            self.key_frame = self.get_frame_num() - 1

        self.pointer = (0, 0, 0)

        self.joint_idx = 0
        self.posture_idx = 0
        self.key_end_effector = -1
        self.transformation_matrices = []

        root = self.bvh_motion.get_root()
        self.save_key_frame_infos(root, self.key_frame)

        self.get_global_axis()

    def move_pointer(self, offset):
        self.pointer = (
            self.pointer[0] + offset[0],
            self.pointer[1] + offset[1],
            self.pointer[2] + offset[2]
        )


    def get_global_axis(self):
        global_poses = []
        for i in range(0, 3):
            global_pos = self.get_global_position(self.key_end_effector - i)
            print(global_pos)
            global_poses.append(global_pos)

        vec_ab = global_poses[1] - global_poses[0]
        vec_ac = global_poses[2] - global_poses[0]

        global_axis = np.cross(vec_ac, vec_ab)
        global_axis = global_axis / np.linalg.norm(global_axis)
        return global_axis

    def get_global_position(self, joint_idx):
        # TODO: change 4 -> 0 after test
        ret = self.transformation_matrices[joint_idx] @ np.array([0, 0, 0, 1]).T
        ret = ret[:-1]
        return ret


    def save_key_frame_infos(self, joint, frame):
        glPushMatrix()

        # Translate or Rotate joint offset.
        glTranslatef(joint.offset[0], joint.offset[1], joint.offset[2])

        # Get current transformation matrix.
        a = (GLfloat * 16)()
        glGetFloatv(GL_MODELVIEW_MATRIX, a)
        arr = np.reshape(np.array(a), (4, 4))
        # Save current joint's transformation matrix.
        self.transformation_matrices.append(arr)

        if joint.isEndEffector:
            # Save first end effector. (target of Inverse Kinematics)
            if self.key_end_effector == -1:
                self.key_end_effector = len(self.transformation_matrices) - 1
                print("key : ", self.key_end_effector)
            glPopMatrix()
            return

        self.transform_by_channel(frame)

        for child in joint.children:
            self.save_key_frame_infos(child, frame)
        glPopMatrix()

    def draw_frame_recursively(self, joint):
        glPushMatrix()
        # Draw link.
        glBegin(GL_LINES)
        glVertex3fv(np.array([0, 0, 0]))
        glVertex3fv(np.array([joint.offset[0], joint.offset[1], joint.offset[2]]))
        glEnd()

        # Translate or Rotate joint offset.
        glTranslatef(joint.offset[0], joint.offset[1], joint.offset[2])

        if joint.isEndEffector:
            glPopMatrix()
            return

        self.transform_by_channel(self.rendering_frame)

        for child in joint.children:
            self.draw_frame_recursively(child)
        glPopMatrix()

    def transform_by_channel(self, frame):
        for chanType in self.bvh_motion.channels[self.joint_idx]:
            val = float(self.bvh_motion.get_value_in_frame(frame, self.posture_idx))
            self.posture_idx += 1
            if chanType == 'XPOSITION':
                glTranslatef(val, 0, 0)
            elif chanType == 'YPOSITION':
                glTranslatef(0, val, 0)
            elif chanType == 'ZPOSITION':
                glTranslatef(0, 0, val)
            elif chanType == 'XROTATION':
                glRotatef(val, 1, 0, 0)
            elif chanType == 'YROTATION':
                glRotatef(val, 0, 1, 0)
            elif chanType == 'ZROTATION':
                glRotatef(val, 0, 0, 1)
        self.joint_idx += 1

    def render(self):
        if self.bvh_motion is None:
            return
        self.rendering_frame = self.cur_frame
        self.posture_idx = 0
        self.joint_idx = 0
        root = self.bvh_motion.get_root()
        self.draw_frame_recursively(root)

        self.posture_idx = 0
        self.joint_idx = 0
        if self.key_frame >= 0:
            glColor3ub(255, 0, 0)
            self.rendering_frame = self.key_frame
            self.draw_frame_recursively(root)
            glColor3ub(255, 255, 255)


class BackgroundRenderer(Renderer, metaclass=ABCMeta):
    def render(self):
        glBegin(GL_LINES)
        line = 20
        for i in range(-line, line + 1):
            glVertex3fv(np.array([i, 0, -line]))
            glVertex3fv(np.array([i, 0, line]))
            glVertex3fv(np.array([-line, 0, i]))
            glVertex3fv(np.array([line, 0, i]))
        glEnd()
