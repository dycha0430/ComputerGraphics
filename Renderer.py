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
        self.channels_idx = 0
        self.animating_mode = False

        self.key_frame = -1
        self.diff_end_effector = (0, 0, 0)
        self.transformation_matrices = []
        self.key_end_effector = -1
        self.global_poses = []
        self.local_axis_a = [0, 0, 0]
        self.local_axis_b = [0, 0, 0]
        self.diff_alpha = 0
        self.diff_beta = 0
        self.idx_a = 0
        self.idx_b = 0
        self.idx_c = 0
        self.joint_idx = -1

        self.KEY_JOINTS = ["RightForeArm", "RightHand"]

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
        elif self.key_frame < 0:
            return

        self.diff_end_effector = (0, 0, 0)
        self.channels_idx = 0
        self.posture_idx = 0
        self.transformation_matrices = []
        self.idx_c = -1
        self.idx_b = -1
        self.idx_a = -1
        self.joint_idx = -1

        root = self.bvh_motion.get_root()
        self.save_key_frame_infos(root, self.key_frame)

        self.local_axis_a = self.get_local_axis(self.transformation_matrices[self.idx_a])
        self.local_axis_b = self.get_local_axis(self.transformation_matrices[self.idx_b])


    # Move target end effector by keyboard input WASD+QE
    def move_end_effector(self, offset):
        self.diff_end_effector = (
            self.diff_end_effector[0] + offset[0],
            self.diff_end_effector[1] + offset[1],
            self.diff_end_effector[2] + offset[2]
        )

    def get_diff_angle(self):
        pos_c = self.global_poses[0]
        pos_a = self.global_poses[2]
        pos_b = self.global_poses[1]
        pos_t = (pos_c[0] + self.diff_end_effector[0], pos_c[1] + self.diff_end_effector[1], pos_c[2] + self.diff_end_effector[2])

        length_ac_after = np.linalg.norm(pos_t - pos_a)
        length_ac_before = np.linalg.norm(pos_c - pos_a)
        length_ab = np.linalg.norm(pos_b - pos_a)
        length_bc = np.linalg.norm(pos_c - pos_b)

        cos_angle_alpha = (np.power(length_ab, 2) + np.power(length_ac_before, 2) - np.power(length_bc, 2)) / (2 * length_ab * length_ac_before)
        angle_alpha = np.arccos(cos_angle_alpha)

        cos_angle_alpha_after = (np.power(length_ab, 2) + np.power(length_ac_after, 2) - np.power(length_bc, 2)) / (2 * length_ab * length_ac_after)
        angle_alpha_after = np.arccos(cos_angle_alpha_after)

        # TODO 반대..? alpha - alpha_after..?
        self.diff_alpha = angle_alpha_after - angle_alpha
        #self.diff_alpha = angle_alpha - angle_alpha_after

        cos_angle_beta = (np.power(length_ab, 2) + np.power(length_bc, 2) - np.power(length_ac_before, 2)) / (2 * length_ab * length_bc)
        angle_beta = np.arccos(cos_angle_beta
                               )
        cos_angle_beta_after = (np.power(length_ab, 2) + np.power(length_bc, 2) - np.power(length_ac_after, 2)) / (2 * length_ab * length_bc)
        angle_beta_after = np.arccos(cos_angle_beta_after)

        self.diff_beta = angle_beta_after - angle_beta
        #self.diff_beta = angle_beta - angle_beta_after


    def get_local_axis(self, local_matrix):
        global_axis = self.get_global_axis()
        # TODO 어떻게 곱해야하지.... global axis를 a의 local에서 바라본 벡터로 나타내려면..
        local_axis = local_matrix @ global_axis
        #local_axis = global_axis @ local_matrix.T
        # local_axis = global_axis @ local_matrix? .T?
        return local_axis

    def get_global_axis(self):
        self.global_poses = []
        # c, b, a 순으로 global position 저장.
        idxes = [self.idx_c, self.idx_b, self.idx_a]
        for idx in idxes:
            global_pos = self.get_global_position(idx)
            print(global_pos)
            self.global_poses.append(global_pos)

        vec_ab = self.global_poses[0] - self.global_poses[2]
        vec_ac = self.global_poses[1] - self.global_poses[2]

        # Get global axis vector by outer product
        # (Vertical vector to a, b, c plane.)
        global_axis = np.cross(vec_ac, vec_ab)
        # Normalize vector.
        global_axis = global_axis / np.linalg.norm(global_axis)
        global_axis = np.append(global_axis, np.array([0]))
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

        self.joint_idx += 1

        if joint.isEndEffector:
            # Save first end effector. (target of Inverse Kinematics)
            if self.idx_c == -1 and self.idx_a != -1:
                self.key_end_effector = self.joint_idx
                print("key : ", self.key_end_effector)

            glPopMatrix()
            return

        if self.bvh_motion.joint_list[self.channels_idx] == self.KEY_JOINTS[0]:
            self.idx_a = self.joint_idx
        elif self.bvh_motion.joint_list[self.channels_idx] == self.KEY_JOINTS[1]:
            self.idx_b = self.joint_idx

        self.transform_by_channel(frame)

        for child in joint.children:
            self.save_key_frame_infos(child, frame)
        glPopMatrix()

    def draw_frame_recursively(self, joint, render_key_frame):
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

        if render_key_frame:
            if self.channels_idx == self.idx_a:
                glRotatef(self.diff_alpha, self.local_axis_a[0], self.local_axis_a[1], self.local_axis_a[2])
            elif self.channels_idx == self.idx_b:
                glRotatef(self.diff_beta, self.local_axis_b[0], self.local_axis_b[1], self.local_axis_b[2])

        self.transform_by_channel(self.rendering_frame)

        for child in joint.children:
            self.draw_frame_recursively(child, render_key_frame)
        glPopMatrix()

    def transform_by_channel(self, frame):
        for chanType in self.bvh_motion.channels[self.channels_idx]:
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
        self.channels_idx += 1

    def render_moved_key_frame(self):
        glColor3ub(0, 255, 0)

        glColor3ub(255, 255, 255)

    def render_key_frame(self):
        self.get_diff_angle()
        self.posture_idx = 0
        self.channels_idx = 0
        root = self.bvh_motion.get_root()
        glColor3ub(255, 0, 0)
        self.rendering_frame = self.key_frame
        self.draw_frame_recursively(root, True)
        glColor3ub(255, 255, 255)

    def render(self):
        if self.bvh_motion is None:
            return
        self.rendering_frame = self.cur_frame
        self.posture_idx = 0
        self.channels_idx = 0
        root = self.bvh_motion.get_root()
        self.draw_frame_recursively(root, False)

        if self.key_frame >= 0:
            self.render_key_frame()


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
