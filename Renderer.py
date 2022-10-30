from OpenGL.GL import *
import numpy as np
from abc import *


class Renderer(metaclass=ABCMeta):
    @abstractmethod
    def render(self):
        pass


class KeyJoint:
    def __init__(self, joint_name=""):
        self.global_pos = []
        self.local_axis = [0, 0, 0]
        self.diff_angle = 0
        self.idx = -1
        self.transformation_matrix = np.array([0] * 16).reshape(4, 4)
        self.joint_name = joint_name

        # For second step (Only for a...)
        self.local_axis2 = [0, 0, 0]
        self.diff_angle2 = 0

    def reset(self):
        self.global_pos = []
        self.local_axis = [0, 0, 0]
        self.diff_angle = 0
        self.idx = -1
        self.transformation_matrix = np.array([0] * 16).reshape(4, 4)

        self.local_axis2 = [0, 0, 0]
        self.diff_angle2 = 0

    def set_global_position(self):
        self.global_pos = self.get_global_position()
    def get_global_position(self):
        # TODO: change 4 -> 0 after test
        # TODO: 매트릭스 .T 해봄.
        ret = self.transformation_matrix.T @ np.array([0, 0, 0, 1]).T
        ret = ret[:-1]
        return ret

    def set_local_axis(self, global_axis):
        self.local_axis = self.get_local_axis(global_axis)

    def set_local_axis2(self, global_axis):
        self.local_axis2 = self.get_local_axis(global_axis)

    def get_local_axis(self, global_axis):
        # TODO 어떻게 곱해야하지.... global axis를 a의 local에서 바라본 벡터로 나타내려면..
        local_axis = self.transformation_matrix @ global_axis
        return local_axis


class BvhRenderer(Renderer, metaclass=ABCMeta):
    def __init__(self, bvh_motion):
        self.bvh_motion = bvh_motion
        self.cur_frame = 0
        self.rendering_frame = 0
        self.posture_idx = 0
        self.channels_idx = 0
        self.animating_mode = False

        self.key_frame = -1
        self.joint_idx = -1
        self.key_joints = {'a': KeyJoint("RightForeArm"), 'b': KeyJoint("RightHand"), 'c': KeyJoint(),
                           'c_prime': KeyJoint(), 't': KeyJoint()}
        self.global_axis = [0, 0, 0]

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

        for key in self.key_joints:
            self.key_joints[key].reset()

        self.set_key_frame_infos(self.save_key_frame_infos)

        self.global_axis = self.get_global_axis()
        self.key_joints['a'].set_local_axis(self.global_axis)
        self.key_joints['b'].set_local_axis(self.global_axis)

    # Move target end effector by keyboard input WASD+QE
    def move_end_effector(self, offset):
        if self.key_frame < 0:
            return

        pos_t_before = self.key_joints['t'].global_pos  # TODO: 얘는 t의 global pos 바뀌어도 로컬에 복사돼있어서 상관없겠지..
        self.key_joints['t'].global_pos = (pos_t_before[0] + offset[0], pos_t_before[1] + offset[1], pos_t_before[2] + offset[2])

        # Get alpha and beta different in first step.
        if not self.get_diff_angles():
            self.key_joints['t'].global_pos = pos_t_before

        # TODO: Get r? (second step)
        self.set_for_c_prime_to_t()

    def set_for_c_prime_to_t(self):
        # Set c_prime's transformation matrix.
        self.set_key_frame_infos(self.get_c_prime_transformation_matrix)
        # Set c_prime's global_pos
        self.key_joints['c_prime'].set_global_position()

        pos_a = self.key_joints['a'].global_pos
        pos_c_prime = self.key_joints['c_prime'].global_pos
        pos_t = self.key_joints['t'].global_pos
        vec_a_c_prime = pos_c_prime - pos_a
        vec_a_t = pos_t - pos_a
        len_a_c_prime = np.linalg.norm(vec_a_c_prime)
        len_a_t = np.linalg.norm(vec_a_t)

        # Set a's local axis2 for second step
        normal_vector = self.get_normal_vector(vec_a_c_prime, vec_a_t)
        self.key_joints['a'].set_local_axis2(normal_vector)

        # Set a's diff angle2 for second step
        inner_product = np.dot(vec_a_c_prime, vec_a_t)
        tau = np.rad2deg(np.arccos(inner_product / (len_a_c_prime * len_a_t)))
        self.key_joints['a'].diff_angle2 = tau

    def set_key_frame_infos(self, set_func):
        self.channels_idx = 0
        self.posture_idx = 0
        self.joint_idx = -1
        root = self.bvh_motion.get_root()
        # To exclude matrix states made by camera control.
        glPushMatrix()
        glLoadIdentity()
        set_func(root, self.key_frame)
        glPopMatrix()

    def is_valid_triangle(self, a, b, c):
        if a + b >= c and b + c >= a and a + c >= b:
            return True
        else:
            return False

    def get_diff_angles(self):
        pos_a = self.key_joints['a'].global_pos
        pos_b = self.key_joints['b'].global_pos
        pos_c = self.key_joints['c'].global_pos
        pos_t = self.key_joints['t'].global_pos

        length_ac_after = np.linalg.norm(pos_t - pos_a)
        length_ac_before = np.linalg.norm(pos_c - pos_a)
        length_ab = np.linalg.norm(pos_b - pos_a)
        length_bc = np.linalg.norm(pos_c - pos_b)
        if not self.is_valid_triangle(length_ab, length_bc, length_ac_after):
            return False

        cos_angle_alpha = (np.power(length_ab, 2) + np.power(length_ac_before, 2) - np.power(length_bc, 2)) / (
                    2 * length_ab * length_ac_before)
        angle_alpha = np.rad2deg(np.arccos(cos_angle_alpha))

        cos_angle_alpha_after = (np.power(length_ab, 2) + np.power(length_ac_after, 2) - np.power(length_bc, 2)) / (
                    2 * length_ab * length_ac_after)
        angle_alpha_after = np.rad2deg(np.arccos(cos_angle_alpha_after))

        # TODO 반대..? alpha - alpha_after..?
        # self.diff_alpha = angle_alpha_after - angle_alpha
        # 이게 맞는듯..? 동작상..
        self.key_joints['a'].diff_angle = angle_alpha - angle_alpha_after

        cos_angle_beta = (np.power(length_ab, 2) + np.power(length_bc, 2) - np.power(length_ac_before, 2)) / (
                    2 * length_ab * length_bc)
        angle_beta = np.rad2deg(np.arccos(cos_angle_beta))

        cos_angle_beta_after = (np.power(length_ab, 2) + np.power(length_bc, 2) - np.power(length_ac_after, 2)) / (
                    2 * length_ab * length_bc)
        angle_beta_after = np.rad2deg(np.arccos(cos_angle_beta_after))

        # self.diff_beta = angle_beta_after - angle_beta
        self.key_joints['b'].diff_angle = angle_beta - angle_beta_after

        return True

    def get_normal_vector(self, vec1, vec2):
        # Get global axis vector by outer product
        # (Vertical vector to a, b, c plane.)
        normal_vector = np.cross(vec1, vec2)
        normal_vector = normal_vector / np.linalg.norm(normal_vector)
        normal_vector = np.append(normal_vector, np.array([0]))
        return normal_vector


    def get_global_axis(self):
        for key in self.key_joints:
            self.key_joints[key].set_global_position()

        pos_a = self.key_joints['a'].global_pos
        pos_b = self.key_joints['b'].global_pos
        pos_c = self.key_joints['c'].global_pos

        vec_ab = pos_b - pos_a
        vec_ac = pos_c - pos_a

        global_axis = self.get_normal_vector(vec_ab, vec_ac)
        return global_axis

    def get_current_transformation_matrix(self):
        # Get current transformation matrix.
        a = (GLfloat * 16)()
        glGetFloatv(GL_MODELVIEW_MATRIX, a)
        return np.reshape(np.array(a), (4, 4))

    def get_c_prime_transformation_matrix(self, joint, frame):
        glPushMatrix()

        # Translate or Rotate joint offset.
        glTranslatef(joint.offset[0], joint.offset[1], joint.offset[2])

        self.joint_idx += 1

        if joint.isEndEffector:
            if self.key_joints['c'].idx == self.joint_idx:
                matrix = self.get_current_transformation_matrix()
                self.key_joints['c_prime'].transformation_matrix = matrix

            glPopMatrix()
            return

        for key in self.key_joints:
            if key != 'a' and key != 'b':
                continue
            if self.key_joints[key].idx == self.joint_idx:
                local_axis = self.key_joints[key].local_axis
                glRotatef(self.key_joints[key].diff_angle, local_axis[0], local_axis[1], local_axis[2])

        self.transform_by_channel(frame)

        for child in joint.children:
            self.save_key_frame_infos(child, frame)
        glPopMatrix()

    def save_key_frame_infos(self, joint, frame):
        glPushMatrix()

        # Translate or Rotate joint offset.
        glTranslatef(joint.offset[0], joint.offset[1], joint.offset[2])

        self.joint_idx += 1

        if joint.isEndEffector:
            # Save first end effector. (target of Inverse Kinematics)
            if self.key_joints['c'].idx == -1 and self.key_joints['a'].idx != -1:
                matrix = self.get_current_transformation_matrix()
                self.key_joints['c'].transformation_matrix = matrix
                self.key_joints['c'].idx = self.joint_idx

                self.key_joints['t'].transformation_matrix = matrix

            glPopMatrix()
            return

        for key in self.key_joints:
            if key != 'a' and key != 'b':
                continue
            if self.bvh_motion.joint_list[self.channels_idx] == self.key_joints[key].joint_name:
                matrix = self.get_current_transformation_matrix()
                self.key_joints[key].transformation_matrix = matrix
                self.key_joints[key].idx = self.joint_idx

                local_axis = self.key_joints[key].local_axis
                glRotatef(self.key_joints[key].diff_angle, local_axis[0], local_axis[1], local_axis[2])

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
        if render_key_frame:
            glColor3ub(255, 0, 0)

        # Translate or Rotate joint offset.
        glTranslatef(joint.offset[0], joint.offset[1], joint.offset[2])

        self.joint_idx += 1

        if joint.isEndEffector:
            glPopMatrix()
            return

        if render_key_frame:
            for key in self.key_joints:
                if key != 'a' and key != 'b':
                    continue

                if self.joint_idx == self.key_joints[key].idx:
                    # For debugging. (Draw a's local axis)
                    glColor3ub(0, 255, 255)
                    glBegin(GL_LINES)
                    glVertex3fv(np.array([0, 0, 0]))
                    local_axis = self.key_joints[key].local_axis
                    glVertex3fv(np.array([local_axis[0], local_axis[1], local_axis[2]]))
                    glColor3ub(255, 255, 0)
                    glEnd()

                    glRotatef(self.key_joints[key].diff_angle, local_axis[0], local_axis[1], local_axis[2])

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

    def render_key_frame(self):
        self.posture_idx = 0
        self.channels_idx = 0
        self.joint_idx = -1
        root = self.bvh_motion.get_root()
        glColor3ub(255, 0, 0)
        self.rendering_frame = self.key_frame
        self.draw_frame_recursively(root, True)
        glColor3ub(255, 255, 255)

        # For Debugging
        pos_a = self.key_joints['a'].global_pos
        pos_b = self.key_joints['b'].global_pos
        pos_c = self.key_joints['c'].global_pos
        pos_t = self.key_joints['t'].global_pos

        glPushMatrix()
        glPointSize(10)
        glBegin(GL_POINTS)
        glColor3ub(50, 255, 0)
        glVertex3fv(np.array([pos_a[0], pos_a[1], pos_a[2]]))
        glVertex3fv(np.array([pos_b[0], pos_b[1], pos_b[2]]))
        glVertex3fv(np.array([pos_c[0], pos_c[1], pos_c[2]]))
        glVertex3fv(np.array([pos_t[0], pos_t[1], pos_t[2]]))
        glColor3ub(255, 255, 255)
        glEnd()
        glPopMatrix()

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
