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
        self.local_axis = np.array([0, 0, 0, 0])
        self.diff_angle = 0
        self.idx = -1
        self.transformation_matrix = np.array([0] * 16).reshape(4, 4)
        self.joint_name = joint_name

        # For second step (Only for a...)
        self.local_axis2 = np.array([0, 0, 0, 0])
        self.diff_angle2 = 0

    def reset(self):
        self.global_pos = []
        self.local_axis = np.array([0, 0, 0, 0])
        self.diff_angle = 0
        self.idx = -1
        self.transformation_matrix = np.array([0] * 16).reshape(4, 4)

        self.local_axis2 = np.array([0, 0, 0, 0])
        self.diff_angle2 = 0

    def set_global_position(self):
        self.global_pos = self.get_global_position()

    def get_global_position(self, test_x=0):
        # TODO: change 4 -> 0 after test
        ret = self.transformation_matrix.T @ np.array([test_x, 0, 0, 1]).T
        ret = ret[:-1]
        return ret

    def set_local_axis(self, global_axis):
        self.local_axis = self.get_local_axis(global_axis)

    def set_local_axis2(self, global_axis):
        self.local_axis2 = self.get_local_axis(global_axis)

    def get_local_axis(self, global_axis):
        local_axis = global_axis @ self.transformation_matrix.T

        local_axis.squeeze()
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
        self.key_joints = {'a': KeyJoint("RightForeArm"), 'b': KeyJoint("RightHand"), 'c': KeyJoint(), 't': KeyJoint(), 'c_prime': KeyJoint()}
        self.global_axis = np.array([0, 0, 0, 0])
        self.global_axis2 = np.array([0, 0, 0, 0])

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

        self.set_key_frame_infos()

        self.global_axis = self.get_global_axis()
        self.key_joints['a'].set_local_axis(self.global_axis)
        self.key_joints['b'].set_local_axis(self.global_axis)

    # Move target end effector by keyboard input WASD+QE
    def move_end_effector(self, offset):
        if self.key_frame < 0:
            return

        pos_t_before = self.key_joints['t'].global_pos
        self.key_joints['t'].global_pos = (pos_t_before[0] + offset[0], pos_t_before[1] + offset[1], pos_t_before[2] + offset[2])

        # Get alpha and beta different in first step.
        if not self.get_diff_angles():
            self.key_joints['t'].global_pos = pos_t_before

        # Get tau value and normal vector of ac't plane (second step)
        self.set_for_second_step()

    def set_for_second_step(self):
        self.set_key_frame_infos()

        # Set c_prime's global_pos
        self.key_joints['c_prime'].set_global_position()

        pos_a = self.key_joints['a'].global_pos
        pos_c_prime = self.key_joints['c_prime'].global_pos
        pos_t = self.key_joints['t'].global_pos
        vec_a_c_prime = pos_c_prime - pos_a
        vec_a_t = pos_t - pos_a

        # Set a's local axis2 for second step
        normal_vector = np.array([0, 0, 0, 0])
        if not np.array_equal(vec_a_c_prime, vec_a_t):
            normal_vector = self.get_normal_vector(vec_a_c_prime, vec_a_t) # TODO: 외적 방향..
        self.global_axis2 = normal_vector
        self.key_joints['a'].set_local_axis2(normal_vector) # TODO: 이때 a의 프레임은 {a} 프레임인데,, 알파 적용하기 전..

        # Set a's diff angle2 for second step
        tau = 0
        if not np.array_equal(vec_a_c_prime, vec_a_t):
            tau = self.get_degree_between_vectors(vec_a_c_prime, vec_a_t)
        self.key_joints['a'].diff_angle2 = tau

    def get_degree_between_vectors(self, vec1, vec2):
        len_vec1 = np.linalg.norm(vec1)
        len_vec2 = np.linalg.norm(vec2)
        inner_product = np.inner(vec1, vec2)
        degree = np.rad2deg(np.arccos(inner_product / (len_vec1 * len_vec2)))
        return degree

    def set_key_frame_infos(self):
        for key in self.key_joints:
            self.key_joints[key].idx = -1

        self.channels_idx = 0
        self.posture_idx = 0
        self.joint_idx = -1
        root = self.bvh_motion.get_root()
        # To exclude matrix states made by camera control.
        glPushMatrix()
        glLoadIdentity()
        self.save_key_frame_infos(root, self.key_frame)
        glPopMatrix()

    def is_valid_triangle(self, a, b, c):
        if a + b >= c and b + c >= a and a + c >= b:
            return True
        else:
            return False

    def get_degree_between_triangle(self, a, b, c):
        # Return degree between a and b
        cos_angle_alpha = (np.power(a, 2) + np.power(b, 2) - np.power(c, 2)) / (
                2 * a * b)
        return np.rad2deg(np.arccos(cos_angle_alpha))

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

        angle_alpha = self.get_degree_between_triangle(length_ab, length_ac_before, length_bc)
        angle_alpha_after = self.get_degree_between_triangle(length_ab, length_ac_after, length_bc)

        self.key_joints['a'].diff_angle = angle_alpha - angle_alpha_after

        angle_beta = self.get_degree_between_triangle(length_ab, length_bc, length_ac_before)
        angle_beta_after = self.get_degree_between_triangle(length_ab, length_bc, length_ac_after)

        self.key_joints['b'].diff_angle = angle_beta - angle_beta_after

        return True

    def get_normal_vector(self, vec1, vec2):
        # Get global axis vector by outer product
        # (Vertical vector to a, b, c plane.)
        normal_vector = np.cross(vec1, vec2)
        normal_vector = np.divide(normal_vector, np.linalg.norm(normal_vector))
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
                self.key_joints['c_prime'].transformation_matrix = matrix
                self.key_joints['c'].idx = self.joint_idx
                self.key_joints['t'].transformation_matrix = matrix

            glPopMatrix()
            return

        self.transform_by_channel(frame)

        for key in self.key_joints:
            if key != 'a' and key != 'b':
                continue
            if self.bvh_motion.joint_list[self.channels_idx-1] == self.key_joints[key].joint_name:
                matrix = self.get_current_transformation_matrix()
                self.key_joints[key].transformation_matrix = matrix
                self.key_joints[key].idx = self.joint_idx

                local_axis = self.key_joints[key].get_local_axis(self.global_axis)

                glRotatef(self.key_joints[key].diff_angle, local_axis[0], local_axis[1], local_axis[2])

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

        self.transform_by_channel(self.rendering_frame)

        if render_key_frame:
            for key in self.key_joints:
                if key != 'a' and key != 'b':
                    continue

                if self.joint_idx == self.key_joints[key].idx:
                    local_axis = self.key_joints[key].local_axis
                    glColor3ub(255, 255, 0)

                    if key == 'a':
                        local_axis2 = self.key_joints['a'].local_axis2
                        glRotatef(self.key_joints['a'].diff_angle2, local_axis2[0], local_axis2[1], local_axis2[2])
                    glRotatef(self.key_joints[key].diff_angle, local_axis[0], local_axis[1], local_axis[2])


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
        pos_t = self.key_joints['t'].global_pos
        pos_a = self.key_joints['a'].global_pos
        pos_b = self.key_joints['b'].global_pos
        pos_c = self.key_joints['c'].global_pos

        glPushMatrix()
        glPointSize(5)
        glBegin(GL_POINTS)
        glColor3ub(50, 255, 0)
        glVertex3fv(np.array([pos_t[0], pos_t[1], pos_t[2]]))
        glEnd()

        glBegin(GL_LINES)
        glColor3ub(255, 0, 0)
        # ab
        glVertex3fv(np.array([pos_a[0], pos_a[1], pos_a[2]]))
        glVertex3fv(np.array([pos_b[0], pos_b[1], pos_b[2]]))
        # ac
        glVertex3fv(np.array([pos_b[0], pos_b[1], pos_b[2]]))
        glVertex3fv(np.array([pos_c[0], pos_c[1], pos_c[2]]))
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
