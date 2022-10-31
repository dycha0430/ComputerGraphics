from OpenGL.GL import *
import numpy as np
import Util as util
from KeyJoint import KeyJoint
from enum import Enum

class Step(Enum):
    FIRST = 1
    SECOND = 2

class LimbIK:
    def __init__(self, bvh_motion):
        self.bvh_motion = bvh_motion
        self.key_frame = -1
        self.key_joints = {'a': KeyJoint("RightArm"), 'b': KeyJoint("RightForeArm"), 'c': KeyJoint(), 't': KeyJoint(),
                           'c_prime': KeyJoint()}
        self.global_axis = np.array([0, 0, 0, 0])
        self.global_axis2 = np.array([0, 0, 0, 0])

        self.posture_idx = 0
        self.channels_idx = 0
        self.joint_idx = -1

    def set_bvh(self, bvh_motion):
        self.bvh_motion = bvh_motion
        self.key_frame = -1

    def get_key_frame(self):
        return self.key_frame

    def set_key_frame(self, frame):
        self.key_frame = frame
        if self.key_frame < 0:
            return

        for key in self.key_joints:
            self.key_joints[key].reset()

        self.set_key_frame_infos()

        self.global_axis = self.get_global_axis()
        self.key_joints['a'].local_axis = self.key_joints['a'].get_local_axis(self.global_axis)
        self.key_joints['b'].local_axis = self.key_joints['b'].get_local_axis(self.global_axis)

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
        self.key_joints['a'].diff_angle2 = 0
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
            normal_vector = util.get_normal_vector(vec_a_c_prime, vec_a_t)
        self.global_axis2 = normal_vector
        self.key_joints['a'].local_axis2 = self.key_joints['a'].get_local_axis(normal_vector)

        # Set a's diff angle2 for second step
        tau = 0
        if not np.array_equal(vec_a_c_prime, vec_a_t):
            tau = util.get_degree_between_vectors(vec_a_c_prime, vec_a_t)
        self.key_joints['a'].diff_angle2 = tau

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

    def get_diff_angles(self):
        pos_a = self.key_joints['a'].global_pos
        pos_b = self.key_joints['b'].global_pos
        pos_c = self.key_joints['c'].global_pos
        pos_t = self.key_joints['t'].global_pos

        length_ac_after = np.linalg.norm(pos_t - pos_a)
        length_ac_before = np.linalg.norm(pos_c - pos_a)
        length_ab = np.linalg.norm(pos_b - pos_a)
        length_bc = np.linalg.norm(pos_c - pos_b)
        if not util.is_valid_triangle(length_ab, length_bc, length_ac_after):
            return False

        angle_alpha = util.get_degree_between_triangle(length_ab, length_ac_before, length_bc)
        angle_alpha_after = util.get_degree_between_triangle(length_ab, length_ac_after, length_bc)

        self.key_joints['a'].diff_angle = angle_alpha - angle_alpha_after

        angle_beta = util.get_degree_between_triangle(length_ab, length_bc, length_ac_before)
        angle_beta_after = util.get_degree_between_triangle(length_ab, length_bc, length_ac_after)

        self.key_joints['b'].diff_angle = angle_beta - angle_beta_after

        return True

    def get_global_axis(self):
        for key in self.key_joints:
            self.key_joints[key].set_global_position()

        pos_a = self.key_joints['a'].global_pos
        pos_b = self.key_joints['b'].global_pos
        pos_c = self.key_joints['c'].global_pos

        vec_ab = pos_b - pos_a
        vec_ac = pos_c - pos_a

        global_axis = util.get_normal_vector(vec_ab, vec_ac)
        return global_axis

    def get_joint_key(self, joint_idx):
        for key in self.key_joints:
            if key != 'a' and key != 'b':
                continue
            if joint_idx == self.key_joints[key].idx:
                return key
        return None

    def get_axis(self, key, step: Step):
        if step == Step.FIRST:
            return self.key_joints[key].local_axis
        else:
            return self.key_joints[key].local_axis2

    def get_degree(self, key, step: Step):
        if step == Step.FIRST:
            return self.key_joints[key].diff_angle
        else:
            return self.key_joints[key].diff_angle2


    def save_key_frame_infos(self, joint, frame):
        glPushMatrix()

        # Translate or Rotate joint offset.
        glTranslatef(joint.offset[0], joint.offset[1], joint.offset[2])

        self.joint_idx += 1

        if joint.isEndEffector:
            # Save first end effector. (target of Inverse Kinematics)
            if self.key_joints['c'].idx == -1 and self.key_joints['a'].idx != -1:
                matrix = util.get_current_transformation_matrix()
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
                matrix = util.get_current_transformation_matrix()
                self.key_joints[key].transformation_matrix = matrix
                self.key_joints[key].idx = self.joint_idx

                local_axis = self.key_joints[key].get_local_axis(self.global_axis)

                if key == 'a':
                    local_axis2 = self.key_joints['a'].local_axis2
                    glRotatef(self.key_joints['a'].diff_angle2, local_axis2[0], local_axis2[1], local_axis2[2])

                glRotatef(self.key_joints[key].diff_angle, local_axis[0], local_axis[1], local_axis[2])

        for child in joint.children:
            self.save_key_frame_infos(child, frame)
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

    def render_helper_lines(self):
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

























'''
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

        for key in self.key_joints:
            if key != 'a' and key != 'b':
                continue

            if self.joint_idx == self.key_joints[key].idx:
                local_axis = self.key_joints[key].local_axis
                if render_key_frame:
                    glColor3ub(255, 255, 0)

                if key == 'a':
                    angle = self.key_joints[key].diff_angle2

                    if not render_key_frame:
                        if 85 <= self.rendering_frame <= 100:
                            angle = (angle / 15) * (self.rendering_frame - 85)
                        elif 100 < self.rendering_frame <= 115:
                            angle = (angle / 15) * (115 - self.rendering_frame)
                        else:
                            angle = 0
                    local_axis2 = self.key_joints['a'].local_axis2
                    glRotatef(angle, local_axis2[0], local_axis2[1], local_axis2[2])

                angle = self.key_joints[key].diff_angle
                if not render_key_frame:
                    if 85 <= self.rendering_frame <= 100:
                        angle = (angle / 15) * (self.rendering_frame - 85)
                    elif 100 < self.rendering_frame <= 115:
                        angle = (angle / 15) * (115 - self.rendering_frame)
                    else:
                        angle = 0
                glRotatef(angle, local_axis[0], local_axis[1], local_axis[2])
                '''