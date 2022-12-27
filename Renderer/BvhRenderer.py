from Renderer.Renderer import Renderer
from Renderer.ParticleRenderer import ParticleRenderer
from LimbIK.LimbIK import LimbIK, Step
import Util
from OpenGL.GL import *
import numpy as np


class BvhRenderer(Renderer):
    def __init__(self, bvh_motion):
        self.bvh_motion = bvh_motion
        self.cur_frame = 0
        self.rendering_frame = 0
        self.animating_mode = False

        self.limb_ik = LimbIK(bvh_motion)
        self.warping_interval = 0
        self.posture_idx = 0
        self.channels_idx = 0
        self.joint_idx = -1

        self.connected_particle_renderer: ParticleRenderer = None
        self.selected_joint_name = ""

    def set_connected_particle_renderer(self, particle_renderer):
        self.connected_particle_renderer = particle_renderer

    def get_bvh(self):
        return self.bvh_motion

    def set_bvh(self, bvh_motion):
        self.bvh_motion = bvh_motion
        self.cur_frame = 0
        self.animating_mode = False
        self.limb_ik.set_bvh(bvh_motion)
        self.warping_interval = 0

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

    def set_key_frame(self, key_frame):
        if self.bvh_motion is None:
            return

        if self.get_frame_num() <= key_frame:
            key_frame = self.get_frame_num() - 1

        self.limb_ik.set_key_frame(key_frame)

    # Move target end effector by keyboard input WASD+QE
    def move_end_effector(self, offset):
        self.limb_ik.move_end_effector(offset)

    def set_enable_warping(self, interval):
        if interval < 0:
            interval = 0
        self.warping_interval = interval

    def get_warping_angle(self, degree):
        key_frame = self.limb_ik.get_key_frame()
        start_frame = max(key_frame - self.warping_interval, 0)
        end_frame = min(key_frame + self.warping_interval, self.get_frame_num() - 1)

        if start_frame <= self.rendering_frame <= key_frame:
            if key_frame == start_frame: return degree
            degree = (degree / (key_frame - start_frame)) * (self.rendering_frame - start_frame)
        elif key_frame < self.rendering_frame <= end_frame:
            if key_frame == end_frame: return degree
            degree = (degree / (end_frame - key_frame)) * (end_frame - self.rendering_frame)
        else:
            degree = 0

        return degree

    def draw_frame_recursively(self, joint, render_key_frame, draw_mode=True):
        glPushMatrix()
        if joint.name == self.selected_joint_name:
            glColor3ub(0, 255, 0)

        # Draw link.
        if draw_mode:
            glBegin(GL_LINES)
            glVertex3fv(np.array([0, 0, 0]))
            glVertex3fv(np.array([joint.offset[0], joint.offset[1], joint.offset[2]]))
            glEnd()
        glColor3ub(255, 255, 255)

        # Translate or Rotate joint offset.
        glTranslatef(joint.offset[0], joint.offset[1], joint.offset[2])
        if render_key_frame:
            glColor3ub(255, 0, 0)

        self.joint_idx += 1

        if not draw_mode:
            if joint.name == self.selected_joint_name:
                transformation_matrix = Util.get_current_transformation_matrix()
                global_position = transformation_matrix.T @ np.array([0, 0, 0, 1]).T
                global_position = global_position[:-1]
                self.connected_particle_renderer.set_bvh_joint_pos(global_position)

        if joint.isEndEffector:
            glPopMatrix()
            return

        self.transform_by_channel(self.rendering_frame)

        if render_key_frame or (self.limb_ik.get_key_frame() >= 0 and self.warping_interval > 0):
            key = self.limb_ik.get_joint_key(self.joint_idx)
            if key is not None:
                if render_key_frame:
                    glColor3ub(255, 255, 0)
                local_axis2 = self.limb_ik.get_local_axis(key, Step.SECOND)
                diff_angle2 = self.limb_ik.get_degree(key, Step.SECOND)
                diff_angle2 = self.get_warping_angle(diff_angle2)
                glRotatef(diff_angle2, local_axis2[0], local_axis2[1], local_axis2[2])

                local_axis = self.limb_ik.get_local_axis(key, Step.FIRST)
                diff_angle = self.limb_ik.get_degree(key, Step.FIRST)
                diff_angle = self.get_warping_angle(diff_angle)
                glRotatef(diff_angle, local_axis[0], local_axis[1], local_axis[2])

        for child in joint.children:
            self.draw_frame_recursively(child, render_key_frame, draw_mode)
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

    def reset_indexes(self):
        self.posture_idx = 0
        self.channels_idx = 0
        self.joint_idx = -1

    def render_key_frame(self, root):
        self.reset_indexes()
        glColor3ub(255, 0, 0)
        self.rendering_frame = self.limb_ik.get_key_frame()
        self.draw_frame_recursively(root, True)
        glColor3ub(255, 255, 255)

    def render(self):
        if self.bvh_motion is None:
            return
        self.rendering_frame = self.cur_frame
        self.reset_indexes()
        root = self.bvh_motion.get_root()

        self.draw_frame_recursively(root, False)

        if self.connected_particle_renderer.change_motion_link_mode:
            glPushMatrix()
            glLoadIdentity()
            self.reset_indexes()
            self.draw_frame_recursively(root, False, draw_mode=False)
            glPopMatrix()

        if self.limb_ik.get_key_frame() >= 0:
            self.render_key_frame(root)

