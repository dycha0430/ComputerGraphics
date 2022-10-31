import sys

from PyQt5.QtCore import QSize
from PyQt5.QtGui import QIcon, QPixmap, QIntValidator

import StyleSheet
from Renderer import *
from CameraController import CameraController
from Motion import *
from StyleSheet import *

# For PyQt5
from PyQt5 import QtCore, QtOpenGL, QtWidgets

###########################################################
# Global renderer
viewer = None
win = None

class Viewer:
    def __init__(self, bvh_motion):
        self.renderers = []
        self.bvh_renderer = BvhRenderer(bvh_motion)
        self.camera_controller = CameraController()
        self.renderers.append(self.bvh_renderer)
        self.renderers.append(BackgroundRenderer())
        
    def render(self):
        self.camera_controller.init_viewport()

        for renderer in self.renderers:
            renderer.render()

##################################################################
            
class GLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        self.parent = parent
        QtOpenGL.QGLWidget.__init__(self, parent)

    def paintGL(self):
        global viewer
        viewer.render()

    def resizeGL(self, width, height):
        global viewer
        viewer.camera_controller.set_viewport_size(width, height)
    
class MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        QtWidgets.QMainWindow.__init__(self) # call the init for the parent class

        self.resize(1260, 900)
        self.setWindowTitle('BVH Viewer')
        self.setMouseTracking(True)
        self.setAcceptDrops(True)

        self.glWidget = GLWidget(self)
        self.time_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.time_input = QtWidgets.QLineEdit()
        self.play_btn = QtWidgets.QPushButton("", self)
        self.key_frame_input = QtWidgets.QLineEdit()
        self.warping_input = QtWidgets.QLineEdit()
        self.key_frame_check_box = QtWidgets.QCheckBox()
        self.warping_check_box = QtWidgets.QCheckBox()

        self.onlyInt = QIntValidator()
        self.time_input.setValidator(self.onlyInt)
        self.key_frame_input.setValidator(self.onlyInt)
        self.warping_input.setValidator(self.onlyInt)
        self.initGUI()
        
        timer = QtCore.QTimer(self)
        timer.setInterval(10) # period, in milliseconds
        timer.timeout.connect(self.glWidget.updateGL)
        timer.start()

        self.frame_timer = QtCore.QTimer(self)
        self.frame_timer.timeout.connect(self.update_frame)

        self.button_left_pressed = False
        self.button_right_pressed = False

    def initGUI(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        whole_layout = QtWidgets.QHBoxLayout()

        gui_layout = QtWidgets.QVBoxLayout()
        gui_layout.addWidget(self.glWidget)
        # --------- Bottom widgets -----------
        widgets = QtWidgets.QHBoxLayout()
        self.play_btn.clicked.connect(self.play_btn_clicked)
        self.set_play_btn_icon(True)
        widgets.addWidget(self.play_btn)

        self.time_slider.setRange(0, 1)
        self.time_slider.setStyleSheet(StyleSheet.TIME_SLIDER_STYLE)

        self.time_slider.valueChanged.connect(lambda val: self.set_frame(val))
        widgets.addWidget(self.time_slider)

        self.time_input.returnPressed.connect(self.set_frame_by_input)
        self.time_input.setFixedSize(30, 30)
        widgets.addWidget(self.time_input)
        # ------------------------------------
        gui_layout.addLayout(widgets)
        # ---------- Right widgets -----------
        widgets2 = QtWidgets.QVBoxLayout()
        key_frame_widget = QtWidgets.QHBoxLayout()
        self.key_frame_check_box.stateChanged.connect(self.enable_key_frame)
        key_frame_widget.addWidget(self.key_frame_check_box)
        self.key_frame_input.returnPressed.connect(self.set_key_frame)
        self.key_frame_input.setPlaceholderText("Key frame")
        key_frame_widget.addWidget(self.key_frame_input)

        warping_widget = QtWidgets.QHBoxLayout()
        self.warping_check_box.stateChanged.connect(self.enable_warping)
        warping_widget.addWidget(self.warping_check_box)
        self.warping_input.returnPressed.connect(self.set_warping)
        self.warping_input.setPlaceholderText("Warping interval")
        warping_widget.addWidget(self.warping_input)

        widgets2.addLayout(key_frame_widget)
        widgets2.addLayout(warping_widget)
        widgets2.addStretch()
        # ------------------------------------
        whole_layout.addLayout(gui_layout, stretch = 4)
        whole_layout.addLayout(widgets2)

        central_widget.setLayout(whole_layout)

    def set_key_frame(self):
        if not self.key_frame_check_box.isChecked():
            self.key_frame_input.clear()
            return
        key_frame = int(self.key_frame_input.text())
        viewer.bvh_renderer.set_key_frame(key_frame)

    def enable_key_frame(self, state):
        if state == QtCore.Qt.Checked:
            viewer.bvh_renderer.set_key_frame(0)
        else:
            self.key_frame_input.clear()
            viewer.bvh_renderer.set_key_frame(-1)

    def set_warping(self):
        if not self.warping_check_box.isChecked():
            self.warping_input.clear()
            return
        interval = int(self.warping_input.text())
        viewer.bvh_renderer.set_enable_warping(interval)

    def enable_warping(self, state):
        if state == QtCore.Qt.Checked:
            viewer.bvh_renderer.set_enable_warping(0)
        else:
            self.warping_input.clear()
            viewer.bvh_renderer.set_enable_warping(-1)

    def set_play_btn_icon(self, is_play):
        pixmap = None
        if is_play:
            pixmap = QPixmap('./play_btn.jpg')
        else:
            pixmap = QPixmap('./pause_btn.png')
        icon = QIcon()
        icon.addPixmap(pixmap)
        self.play_btn.setIcon(icon)
        self.play_btn.setIconSize(QSize(40, 40))

    def show_alert(self, message):
        QtWidgets.QMessageBox.question(self, 'Error', message, QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.NoButton)

    def set_frame_by_input(self):
        global viewer
        val = self.time_input.text()
        val = int(val)

        if val < 0:
            self.show_alert("Enter positive number")
            return
        elif val >= viewer.bvh_renderer.get_frame_num():
            self.show_alert("Frame number cannot exceed bvh file's frame number")
            return
        self.set_frame(val)
        
    def play_btn_clicked(self):
        global viewer
        viewer.bvh_renderer.start_or_stop_animation()

        self.set_play_btn_icon(True)

        if viewer.bvh_renderer.animating_mode:
            self.set_play_btn_icon(False)
        
    def set_frame(self, frame):
        global viewer
        viewer.bvh_renderer.cur_frame = frame
        self.time_input.setText(str(frame))

    def keyPressEvent(self, event):
        global viewer

        ik_factor = 0.01

        focused_widget = QtWidgets.QApplication.focusWidget()
        if isinstance(focused_widget, QtWidgets.QLineEdit) or isinstance(focused_widget, QtWidgets.QCheckBox):
            focused_widget.clearFocus()
        if event.key() == QtCore.Qt.Key_Space:
            viewer.bvh_renderer.start_or_stop_animation()
        elif event.key() == QtCore.Qt.Key_V:
            viewer.camera_controller.flip_projection()
        elif event.key() == QtCore.Qt.Key_W: # Up
            viewer.bvh_renderer.move_end_effector([0, ik_factor, 0])
        elif event.key() == QtCore.Qt.Key_S: # Down
            viewer.bvh_renderer.move_end_effector([0, -ik_factor, 0])
        elif event.key() == QtCore.Qt.Key_A: # Left
            viewer.bvh_renderer.move_end_effector([-ik_factor, 0, 0])
        elif event.key() == QtCore.Qt.Key_D: # Right
            viewer.bvh_renderer.move_end_effector([ik_factor, 0, 0])
        elif event.key() == QtCore.Qt.Key_Q: # Back
            viewer.bvh_renderer.move_end_effector([0, 0, -ik_factor])
        elif event.key() == QtCore.Qt.Key_E: # Front
            viewer.bvh_renderer.move_end_effector([0, 0, ik_factor])

    def wheelEvent(self, event):
        y_offset = 0.005 * event.angleDelta().y()
        global viewer
        viewer.camera_controller.zoom(y_offset)
        

    def mousePressEvent(self, event):
        global viewer
        xpos = event.x()
        ypos = event.y()
        if event.buttons() & QtCore.Qt.RightButton:
            self.button_right_pressed = True
            viewer.camera_controller.set_cur_pos(xpos, ypos)
        elif event.buttons() & QtCore.Qt.LeftButton:
            self.button_left_pressed = True
            viewer.camera_controller.set_cur_pos(xpos, ypos)

    def mouseReleaseEvent(self, event):
        self.button_right_pressed = False
        self.button_left_pressed = False
    
    def mouseMoveEvent(self, event):
        global viewer
        xpos = event.x()
        ypos = event.y()
        if self.button_left_pressed:
            viewer.camera_controller.change_orbit(xpos, ypos)
            viewer.camera_controller.set_cur_pos(xpos, ypos)
        elif self.button_right_pressed:
            viewer.camera_controller.change_panning(xpos, ypos)
            viewer.camera_controller.set_cur_pos(xpos, ypos)
    
    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()
            
    def dropEvent(self, event):
        global viewer

        file_name = [u.toLocalFile() for u in event.mimeData().urls()][0]
        viewer.bvh_renderer.set_bvh(BvhMotion(file_name))
        
        self.time_slider.setRange(0, viewer.bvh_renderer.get_frame_num()-1)
        self.frame_timer.stop()
        self.frame_timer.setInterval(int(viewer.bvh_renderer.get_bvh().frame_time * 1000))
        self.frame_timer.start()

    def update_frame(self):
        global viewer
        self.time_slider.setValue(viewer.bvh_renderer.cur_frame)
        if viewer.bvh_renderer.animating_mode:
            self.time_input.setText(str(viewer.bvh_renderer.cur_frame))
        viewer.bvh_renderer.set_next_frame()
        
##################################################################
def main():
    global viewer, win
    bvh_motion = None
    viewer = Viewer(bvh_motion)

    app = QtWidgets.QApplication(sys.argv)

    win = MainWindow()
    win.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
    
