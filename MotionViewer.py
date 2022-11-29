import sys
import StyleSheet
from Renderer import *
from ParticleRenderer import *
from CameraController import CameraController
from Motion import *

# For PyQt5
from PyQt5 import QtCore, QtOpenGL, QtWidgets
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QIcon, QPixmap, QIntValidator

###########################################################
# Global renderer
viewer = None
win = None

class Viewer:
    def __init__(self, bvh_motion):
        self.renderers = []
        self.bvh_renderer = BvhRenderer(bvh_motion)
        self.particle_renderer = ParticleRenderer()
        self.camera_controller = CameraController()
        self.renderers.append(self.bvh_renderer)
        self.renderers.append(self.particle_renderer)
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

        self.kd_input = QtWidgets.QLineEdit()
        self.ks_input = QtWidgets.QLineEdit()
        self.integration_method_combo_box = QtWidgets.QComboBox()

        self.onlyInt = QIntValidator()
        self.time_input.setValidator(self.onlyInt)
        self.key_frame_input.setValidator(self.onlyInt)
        self.warping_input.setValidator(self.onlyInt)
        self.kd_input.setValidator(self.onlyInt)
        self.ks_input.setValidator(self.onlyInt)
        self.rendering_square_btn = QtWidgets.QPushButton("", self)
        self.rendering_particle_system_btn = QtWidgets.QPushButton("", self)

        self.particle_list_title = QtWidgets.QLineEdit()
        self.particle_list_title.setText("Particles")
        self.particle_list_title.setReadOnly(True)
        self.particle_list_title.setStyleSheet("* { background-color: rgba(0, 0, 0, 0); border: none;}")
        self.particle_list = QtWidgets.QListWidget()
        self.selected_particle_list_title = QtWidgets.QLineEdit()
        self.selected_particle_list_title.setText("Selected Particles")
        self.selected_particle_list_title.setReadOnly(True)
        self.selected_particle_list_title.setStyleSheet("* { background-color: rgba(0, 0, 0, 0); border: none;}")
        self.selected_particle_list = QtWidgets.QListWidget()
        self.add_spring_btn = QtWidgets.QPushButton("Add Spring", self)
        self.move_mode_check_box = QtWidgets.QCheckBox("Move particle mode")

        self.initGUI()
        
        timer = QtCore.QTimer(self)
        timer.setInterval(33) # period, in milliseconds
        timer.timeout.connect(self.glWidget.updateGL)
        timer.start()

        self.frame_timer = QtCore.QTimer(self)
        self.frame_timer.timeout.connect(self.update_frame)

        self.simulation_time_stamp = 1/1000
        simulation_timer = QtCore.QTimer(self)
        simulation_timer.setInterval(int(self.simulation_time_stamp * 1000)) # 1/1000 s
        simulation_timer.timeout.connect(self.simulate_particle)
        simulation_timer.start()

        self.button_left_pressed = False
        self.button_right_pressed = False

    def simulate_particle(self):
        global viewer
        viewer.particle_renderer.simulate_particle(self.simulation_time_stamp)

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

        self.kd_input.returnPressed.connect(self.set_kd)
        self.kd_input.setPlaceholderText("Set spring kd")
        self.ks_input.returnPressed.connect(self.set_ks)
        self.ks_input.setPlaceholderText("Set spring ks")

        self.integration_method_combo_box.addItem("Semi-implicit Euler")
        self.integration_method_combo_box.addItem("Euler")
        self.integration_method_combo_box.currentIndexChanged.connect(self.change_integration_method)

        self.rendering_square_btn.clicked.connect(self.rendering_square_btn_clicked)
        self.set_rendering_btn_icon(False)
        self.rendering_square_btn.setCheckable(True)

        self.rendering_particle_system_btn.clicked.connect(self.rendering_particle_system_btn_clicked)
        self.set_rendering_particle_system_btn_icon(False)

        self.particle_list.itemClicked.connect(self.particle_clicked)
        self.particle_list.itemDoubleClicked.connect(self.particle_double_clicked)

        self.selected_particle_list.itemDoubleClicked.connect(self.selected_particle_double_clicked)
        self.add_spring_btn.clicked.connect(self.add_spring)

        self.move_mode_check_box.stateChanged.connect(self.change_move_mode)

        widgets2.addLayout(key_frame_widget)
        widgets2.addLayout(warping_widget)
        widgets2.addWidget(self.kd_input)
        widgets2.addWidget(self.ks_input)
        widgets2.addWidget(self.rendering_square_btn)
        widgets2.addWidget(self.integration_method_combo_box)
        widgets2.addWidget(self.rendering_particle_system_btn)

        widgets2.addWidget(self.particle_list_title)
        widgets2.addWidget(self.particle_list)
        widgets2.addWidget(self.selected_particle_list_title)
        widgets2.addWidget(self.selected_particle_list)
        widgets2.addWidget(self.add_spring_btn)
        widgets2.addWidget(self.move_mode_check_box)
        widgets2.addStretch()
        # ------------------------------------
        whole_layout.addLayout(gui_layout, stretch = 4)
        whole_layout.addLayout(widgets2)

        central_widget.setLayout(whole_layout)

    def change_move_mode(self):
        if self.selected_particle_list.count() > 1:
            return
        elif self.selected_particle_list.count() == 1:
            viewer.particle_renderer.change_move_mode(select_particle=True)
        else:
            viewer.particle_renderer.change_move_mode()

    def add_spring(self):
        if self.selected_particle_list.count() != 2:
            return
        viewer.particle_renderer.add_spring()
        self.selected_particle_list.clear()
        viewer.particle_renderer.clear_selected_particle()

    def selected_particle_double_clicked(self):
        index = self.selected_particle_list.currentRow()
        viewer.particle_renderer.remove_selected_particle(index)
        self.selected_particle_list.takeItem(index)

    def particle_clicked(self):
        index = self.particle_list.currentRow()
        viewer.particle_renderer.click_particle(index)

    def particle_double_clicked(self):
        if self.selected_particle_list.count() >= 2:
            return
        index = self.particle_list.currentRow()
        if viewer.particle_renderer.contains_particle(index):
            return
        viewer.particle_renderer.select_particle(index)
        self.selected_particle_list.addItem("Particle " + str(index))

    def rendering_particle_system_btn_clicked(self):
        viewer.particle_renderer.set_render_particle_system()
        if viewer.particle_renderer.rendering_particle_system:
            self.set_rendering_particle_system_btn_icon(True)
        else:
            self.set_rendering_particle_system_btn_icon(False)

    def set_rendering_particle_system_btn_icon(self, is_rendering):
        if is_rendering:
            self.rendering_particle_system_btn.setText("Stop particle simulation")
        else:
            self.rendering_particle_system_btn.setText("Start particle simulation")

    def change_integration_method(self):
        index = self.integration_method_combo_box.currentIndex()
        viewer.particle_renderer.change_integration_method(index)

    def rendering_square_btn_clicked(self):
        viewer.particle_renderer.set_rendering_square()
        if self.rendering_square_btn.isChecked():
            self.rendering_square_btn.setCheckable(False)
            self.set_rendering_btn_icon(True)
        else:
            self.rendering_square_btn.setCheckable(True)
            self.set_rendering_btn_icon(False)

    def set_rendering_btn_icon(self, is_rendering):
        if is_rendering:
            self.rendering_square_btn.setText("Stop square particle simulation")
        else:
            self.rendering_square_btn.setText("Start square particle simulation")
    def set_kd(self):
        kd = int(self.kd_input.text())
        viewer.particle_renderer.set_spring_kd(kd)

    def set_ks(self):
        ks = int(self.ks_input.text())
        viewer.particle_renderer.set_spring_ks(ks)

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
        pointer_factor = 0.05

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
        elif event.key() == QtCore.Qt.Key_I:
            viewer.particle_renderer.move_pointer([0, pointer_factor, 0])
        elif event.key() == QtCore.Qt.Key_K:  # Down
            viewer.particle_renderer.move_pointer([0, -pointer_factor, 0])
        elif event.key() == QtCore.Qt.Key_J:  # Left
            viewer.particle_renderer.move_pointer([-pointer_factor, 0, 0])
        elif event.key() == QtCore.Qt.Key_L:  # Right
            viewer.particle_renderer.move_pointer([pointer_factor, 0, 0])
        elif event.key() == QtCore.Qt.Key_U:  # Back
            viewer.particle_renderer.move_pointer([0, 0, -pointer_factor])
        elif event.key() == QtCore.Qt.Key_O:  # Front
            viewer.particle_renderer.move_pointer([0, 0, pointer_factor])
        elif event.key() == QtCore.Qt.Key_Shift:
            viewer.particle_renderer.add_particle()
            self.particle_list.addItem("Particle " + str(self.particle_list.count()))

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
    
