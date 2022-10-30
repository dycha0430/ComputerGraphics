import sys

from PyQt5.QtCore import QSize
from PyQt5.QtGui import QIcon, QPixmap

from Renderer import *
from CameraController import CameraController
from Motion import *

# For PyQt5
from PyQt5 import QtCore
from PyQt5 import QtOpenGL 
from PyQt5 import QtWidgets

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

        self.resize(800, 800)
        self.setWindowTitle('BVH Viewer')
        self.setMouseTracking(True)
        self.setAcceptDrops(True)

        self.glWidget = GLWidget(self)
        self.time_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.time_input = QtWidgets.QLineEdit()
        self.play_btn = QtWidgets.QPushButton("", self)
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
        gui_layout = QtWidgets.QVBoxLayout()
        self.setCentralWidget(central_widget)
        gui_layout.addWidget(self.glWidget)

        widgets = QtWidgets.QHBoxLayout()

        self.play_btn.clicked.connect(self.play_btn_clicked)
        self.set_play_btn_icon(True)
        widgets.addWidget(self.play_btn)

        self.time_slider.setRange(0, 1)
        self.time_slider.setStyleSheet('''
            QSlider::groove:horizontal {
            border: 1px solid #bbb;
            background: white;
            height: 10px;
            border-radius: 4px;
            }
            
            QSlider::sub-page:horizontal {
            background: qlineargradient(x1: 0, y1: 0,    x2: 0, y2: 1,
                stop: 0 #66e, stop: 1 #bbf);
            background: qlineargradient(x1: 0, y1: 0.2, x2: 1, y2: 1,
                stop: 0 #bbf, stop: 1 #55f);
            border: 1px solid #777;
            height: 10px;
            border-radius: 4px;
            }
            
            QSlider::add-page:horizontal {
            background: #fff;
            border: 1px solid #777;
            height: 10px;
            border-radius: 4px;
            }
            
            QSlider::handle:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #eee, stop:1 #ccc);
            border: 1px solid #777;
            width: 12px;
            margin-top: -2px;
            margin-bottom: -2px;
            border-radius: 4px;
            }
            
            QSlider::handle:horizontal:hover {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #fff, stop:1 #ddd);
            border: 1px solid #444;
            border-radius: 7px;
            }
            
            QSlider::sub-page:horizontal:disabled {
            background: #bbb;
            border-color: #999;
            }
            
            QSlider::add-page:horizontal:disabled {
            background: #eee;
            border-color: #999;
            }
            
            QSlider::handle:horizontal:disabled {
            background: #eee;
            border: 1px solid #aaa;
            border-radius: 4px;
            }
        ''')

        self.time_slider.valueChanged.connect(lambda val: self.set_frame(val))
        widgets.addWidget(self.time_slider)

        self.time_input.returnPressed.connect(self.set_frame_by_input)
        self.time_input.setFixedSize(30, 30)
        widgets.addWidget(self.time_input)
        central_widget.setLayout(gui_layout)

        gui_layout.addLayout(widgets)

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
        
        if not val.isdigit():
            self.show_alert("Enter decimal number")
            return
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
        if isinstance(focused_widget, QtWidgets.QLineEdit):
            focused_widget.clearFocus()
        if event.key() == QtCore.Qt.Key_Space:
            viewer.bvh_renderer.start_or_stop_animation()
        elif event.key() == QtCore.Qt.Key_V:
            viewer.camera_controller.flip_projection()
        elif event.key() == QtCore.Qt.Key_I:
            frame = QtWidgets.QInputDialog.getInt(self, 'Inverse Kinematics', "Enter a frame #")
            viewer.bvh_renderer.set_key_frame(frame)
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
    
