import sys
from MotionViewer import *

def main():
    bvh_motion = None
    viewer = Viewer(bvh_motion)

    app = QtWidgets.QApplication(sys.argv)

    win = MainWindow(viewer)
    win.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
    