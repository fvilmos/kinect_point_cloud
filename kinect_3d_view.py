from PyQt5 import Qt,QtCore
from pyvistaqt import QtInteractor

import pyvista as pv
import sys
import numpy as np
import freenect
import cv2
import threading
import argparse

class MainWindow(Qt.QMainWindow):

    def __init__(self, parent=None, show=True, winsize=(640,480), campos=[(0.0,0.0,5.0),(0.0,0.0,2.0),(0.0,1.0,0.0)]):
        '''
        Basic class to show 3D projection
        :param parent: inherit from Qt.Mainvindow
        :param show:
        '''
        Qt.QMainWindow.__init__(self, parent)

        # initial size
        self.setMinimumSize(winsize[0],winsize[1])

        # create the frame
        self.frame = Qt.QFrame()
        vlayout = Qt.QVBoxLayout()
        vlayout.setContentsMargins(0,0,0,0)


        # add a menu
        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu('File')
        exitButton = Qt.QAction('Exit', self)
        exitButton.setShortcut('Ctrl+Q')
        exitButton.triggered.connect(self.close)
        fileMenu.addAction(exitButton)

        self.add_save_action = Qt.QAction('Save out.ply', self)
        self.add_save_action.triggered.connect(self.save_ply)
        fileMenu.addAction(self.add_save_action)


        # add the pyvista interactor object
        self.plotter = QtInteractor(self.frame,auto_update=0.0,point_smoothing=False)
        self.plotter.show_axes_all()

        # give depth perspective for the points
        self.plotter.enable_eye_dome_lighting()

        # set initial camera pozition
        self.cpos = np.array(campos)
        self.plotter.camera_position = self.cpos

        vlayout.addWidget(self.plotter.interactor)

        self.frame.setLayout(vlayout)
        self.setCentralWidget(self.frame)

        # set up timer for cyclic update
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.PlotUpdate)
        timer.start(1)

        if show:
            self.show()

    def PlotUpdate(self):
        '''
        Plot the point cloud
        :return:
        '''

        # holds the data
        global kpoints

        # clean the view
        self.plotter.clear()
        point_cloud = pv.PolyData(kpoints)

        # add data to be visualized
        self.plotter.add_mesh(point_cloud, point_size=1.0, lighting=True, render_points_as_spheres=True)

    def save_ply(self):
        '''
        Save curent layout to an out.ply file
        :return:
        '''
        global kpoints
        ply_header = '''ply
        format ascii 1.0
        element vertex %(vert_num)d
        property float x
        property float y
        property float z
        property uchar red
        property uchar green
        property uchar blue
        end_header
        '''

        verts = kpoints.reshape(-1, 3)
        colors = np.ones(verts.shape)
        verts = np.hstack([verts, colors])
        with open('out.ply', 'wb') as f:
            f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
            np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

def worker():
    '''
    Get the data from Kinect and prepare for plotting
    :return:
    '''
    global kpoints, kcolors
    while True:
        # get data from kinect
        disp = np.array(freenect.sync_get_depth()[0], dtype=np.float32)
        data = freenect.sync_get_video()[0]

        frame = data[:, :, ::-1]
        h, w = frame.shape[:2]

        # estimate focal length
        f = 4.0 * w

        # prepare reprojection in 3D
        Q = np.float32([[1, 0, 0, -0.5 * w],
                        [0, -1, 0, 0.5 * h],
                        [0, 0, 0, f],
                        [0, 0, 1, 0]])

        points = cv2.reprojectImageTo3D(disp, Q)

        # get only points of interest
        mask = (disp > disp.min()) & (disp < disp.max())
        kpoints = points[mask]


if __name__ == '__main__':

    # process cmd line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-camid', type=int, metavar='camera',required=True, default=None, choices=[None, 0, 1, 2, 3], help='usb port on which kinect is connected, camid=[None,0,1,2,3], default=None')

    args = parser.parse_args()

    if args.camid is None:
        print ("No kinect ID")
        sys.exit(1)
    else:
        KINECT_ID = args.camid

    # globals
    kpoints = np.empty((1, 3), np.float32)
    disp = np.empty((1, 3), np.uint16)

    # kinect init
    ctx = freenect.init()
    mdev = freenect.open_device(freenect.init(), KINECT_ID)
    freenect.set_depth_mode(mdev, freenect.RESOLUTION_MEDIUM, freenect.DEPTH_REGISTERED)


    # start serving the kinect frames
    th = threading.Thread(target=worker, daemon=True)
    th.start()

    # crate application window for plotting
    app = Qt.QApplication(sys.argv)
    window = MainWindow()
    app.exec_()

    # free the kinect
    freenect.sync_stop()
    freenect.Kill()

    sys.exit()