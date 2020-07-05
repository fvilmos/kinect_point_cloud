from PyQt5 import Qt,QtCore
from pyvistaqt import QtInteractor

import pyvista as pv
import sys
import numpy as np
import freenect
import cv2
import threading
import argparse

class KinectHandler():
    '''
    Kinect handler
    '''

    import numpy as np
    import cv2

    # depth cam paramaters
    DepthCamParams = {
        "fx": 5.8818670481438744e+02,
        "fy": 5.8724220649505514e+02,
        "cx": 3.1076280589210484e+02,
        "cy": 2.2887144980135292e+02,
        "k1": 0.0,
        "k2": 0.0,
        "p1": 0.0,
        "p2": 0.0,
        "k3": 0.0,
        "a": -0.0030711,
        "b": 3.3309495,
    }

    # RGB cam parameters
    RGBCamParams = {
        "fx": 5.2161910696979987e+02,
        "fy": 5.2132946256749767e+02,
        "cx": 3.1755491910920682e+02,
        "cy": 2.5921654718027673e+02,
        "k1": 0.0,
        "k2": 0.0,
        "p1": 0.0,
        "p2": 0.0,
        "k3": 0.0,
        "rot": np.array([[9.9996518012567637e-01, 2.6765126468950343e-03, -7.9041012313000904e-03],
                         [-2.7409311281316700e-03, 9.9996302803027592e-01, -8.1504520778013286e-03],
                         [7.8819942130445332e-03, 8.1718328771890631e-03, 9.9993554558014031e-01]]),
        "trans": np.array([[-2.5558943178152542e-02, 1.0109636268061706e-04, 2.0318321729487039e-03]])
    }

    def __init__(self, kinectid=1):
        self.kinectid = kinectid
        self.rgb = None
        self.depth = None
        self.keep_running = True

        # init Kenect
        self.context = freenect.init()
        self.device = freenect.open_device(self.context, self.kinectid)
        self.init = freenect.set_depth_mode(self.device, freenect.RESOLUTION_MEDIUM, freenect.DEPTH_REGISTERED)

    def get_depth_rgb_synch(self):
        '''
        Get synchronously depth and RGB frames
        :return:
        '''
        depth = freenect.sync_get_depth()[0]
        rgb = freenect.sync_get_video()[0]

        return depth, rgb

    def depth_cam_mat(self):
        '''
        Returns camera matrix, including the transformation values of depth to meters
        :return: camera (intrisec) matrix
        '''

        mat = np.array([[1 / self.DepthCamParams['fx'], 0, 0, -self.DepthCamParams['cx'] / self.DepthCamParams['fx']],
                        [0, 1 / self.DepthCamParams['fy'], 0, -self.DepthCamParams['cy'] / self.DepthCamParams['fy']],
                        [0, 0, 0, 1],
                        [0, 0, self.DepthCamParams['a'], self.DepthCamParams['b']]])

        return mat

    def get_registred_depth_rgb(self):
        '''
        Returns the registred pointclaud and image with transforming the cameras position in world coordinate system
        :return: registred point cloud and image
        '''
        depth, img = self.get_depth_rgb_synch()

        h, w = img.shape[:2]

        depth = np.array(depth, dtype=np.float32)

        # project points to 3D space
        points = cv2.reprojectImageTo3D(depth, self.depth_cam_mat())

        # transform coordinates to RGB camera coordinates
        points = np.dot(points, self.RGBCamParams['rot'].T)
        points = np.add(points, self.RGBCamParams['trans'])

        # handle invalid values
        points[depth >= depth.max()] = 0.0

        points = points.reshape(-1, 640, 3)

        # project 3D points back to image plain
        x = np.array((points[:, :, 0] * (self.RGBCamParams['fx'] / points[:, :, 2]) + self.RGBCamParams['cx']),
                     dtype=np.int).clip(0, w - 1)
        y = np.array((points[:, :, 1] * (self.RGBCamParams['fy'] / points[:, :, 2]) + self.RGBCamParams['cy']),
                     dtype=np.int).clip(0, h - 1)

        return points, img[y, x]


class MainWindow(Qt.QMainWindow):

    def __init__(self, parent=None, show=True, winsize=(640,480), campos=[(0.0,0.0,-5.0),(0.0,0.0,-2.0),(0.0,-1.0,0.0)],kinectid=0):
        '''
        Basic class to show 3D projection
        :param parent: inherit from Qt.Mainvindow
        :param show:
        '''
        Qt.QMainWindow.__init__(self, parent)

        self.pc_points = None
        self.pc_colors = None

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
        #self.plotter.disable_eye_dome_lighting()
        self.plotter.enable_depth_peeling()
        self.plotter.disable_parallel_projection()

        # set initial camera pozition
        self.cpos = np.array(campos)
        self.plotter.camera_position = self.cpos

        vlayout.addWidget(self.plotter.interactor)

        self.frame.setLayout(vlayout)
        self.setCentralWidget(self.frame)

        # start serving the kinect frames
        self.kin = KinectHandler(kinectid)

        # set up timer for cyclic update
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.PlotUpdate)
        timer.start(80)

        if show:
            self.show()

    def PlotUpdate(self):
        '''
        Plot the point cloud
        :return:
        '''

        self.pc_points, self.pc_colors = self.kin.get_registred_depth_rgb()

        if self.pc_colors is not None and self.pc_colors is not None:
            # clean the view
            self.plotter.clear()
            point_cloud = pv.PolyData(self.pc_points)

            if args.rgb == 'c':
                point_cloud["colors"] = self.pc_colors.reshape(-1,3)
            else:
                dc = cv2.normalize(self.pc_points[:, -1], None, 0.0, 1.0, cv2.NORM_MINMAX)
                point_cloud["colors"] = dc

            # add data to be visualized
            self.plotter.add_mesh(point_cloud, point_size=2.0, lighting=False, render_points_as_spheres=False, color=True, interpolate_before_map=False, show_scalar_bar=False)

    def save_ply(self):
        '''
        Save curent layout to an out.ply file
        :return:
        '''

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

        verts = self.pc_points.reshape(-1, 3)
        colors = np.ones(verts.shape)
        verts = np.hstack([verts, self.pc_colors.reshape(-1,3)])
        
        with open('out.ply', 'wb') as f:
            f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
            np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

if __name__ == '__main__':

    # process cmd line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-camid', type=int, metavar='camera',required=True, default=None, choices=[None, 0, 1, 2, 3], help='usb port on which kinect is connected, camid=[None,0,1,2,3], default=None')
    parser.add_argument('-rgb', type=str, metavar='rgb', required=False, default='c', choices=['c', 'd'], help=' [c,d] Use RGB colors from ply file or use depth info to colorize')

    args = parser.parse_args()

    if args.camid is None:
        print ("No kinect ID")
        sys.exit(1)
    else:
        KINECT_ID = args.camid

    # crate application window for plotting
    app = Qt.QApplication(sys.argv)
    window = MainWindow(kinectid=KINECT_ID)
    app.exec_()

    sys.exit()
