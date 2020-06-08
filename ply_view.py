import pyvista as pv
import argparse
import numpy as np
# process cmd line arguments
parser = argparse.ArgumentParser()
parser.add_argument('-f', type=str, metavar='file', required=True, help=' filename.ply, load a .ply file')
parser.add_argument('-o', type=int, metavar='orbit', required=False, default=0, choices=[0,1] , help='[0, 1] creat animated gif, orbiting over the .ply objects')
parser.add_argument('-cp', type=int, metavar='camposition', required=False, default=[(0, -0.5, 5.0), (0.0, 0.0, 3.0), (0, 1, 0)], help=' camera position, default [(0, -0.5, 5.0), (0.0, 0.0, 3.0), (0, 1, 0)]')
args = parser.parse_args()


cpos =  np.array(args.cp,dtype=np.float32)
mesh = pv.read(args.f)
points = mesh.points

pc = pv.PolyData(points)
colors = points[:,-1]

pc["colors"] = colors

plotter = pv.Plotter()
plotter.add_mesh(pc, render_points_as_spheres=True, point_size=2.0)

plotter.set_position(cpos)
# create animated gif
if args.o == 1:
    plotter.show(auto_close=False)

    viewup = [0,2,0]

    path = plotter.generate_orbital_path(factor=4.0, n_points=36, shift=0.2, viewup=viewup)
    plotter.open_gif("out.gif")
    plotter.orbit_on_path(path, write_frames=True, viewup=viewup, step=0.5)

    plotter.close()

# interactive display
if args.o == 0:
    cpos = plotter.show()