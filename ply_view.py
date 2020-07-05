import pyvista as pv
import argparse
import numpy as np
# process cmd line arguments
parser = argparse.ArgumentParser()
parser.add_argument('-f', type=str, metavar='file', required=True, help=' filename.ply, load a .ply file')
parser.add_argument('-o', type=int, metavar='orbit', required=False, default=0, choices=[0,1] , help='[0, 1] creat animated gif, orbiting over the .ply objects')
parser.add_argument('-cp', type=int, metavar='camposition', required=False, default=[(0.0, 0.0, -5.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)], help=' camera position, default [(0, -0.5, 5.0), (0.0, 0.0, 3.0), (0, 1, 0)]')
parser.add_argument('-rgb', type=str, metavar='rgb', required=False, default='c',choices=['c','d'], help=' [c,d] Use RGB colors from ply file or use depth info to colorize')
args = parser.parse_args()


cpos =  np.array(args.cp,dtype=np.float32)
mesh = pv.read(args.f)
points = mesh.points

pc = pv.PolyData(points)
plotter = pv.Plotter()
plotter.set_position(cpos)
plotter.set_viewup([0,-1,0])

#use depth info
if args.rgb == 'd':
    colors = points[:,-1]
    pc["colors"] = colors
    plotter.add_mesh(pc, render_points_as_spheres=False, point_size=2.0)

#use rgb colors
if args.rgb == 'c':
    cl = mesh['RGB']
    plotter.add_mesh(pc, render_points_as_spheres=False, point_size=2.0,rgb=True, scalars=cl)


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