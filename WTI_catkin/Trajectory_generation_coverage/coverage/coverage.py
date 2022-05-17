"""
Adapted from trimesh example raytrace.py
----------------
Install `pyembree` for a speedup (600k+ rays per second)
"""
from scipy.spatial.transform.rotation import Rotation
from stl import mesh

from Visualizer.CoverageDisplay3D import CoverageDisplay3D
from Visualizer.Display3D import Display3D
from objects.mesh_base import Wireframe
import numpy as np

from polynomial_fitting import plot_axis
from tranformation.camera import CameraInfo
from tranformation.transform import TRotation, Transform


def readfile_multiple_values(filename, split_char=" "):
    file1 = open(filename, 'r')
    Lines = file1.readlines()
    offset = np.array([-68, 32, 70])
    rotaion_offset = 180
    count = 0
    # Strips the newline character
    lines = []
    poses = []
    yaws = []
    x_refs = []
    ts = []
    for line in Lines:
        line = line.strip()
        l_values = line.split(split_char)
        poses.append(np.array(l_values[4:7]).astype(float) + offset)
        yaw = np.array([l_values[18]]).astype(float) + rotaion_offset
        yaw = -yaw
        yaws.append(yaw)
        x_refs.append(np.array(l_values[1:4]).astype(float) + offset)
        ts.append(np.array([l_values[0]]).astype(float))
    lines = [np.array(ts), np.array(poses), np.array(yaws), np.array(x_refs)]
    return lines


if __name__ == '__main__':
    # Zenmuse x3 (dji camera) f= 22 or 35 - https://www.dji.com/dk/zenmuse-x3/info  1/2.3 = 6.17 x 4.55  mm = https://en.wikipedia.org/wiki/Image_sensor_format
    camera_info = CameraInfo(focal=22, FOV=94,
                             image_width=1280, image_height=720,
                             render_width=1280, render_height=720,
                             sensor_width=6.17, sensor_height=4.55)

    # Zenmuse x7 https://www.bhphotovideo.com/lit_files/372177.pdf,  https://www.dji.com/dk/zenmuse-x7/info 23.5×15.7 mm
    camera_info = CameraInfo(focal=35, FOV=0,
                             image_width=6016, image_height=3376,
                             render_width=1280, render_height=720,
                             sensor_width=23.5, sensor_height=15.7)

    #camera_info = CameraInfo(focal=20, FOV=94,
    #                         image_width=1280, image_height=720,
    #                         render_width=1280, render_height=720,
    #                         sensor_width=6.17, sensor_height=4.55)



    view = CoverageDisplay3D(1280, 720, camera_info)
    wt = Wireframe.from_stl_path('data/in/turbine_v14.stl')
    r = Rotation.from_euler("XYZ", [0, 0, 90], degrees=True).as_matrix()

    r = TRotation().set_matrix(r, "XYZ")
    t = Transform(np.expand_dims(np.array([-80, 0, 20]), axis=1), r,
                  translate_before_rotate=False)

    wt = wt.transform(t)
    cube = Wireframe.from_stl_path('data/in/camera.stl')
    view.add_object("windturbine", wt)
    view.add_object("camera", cube)
    data = None
    data = readfile_multiple_values("data/in/GT_traj.txt", ",")
    # plot_axis(data[1].transpose(),data[1].transpose(),"axis")
    data= data[:700][:]
    view.run(data)
