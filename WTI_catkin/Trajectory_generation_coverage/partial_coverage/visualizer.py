import pygame
import numpy as np
from math import *
from stl import mesh
from scipy.spatial.transform import Rotation
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transform_manager import TransformManager

from objects.mesh_base import Mesh, Points
from tranformation.camera import CameraInfo
from tranformation.transform import TRotation, Transform


def get_objects():
    your_mesh = mesh.Mesh.from_file('data/in/cube.stl')
    cube = Mesh(your_mesh.vectors, WHITE)
    points = Points(np.array([[0, 0, 0],
                              [0, 1, 0],
                              [0, -1, 0]]), RED)
    return [cube, points]


WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

WIDTH, HEIGHT = 1280, 720
pygame.display.set_caption("3D projection in pygame!")
screen = pygame.display.set_mode((WIDTH, HEIGHT))
center = (int(WIDTH / 2), int(HEIGHT / 2))

roll = 0
pitch = 0
yaw = 0
camera_translation = np.array([-30, 0, 0]).astype(float)

pygame.init()
fps_font = pygame.font.SysFont("Arial", 20)
clock = pygame.time.Clock()
# Zenmuse x3 (dji camera) f= 22 or 35 - https://www.dji.com/dk/zenmuse-x3/info  1/2.3 = 6.17 x 4.55  mm = https://en.wikipedia.org/wiki/Image_sensor_format
camera = CameraInfo(focal=22, FOV=94, image_width=1280, image_height=720, sensor_width=6.17, sensor_height=4.55)
# Zenmuse x7 https://www.bhphotovideo.com/lit_files/372177.pdf,  https://www.dji.com/dk/zenmuse-x7/info 23.5×15.7 mm
camera = CameraInfo(focal=16, FOV=0, image_width=1280, image_height=720, sensor_width=23.5, sensor_height=15.7)

orbit_center = np.array([0, 0, 0]).astype(float)


def show_fps():
    fps_text = str(int(clock.get_fps()))
    fps_surface = fps_font.render(fps_text, True, pygame.Color("green"))
    screen.blit(fps_surface, (WIDTH - 30, 10))


def compute_transform(translate_before_rotate=True):
    T_world2camera_tmp = np.eye(4)
    R = Rotation.from_euler("XYZ", [roll, pitch, yaw], degrees=True).as_matrix()
    T_world2camera_tmp[:3, :3] = R
    if translate_before_rotate:
        translate = R @ -camera_translation
    else:
        translate = -camera_translation
    T_world2camera_tmp[:3, 3] = translate
    T_camera2world_tmp = np.linalg.inv(T_world2camera_tmp)

    rotate = TRotation().set_matrix(R, "XYZ")
    T_world2camera = Transform(np.expand_dims(-camera_translation, axis=1), rotate,
                               translate_before_rotate=translate_before_rotate)
    T_camera2world = T_world2camera.inverse()
    return T_world2camera, T_camera2world


def unit_vector(v):
    return v / np.linalg.norm(v)


def lookat(cam, target, up=np.array([0, 0, 1])):
    forward = [1, 0, 0]
    v_forward = unit_vector(target - cam)
    dot_product = np.dot([1, 0], v_forward[:-1])
    angle = np.arccos(dot_product)
    yaw = angle * 180 / np.pi

    # if v_forward[0] == 1:
    #     return (0, 0, 0)
    # elif v_forward[0] == -1:
    #     return (0, 0, 180)

    right_v = unit_vector(np.cross(up, v_forward))
    up_v = unit_vector(np.cross(v_forward, right_v))
    r = np.array([[v_forward[0], v_forward[1], v_forward[2]],
                  [right_v[0], right_v[1], right_v[2]],
                  [up_v[0], up_v[1], up_v[2]]])
    # r = np.array([mx[0], my[0], mz[0], 0, mx[1], my[1], mz[1], 0, mx[2], my[2], mz[2], 0, tx, ty, tz, 1])
    R = Rotation.from_matrix(r).as_euler('XYZ', degrees=True)
    return R[0], R[1], R[2]


def lookat2(cam, target, up=np.array([0, 0, 1])):
    pass


projection_matrix = np.array([[1, 0, 0],
                              [0, 1, 0],
                              [0, 0, 0]])

objects = get_objects()
mouse_x = None
mouse_y = None
right_mouse = False
left_mouse = False
translate_before_rotate = True
while True:
    clock.tick(60)
    # pitch += 0.1
    # yaw += 0.1
    change = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        if event.type == pygame.KEYDOWN:
            change = True
            if event.key == pygame.K_ESCAPE:
                pygame.quit()
                exit()
            elif event.key == pygame.K_DOWN or event.key == ord('s'):
                camera_translation += np.array([0.2, 0, 0])
            elif event.key == pygame.K_UP or event.key == ord('w'):
                camera_translation -= np.array([0.2, 0, 0])
            elif event.key == pygame.K_LEFT or event.key == ord('a'):
                camera_translation += np.array([0, 0.2, 0])
            elif event.key == pygame.K_RIGHT or event.key == ord('d'):
                camera_translation -= np.array([0, 0.2, 0])
            elif event.key == ord('q'):
                yaw += 5
            elif event.key == ord('e'):
                yaw -= 5
            elif event.key == ord('f'):
                translate_before_rotate = not translate_before_rotate
                print(f"translate_before_rotate: {translate_before_rotate}")
        elif event.type == pygame.MOUSEBUTTONDOWN:
            print("button down")
            if event.button == 1:  # left click
                # Record starting pos
                mouse_x, mouse_y = event.pos
                right_mouse = True
                pygame.mouse.set_visible(False)
            if event.button == 3:  # right click
                left_mouse = True
                pygame.mouse.set_visible(False)
                mouse_x, mouse_y = event.pos
            if event.button == 4:  # Scroll forward
                translate = camera.t_c2w(np.array([-0.2, 0, 0])).squeeze()
                print(f"B camera_translation: {camera_translation}, A camera_translation {translate}")
                camera_translation = translate
            if event.button == 5:  # Scroll forward
                translate = camera.t_c2w(np.array([0.2, 0, 0])).squeeze()
                print(f"B camera_translation: {camera_translation}, A camera_translation {translate}")
                camera_translation = translate
            change = True
        elif event.type == pygame.MOUSEBUTTONUP:
            print("button up")
            change = True
            if event.button == 1:
                mouse_x, mouse_y = None, None
                right_mouse = False
                pygame.mouse.set_visible(True)
            if event.button == 3:
                mouse_x, mouse_y = None, None
                left_mouse = False
                pygame.mouse.set_visible(True)

        elif event.type == pygame.MOUSEMOTION:
            if mouse_x is not None and mouse_y is not None:
                change = True
                new_mouse_x, new_mouse_y = event.pos
                move_x = np.clip(new_mouse_x - mouse_x, -2, 2)
                move_y = np.clip(new_mouse_y - mouse_y, -2, 2)
                if right_mouse and translate_before_rotate:
                    translate = camera.t_c2w(np.array([0, move_x / 50, move_y / 50])).squeeze()
                    camera_translation = translate
                if left_mouse:
                    # target = camera.transform_to_camera(np.array([0, 0, 0]))
                    # target = np.squeeze(target)[:-1]
                    yaw -= move_x / 20
                    pitch += move_y / 20

                    # camera_translation += np.array([0, move_x / 100, move_y / 100])
                    # roll, pitch, yaw = lookat(camera_translation, np.array([0, 0, 0]))
                    # roll, pitch, yaw = lookat(np.array([0, 0, 0]), target)
                    # print(f"t: {camera_translation} roll: {roll}, pitch: {pitch}, yaw:{yaw}")
                pygame.mouse.set_pos(center)
                mouse_x = center[0]
                mouse_y = center[1]

                # Compute tranform of camera:
    if change:
        print(f"STATE: t: {camera_translation} roll: {roll}, pitch: {pitch}, yaw:{yaw}")
    t_w2c, t_c2w = compute_transform(translate_before_rotate=translate_before_rotate)
    camera.update_transform(t_w2c)
    # update stuff
    screen.fill(BLACK)
    show_fps()
    vertices_c = []  # All points in camera frame
    vertices_im = []
    for object in objects:
        object.draw(screen, camera)

    objects[0].draw_vertices(screen, camera)

    #     for points in object:
    #         vertice_c = []
    #         vertice_im = []
    #         for i in range(len(points)):
    #             # transform into camera
    #             p1_c = t_w2c @ np.concatenate((points[i], [1])).reshape(4, 1)
    #             p1_im = transform_to_frame(p1_c[:-1])
    #             vertice_c.append(p1_c[:-1])
    #             vertice_im.append(p1_im)
    #             #
    #             # p2 = points[(i + 1) % (len(points) - 1)]
    #             # p2_c = t_w2c @ np.concatenate((p2, [1])).reshape(4, 1)
    #         vertices_c.append(vertice_c)
    #         vertices_im.append(vertice_im)
    #
    # # drawing
    # for points in vertices_im:
    #     for i in range(len(points)):
    #         p1 = points[i]
    #         p2 = points[(i + 1) % (len(points) - 1)]
    #         pygame.draw.line(screen, WHITE, p1, p2)

    pygame.display.update()
