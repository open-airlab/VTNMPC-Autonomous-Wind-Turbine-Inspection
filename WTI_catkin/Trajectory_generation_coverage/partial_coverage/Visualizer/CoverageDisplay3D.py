import json
import time
from pathlib import Path

import pygame
import numpy as np
from scipy.spatial.transform import Rotation

from Visualizer.Display3D import Display3D
from Visualizer.utils.colors import *
from objects.mesh_base import Wireframe, unit_vector, random_color
# from shortest_path import get_shortest_path
from tranformation.camera import CameraInfo
from tranformation.transform import TRotation, Transform
from utils.folder import get_project_root


class CoverageDisplay3D(Display3D):
    def __init__(self, width, height, camera_info: CameraInfo, verbose=False):
        super().__init__(width, height, camera_info, verbose=verbose)
        self.max_dist_to_viewed_vertice_meter = 15
        self.converage = 0
        self.displayCoverage = True
        self.only_draw_if_seen = True
        self.dont_add_to_seen = False  # Dont change this
        self.seen_counter = 0
        self.done = False
        self.path = None
        self.read_path = False
        self.data = {}
        self.path_color = [random_color() for _ in range(300)]

    def run(self, traj=None, headless=False):
        """ Display wireframe on screen and respond to keydown events """
        running = True
        key_down = False
        index = 0
        max_index = len(traj[0])
        self.displaceTraj = self.displaceTraj if traj is not None else False
        self.traj = traj
        target_fps = 200 if not headless else 800
        while running:
            self.clock.tick(target_fps)
            if traj is None:
                self.handle_user_input()
                self.camera_rpy[1] = 0
            else:
                self.handle_user_input()
                if self.third_person:
                    self.camera_obj_translation = traj[1][index]
                    self.camera_obj_rpy = np.array([0, 0, traj[2][index][0]])
                    if index % 10 == 0:
                        self.print_info(f"STATE: i: {index} t: {self.camera_obj_translation}"
                                        f" roll: {self.camera_obj_rpy[0]}, pitch: {self.camera_obj_rpy[1]}, yaw:{self.camera_obj_rpy[2]}")
                else:
                    self.camera_translation = traj[1][index]
                    self.camera_rpy = np.array([0, 0, traj[2][index][0]])
                    if index % 10 == 0:
                        self.print_info(f"STATE: i: {index} t: {self.camera_translation}"
                                        f" roll: {self.camera_rpy[0]}, pitch: {self.camera_rpy[1]}, yaw:{self.camera_rpy[2]}")
            # if "windturbine" in self.objects:
            #     wt = self.objects["windturbine"]
            #     self.path = self.read_json_file()
            # self.path_color = [random_color() for _ in self.path[1]]
            # self.update_objects()
            draw = index % self.increment_speed == 0 or self.done
            draw = draw and not headless
            self.display(draw=draw)
            if not self.pause:
                index += 1
            index = min(index, max_index - 1)
            self.dont_add_to_seen = index == max_index - 1
            if "windturbine" in self.objects:
                wt = self.objects["windturbine"]
                coverage = wt.get_coverage()
                if coverage > self.converage:
                    self.converage = coverage
                    wt.output_coverage()
                if index == max_index - 1 and not self.done:
                    self.done = True
                    wt.output_avg_vertice_per_img(index)
                    wt.output_seen_count_per_vertice()
                    wt.output_partial_seen_count_per_vertice()
            if draw:
                pygame.display.update()
            if headless and index % target_fps == 0:
                self.screen.fill(self.background)
                self.show_fps()
                pygame.display.update()

        pygame.quit()

    def display(self, verbose=False, draw=True):
        if draw:
            self.screen.fill(self.background)
        transform_stack = self.get_transform_stack()

        if self.displayAxis:
            p0 = np.array([0, 0, 0])
            px = np.array([100, 0, 0])
            py = np.array([0, 100, 0])
            pz = np.array([0, 0, 100])
            colors = [RED, GREEN, BLUE]
            p0_im = self.camera.transform_world_to_image(p0)
            for c, p in zip(colors, [px, py, pz]):
                p_im = self.camera.transform_world_to_image(p)
                pygame.draw.line(self.screen, c, p0_im, p_im, width=5)

        counter = 0
        seen_counter = 0
        for name, obj in self.objects.items():
            o: Wireframe = obj
            if name == "camera":
                if self.third_person:
                    r = Rotation.from_euler("XYZ", self.camera_obj_rpy, degrees=True).as_matrix()

                    cam_r = TRotation().set_matrix(r, "XYZ")
                    cam_t = Transform(np.expand_dims(self.camera_obj_translation, axis=1), cam_r,
                                      translate_before_rotate=False)
                    o = o.transform(cam_t)
                else:
                    continue

            for t in transform_stack:
                o = o.transform(t)

            us, vs = self.camera.transform_cam_to_image_array(o.vertices)
            uvs = np.concatenate([us, vs], axis=2)
            if not self.object_in_image(uvs):
                continue
            for index in o.sorted_vertices_ind():
                vertice = o.vertices[index]
                # Check if it is behind the camera
                if self.camera_view_dir[0] == 1:
                    if vertice[:, 0].max() > 0:
                        self.print_info("behind me!")
                        continue
                else:
                    if vertice[:, 0].min() < 0:
                        self.print_info("behind me!")
                        continue

                # Check if we can see one of the points:
                all_points_in_frame = True
                one_point_in_frame = False
                for point_index in range(len(vertice)):
                    u, v = uvs[index][point_index]
                    in_frame = self.point_in_frame(u, v)[0]
                    all_points_in_frame = all_points_in_frame and in_frame
                    one_point_in_frame = one_point_in_frame or in_frame

                if not one_point_in_frame:
                    continue

                color = o.vertice_colors[index]
                center = o.center[index]
                normal = o.normals[index]
                towards_us = np.dot(normal, self.camera_view_dir)

                # Check if we are seen the backside of the vertice
                projected_point = self.ClosestPointOnLine(center, center + normal, np.array([0, 0, 0]))
                direction = unit_vector(projected_point - center)
                # is_backside1 = not np.isclose(direction[0], normal[0])
                is_backside = not (abs(direction[0] - normal[0]) < 1.e-8)
                # if is_backside != is_backside1:
                #     print(f"NOT CLOSE! {direction[0]}, {normal[0]}")

                towards_us_threshold = 0.0

                if self.displayFaces and draw:
                    if obj.seen[index] or not self.only_draw_if_seen:
                        # Only draw faces that face us
                        if towards_us > towards_us_threshold and not is_backside:
                            points_im = []
                            x = 0
                            y = 0
                            for point in vertice:
                                depht = 0
                                u, v = self.camera.transform_cam_to_image(point)
                                points_im.append((u, v))
                                x += np.clip(u / self.camera.render_size[0], -1, 1)
                                y += np.clip(v / self.camera.render_size[1], -1, 1)
                            if abs(x) == 3 or abs(y) == 3:
                                debug = 0
                                continue
                            a = self.area(points_im)
                            if a >= 10000000:
                                self.print_info(f"skipped by area: {a}")
                                continue
                            counter += 1
                            if verbose:
                                self.print_info(f"drawing: {counter}")
                            pygame.draw.polygon(self.screen, color, points_im, 0)
                if self.displayCenter and draw:
                    color = complement(color)
                    # Only draw faces that face us
                    if towards_us > towards_us_threshold and not is_backside:
                        center = o.center[index]
                        u, v = self.camera.transform_cam_to_image(center)
                        pygame.draw.circle(self.screen, color, (u, v), 5)
                if self.do_coverage:
                    # Compute angle between two vector for both xz and xy:
                    if towards_us > towards_us_threshold and not is_backside:
                        normal_w = obj.normals[index]
                        inspection_normal_w = unit_vector(np.array([normal_w[0], normal_w[1], 0]))
                        camera_view_dir_w = self.camera.t_c2w(
                            self.camera_view_dir).squeeze() - self.camera_translation
                        towards_us_w = np.dot(unit_vector(inspection_normal_w), unit_vector(camera_view_dir_w))
                        angle = np.arccos(towards_us_w / (
                                np.sqrt(inspection_normal_w[0] ** 2 + inspection_normal_w[1] ** 2 +
                                        inspection_normal_w[
                                            2] ** 2) * np.sqrt(
                            camera_view_dir_w[0] ** 2 + camera_view_dir_w[1] ** 2 + camera_view_dir_w[
                                2] ** 2)))
                        angle = np.rad2deg(angle)
                        distance_to_center = self.dist(center)
                        if abs(angle) < 45 and distance_to_center <= self.max_dist_to_viewed_vertice_meter:
                            center = o.center[index]
                            u, v = self.camera.transform_cam_to_image(center)
                            points_in = 0
                            points_in += 1 if self.point_in_frame(u, v)[0] else 0
                            p_im = []
                            for point in vertice:
                                u, v = self.camera.transform_cam_to_image(point)
                                p_im.append((u, v))
                                points_in += 1 if self.point_in_frame(u, v)[0] else 0

                            if points_in == 4:
                                if draw:
                                    for j in range(len(p_im)):
                                        p1 = p_im[j]
                                        p2 = p_im[(j + 1) % len(p_im)]
                                        pygame.draw.line(self.screen, YELLOW, p1, p2, 5)
                                if not self.dont_add_to_seen:
                                    self.seen_counter += 1
                                    obj.seen[index] += 1
                            elif points_in == 3:
                                if not self.dont_add_to_seen:
                                    obj.partial_seen[index] += 1

                # if self.displayEdges:
                #     for (n1, n2) in wireframe.edges:
                #         if self.perspective:
                #             if wireframe.nodes[n1][2] > -self.perspective and nodes[n2][2] > -self.perspective:
                #                 z1 = self.perspective / (self.perspective + nodes[n1][2])
                #                 x1 = self.width / 2 + z1 * (nodes[n1][0] - self.width / 2)
                #                 y1 = self.height / 2 + z1 * (nodes[n1][1] - self.height / 2)
                #
                #                 z2 = self.perspective / (self.perspective + nodes[n2][2])
                #                 x2 = self.width / 2 + z2 * (nodes[n2][0] - self.width / 2)
                #                 y2 = self.height / 2 + z2 * (nodes[n2][1] - self.height / 2)
                #
                #                 pygame.draw.aaline(self.screen, colour, (x1, y1), (x2, y2), 1)
                #         else:
                #             pygame.draw.aaline(self.screen, colour, (nodes[n1][0], nodes[n1][1]),
                #                                (nodes[n2][0], nodes[n2][1]), 1)
                #
                # if self.displayNodes:
                #     for node in nodes:
                #         pygame.draw.circle(self.screen, colour, (int(node[0]), int(node[1])), self.nodeRadius, 0)
        # print(seen_counter)
        if self.displaceTraj and draw:
            last_point = None
            for i, p in enumerate(self.traj[1]):
                if i % 100 == 0 or i == 0:
                    p_im = self.camera.transform_world_to_image(p)
                    if i == 0:
                        last_point = p_im
                        continue
                    pygame.draw.line(self.screen, RED, last_point, p_im, width=5)
                    last_point = p_im
        if self.path is not None and draw:
            indices, path, start_node = self.path
            u, v = self.camera.transform_world_to_image(start_node)
            last_node = (u, v)
            for i, node in enumerate(path):
                u, v = self.camera.transform_world_to_image(node)
                c = self.path_color[i]
                pygame.draw.line(self.screen, c, last_node, (u, v), width=5)
                last_node = (u, v)
        if self.read_path:
            try:
                self.data = self.read_json_file()
            except:
                pass
            if "current_path" in self.data and draw:
                current_path = self.data["current_path"]
                if "windturbine" in self.objects:
                    wt = self.objects["windturbine"]
                    path = wt.center[current_path]
                    for i, node in enumerate(path):
                        c = self.path_color[i]
                        u, v = self.camera.transform_world_to_image(node)
                        if i == 0:
                            last_node = (u, v)
                            continue
                        pygame.draw.line(self.screen, c, last_node, (u, v), width=5)
                        last_node = (u, v)
            print("path read")

        if self.displayFPS and draw:
            self.show_fps()

        if self.displayCoverage and draw:
            self.show_coverage()

    def point_in_frame(self, u, v):
        u_in = (0 <= u / self.camera.render_size[0] <= 1)
        v_in = (0 <= v / self.camera.render_size[1] <= 1)
        return u_in and v_in, u_in, v_in

    @staticmethod
    def ClosestPointOnLine(a, b, p):
        ap = p - a
        ab = b - a
        result = a + np.dot(ap, ab) / np.dot(ab, ab) * ab
        return result

    def dist(self, p_to, p_from=np.array([0, 0, 0])):
        v = p_to - p_from
        return np.linalg.norm(v)

    def show_coverage(self):
        if "windturbine" in self.objects:
            wt = self.objects["windturbine"]
            coverage_text = wt.get_coverage_text()
            coverage_surface = pygame.font.SysFont("Arial", 20).render(coverage_text, True, pygame.Color("green"))
            self.screen.blit(coverage_surface, (self.width - 60, self.height - 30))

    def object_in_image(self, uvs):
        min_u = uvs[:, :, 0].min()
        max_u = uvs[:, :, 0].max()
        min_v = uvs[:, :, 1].min()
        max_v = uvs[:, :, 1].max()
        w, h = self.camera.render_size
        if min_u > w:
            return False
        if max_u < 0:
            return False
        if min_v > h:
            return False
        if max_v < 0:
            return False
        return True

    def read_json_file(self):
        project_folder = get_project_root()
        with open(str(Path(project_folder, 'data/out/shortest_path.json')), 'r') as json_file:
            data = json.load(json_file)
        return data
