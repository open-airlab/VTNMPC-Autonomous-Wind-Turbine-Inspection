import time
from abc import abstractmethod, ABC
from pathlib import Path

import pygame
import numpy as np

from tranformation.camera import CameraInfo
from stl import mesh

from tranformation.transform import Transform


class MeshBase(ABC):
    def __init__(self, vertices, color):
        self.vertices = vertices
        self.color = color

    @abstractmethod
    def draw(self, screen, camera, color=None):
        pass


def random_color():
    return (int(np.random.uniform(0, 255)), int(np.random.uniform(0, 255)), int(np.random.uniform(0, 255)))


def unit_vector(v):
    return v / np.linalg.norm(v)


class Mesh(MeshBase):
    def __init__(self, vertices, color):
        super().__init__(vertices, color)
        self.colors = [random_color() for _ in range(len(self.vertices))]
        self.colors = np.array(self.colors)

    def draw(self, screen, camera: CameraInfo, color=None):
        c = color if color is not None else self.color
        for points in self.vertices:
            for i in range(len(points)):
                # transform into camera
                p1_im = camera.transform_world_to_image(points[i])
                p2_im = camera.transform_world_to_image(points[(i + 1) % (len(points) - 1)])
                pygame.draw.line(screen, c, p1_im, p2_im)

    def draw_vertices(self, screen, camera: CameraInfo):
        depth = []
        dist = []
        for points in self.vertices:
            v = []
            center = 0
            for point in points:
                # p_c = np.squeeze(camera.transform_to_camera(point))[:-1]
                p_c = np.squeeze(camera.transform_to_camera(point))
                v.append(p_c)
                center += p_c[0]
            center /= 3
            depth.append(v)
            dist.append(center)
        depth = np.array(depth)
        ind = np.argsort(dist)[::-1]
        depth = depth[ind]
        c = self.colors[ind]
        for i, points in enumerate(depth):
            p = []
            for point in points:
                u, v = camera.transform_cam_to_image(point)
                p.append((u, v))
            pygame.draw.polygon(screen, c[i], p)
        debug = 0


class Points:
    def __init__(self, points, color):
        self.points = points
        self.color = color

    def draw(self, screen, camera: CameraInfo, color=None):
        c = color if color is not None else self.color
        for p in self.points:
            u, v = camera.transform_world_to_image(p)
            pygame.draw.circle(screen, c, (u, v), radius=3)


class Wireframe:
    """ An array of vectors in R3 and list of edges connecting them. """

    def __init__(self, vertices=None, normals=None, vertice_colors=None):
        self.vertices = vertices
        self.vertice_colors = [random_color() for _ in vertices] if vertice_colors is None else vertice_colors
        self.normals = self.compute_normals() if normals is None else normals
        self.center = self.compute_center()
        self.seen = np.zeros(len(self.vertices), int)
        self.partial_seen = np.zeros(len(self.vertices), int)

    def add_vertices(self, vertice_array):
        """ Append 1s to a list of 3-tuples and add to self.nodes. """
        ones_added = np.hstack((vertice_array, np.ones((len(vertice_array), 1))))
        self.vertices = np.vstack((self.vertices, ones_added))

    def add_faces(self, face_list, face_colour=(255, 255, 255)):
        for node_list in face_list:
            num_nodes = len(node_list)
            if all((node < len(self.nodes) for node in node_list)):
                # self.faces.append([self.nodes[node] for node in node_list])
                self.faces.append((node_list, np.array(face_colour, np.uint8)))
                self.add_edges([(node_list[n - 1], node_list[n]) for n in range(num_nodes)])

    def output(self):
        if len(self.vertices) > 1:
            self.output_vertices()
            self.output_normals()
            self.output_coverage()

    def output_vertices(self):
        print("\n --- Vertices --- ")
        for i, vertice in enumerate(self.vertices):
            print(f"  {i}:")
            for (x, y, z) in vertice:
                print(f"  \t{(x, y, z)}")

    def output_normals(self):
        print("\n --- Normals --- ")
        for i, (x, y, z) in enumerate(self.normals):
            print(f"  {i}:\t{(x, y, z)}")

    def output_coverage(self):
        print("\n --- Coverage --- ")
        print(f"  total amount covered: {(len(np.where(self.seen >= 1)[0]) / len(self.seen)) * 100}%")
        print(f"  total triangles: {len(self.seen)} and covered: {len(np.where(self.seen >= 1)[0])}")
        index = np.where(self.seen == 0)
        par_seen = self.partial_seen[index]
        number_par = len(np.where(par_seen >= 1)[0])
        print(
            f"  out of triangles not covered ({len(np.where(self.seen ==0)[0])}) triangle partially covered is:{len(np.where(par_seen >= 1)[0])}")

    def output_avg_vertice_per_img(self, num_img):
        s = np.sum(self.seen)
        print("\n --- avg vertice --- ")
        print(f"  avg vertice seen per image: sum_of_triangle_seen:{s} / timesteps:{num_img} = {s / num_img}")
        return s / num_img

    def output_seen_count_per_vertice(self):
        print("\n --- Seen count per vertice --- ")
        for i, count in enumerate(self.seen):
            print(f"  i={i}: {count}")

    def output_partial_seen_count_per_vertice(self):
        print("\n --- Partial seen count per vertice --- ")
        for i, count in enumerate(self.partial_seen):
            print(f"  i={i}: {count}")

    def get_coverage(self):
        return (len(np.where(self.seen >= 1)[0]) / len(self.seen)) * 100

    def get_coverage_text(self):
        return f"{len(np.where(self.seen >= 1)[0])}/{len(self.seen)}"

    def transform(self, transform: Transform):
        """ Apply a transformation defined by a transformation matrix. """
        # start = time.perf_counter()
        # transformed_vertices = []
        #
        # for vertice in self.vertices:
        #     v = []
        #     for point in vertice:
        #         p = transform(point)
        #         v.append(p)
        #     transformed_vertices.append(v)
        # print(f"handmade time: {time.perf_counter() - start}")
        # m2 = np.array(transformed_vertices).squeeze()
        # start = time.perf_counter()
        # print(f"all at once time: {time.perf_counter() - start} and is equal {np.all(m == m2)}\n\n")
        m = transform.apply_to_matrix(self.vertices)
        return Wireframe(m, vertice_colors=self.vertice_colors)

    def find_centre(self):
        """ Find the spatial centre by finding the range of the x, y and z coordinates. """

        min_values = self.nodes[:, :-1].min(axis=0)
        max_values = self.nodes[:, :-1].max(axis=0)
        return 0.5 * (min_values + max_values)

    def get_boundary_points(self):
        min_x = self.vertices[:, :, 0].argmin(axis=0)
        min_x = self.vertices[:, :, 0].argmin(axis=0)
        min_x = self.vertices[:, :, 0].argmin(axis=0)
        min_x = self.vertices[:, :, 0].argmin(axis=0)

    def sorted_vertices_ind(self):
        l = np.abs(self.vertices[:, :, 0].min(axis=1))
        l2 = np.abs(self.center[:, 0])
        ind = np.lexsort((l2, l))
        # ind = np.argsort(self.vertices[:, :, 0].min(axis=1))
        ind_inv = ind[::-1]
        return ind_inv

    def compute_normals(self):
        v1 = self.vertices[:, 1] - self.vertices[:, 0]
        v2 = self.vertices[:, 2] - self.vertices[:, 0]
        cross1 = np.cross(v1, v2)
        norm = np.linalg.norm(cross1, axis=1)
        self.normals = (cross1.transpose() / norm).transpose()

        # Commented out because it is too slow, but here is the more readable version
        # normals = []
        # for vertice in self.vertices:
        #     v1 = vertice[1] - vertice[0]
        #     v2 = vertice[2] - vertice[0]
        #     normal = unit_vector(np.cross(v1, v2))
        #     normals.append(normal)
        # self.normals = np.array(normals)

        return self.normals

    def compute_center(self):
        centers = np.sum(self.vertices, axis=1) / 3
        # commented out because it is too slow
        # centers = []
        # for vertice in self.vertices:
        #     c = np.array([0, 0, 0], float)
        #     for point in vertice:
        #         c += point
        #     centers.append(c)
        # centers = np.array(centers)
        return centers

    def update(self):
        """ Override this function to control wireframe behaviour. """
        pass

    @staticmethod
    def from_stl(stl):
        vertices = stl.vectors
        normals = stl.units
        return Wireframe(vertices, normals)

    @staticmethod
    def from_stl_path(path):
        stl = mesh.Mesh.from_file(str(path))
        return Wireframe.from_stl(stl)


##############################################################################
############################      BACKUP       ###############################
##############################################################################
# class Wireframe:
#     """ An array of vectors in R3 and list of edges connecting them. """
#
#     def __init__(self, nodes=None):
#         self.nodes = np.zeros((0, 4))
#         self.edges = []
#         self.faces = []
#
#         if nodes:
#             self.add_nodes(nodes)
#
#     def add_nodes(self, node_array):
#         """ Append 1s to a list of 3-tuples and add to self.nodes. """
#
#         ones_added = np.hstack((node_array, np.ones((len(node_array), 1))))
#         self.nodes = np.vstack((self.nodes, ones_added))
#
#     def add_edges(self, edge_list):
#         """ Add edges as a list of 2-tuples. """
#
#         # Is it better to use a for loop or generate a long list then add it?
#         # Should raise exception if edge value > len(self.nodes)
#         self.edges += [edge for edge in edge_list if edge not in self.edges]
#
#     def add_faces(self, face_list, face_colour=(255, 255, 255)):
#         for node_list in face_list:
#             num_nodes = len(node_list)
#             if all((node < len(self.nodes) for node in node_list)):
#                 # self.faces.append([self.nodes[node] for node in node_list])
#                 self.faces.append((node_list, np.array(face_colour, np.uint8)))
#                 self.add_edges([(node_list[n - 1], node_list[n]) for n in range(num_nodes)])
#
#     def output(self):
#         if len(self.nodes) > 1:
#             self.output_nodes()
#         if self.edges:
#             self.output_edges()
#         if self.faces:
#             self.output_faces()
#
#     def output_nodes(self):
#         print("\n --- Nodes --- ")
#         for i, (x, y, z, _) in enumerate(self.nodes):
#             print("   %d: (%d, %d, %d)" % (i, x, y, z))
#
#     def output_edges(self):
#         print("\n --- Edges --- ")
#         for i, (node1, node2) in enumerate(self.edges):
#             print("   %d: %d -> %d" % (i, node1, node2))
#
#     def output_faces(self):
#         print("\n --- Faces --- ")
#         for i, nodes in enumerate(self.faces):
#             print("   %d: (%s)" % (i, ", ".join(['%d' % n for n in nodes])))
#
#     def transform(self, transformation_matrix):
#         """ Apply a transformation defined by a transformation matrix. """
#
#         self.nodes = np.dot(self.nodes, transformation_matrix)
#
#     def find_centre(self):
#         """ Find the spatial centre by finding the range of the x, y and z coordinates. """
#
#         min_values = self.nodes[:, :-1].min(axis=0)
#         max_values = self.nodes[:, :-1].max(axis=0)
#         return 0.5 * (min_values + max_values)
#
#     def sorted_faces(self):
#         return sorted(self.faces, key=lambda face: min(self.nodes[f][2] for f in face[0]))
#
#     def update(self):
#         """ Override this function to control wireframe behaviour. """
#         pass
#
#     @staticmethod
#     def from_stl(stl):
#         vertices = stl.vectors
#         normals = stl.units
#         nodes = []
#         for vertice in vertices:
#
#         debug = 0
#         return Wireframe()
#
#     @staticmethod
#     def from_stl_path(path):
#         stl = mesh.Mesh.from_file(str(path))
#         return Wireframe.from_stl(stl)

if __name__ == '__main__':
    def get_project_root() -> Path:
        return Path(__file__).parent.parent


    wt = Wireframe.from_stl_path(Path(get_project_root(), "data/in/turbine_v2.stl"))
    wt.output()
