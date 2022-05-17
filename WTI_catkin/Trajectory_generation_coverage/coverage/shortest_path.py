import json
import random
import time
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from typing import List

from Visualizer.utils.colors import get_distinct_color
from objects.mesh_base import Wireframe, unit_vector
from polynomial_fitting import plot_data
from tranformation.transform import TRotation, Transform
from utils.folder import get_project_root
from k_means_constrained import KMeansConstrained

cache = {}
data = {}
use_cache = True


class Section:
    def __init__(self, points, normal, color):
        self.points = np.array(points)
        self.normal = normal
        if len(self.points) > 1:
            indices = np.argsort(self.points[:, 2])
            self.points = self.points[indices]
            self.p1 = midpoint(self.points[0], self.points[1])  # lowest point (no need to sort)
            self.p2 = midpoint(self.points[-1], self.points[-2])  # highest point (need to reorder list)
        else:
            self.p1 = self.points[0]
            self.p2 = self.points[0]
        self.length = dist(self.p1, self.p2)
        self.p1_str = str(self.p1)
        self.p2_str = str(self.p2)
        self.is_one_point_section = self.p1_str == self.p2_str
        self.color = color

    def get_other_end(self, p):
        if np.all(np.isclose(p, self.p1)):
            return self.p2
        return self.p1

    def get_other_end_str(self, p_str: str) -> str:
        if p_str == self.p1_str:
            return self.p2_str
        return self.p1_str

    def scale_using_normal(self, multiplier):
        n = self.normal * np.array([1, 1, 0])
        n = unit_vector(n)
        p = self.points + n * multiplier
        return Section(p, self.normal, self.color)

    def get_ordered_points(self, p_str):
        if p_str == self.p1_str:
            return self.points
        return self.points[::-1].tolist()

    def get_ordered_points_and_index(self, p_str):
        ps = self.get_ordered_points(p_str)
        return ps, 1 if self.is_one_point_section else 2


def get_random_color():
    return (np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9))


def read_json_file():
    global cache
    project_folder = get_project_root()
    with open(str(Path(project_folder, 'data/out/shortest_path.json')), 'r') as json_file:
        data = json.load(json_file)
    return data


def writefile(filename, data):
    with open(filename, 'w') as file:
        for data_point in data:
            L = [f"{str(data_point)}\n"]
            file.writelines(L)


def dist(p_to, p_from=np.array([0, 0, 0])):
    v = p_to - p_from
    return np.linalg.norm(v)


def move_to(current: int, currentDistance: float, visited: list, node_order: list, global_best_dist: float, edges,
            points_to_sections, point_to_index, points):
    # Set visited:
    visited[current] = 1
    node_order.append(int(current))
    # Get end of section:
    p = points[current]
    section = points_to_sections[p]
    p2 = section.get_other_end_str(p)

    # Set visisted
    current = point_to_index[p2]
    if visited[current] == 0:
        visited[current] = 1
        node_order.append(int(current))
    # update distance travelled:
    currentDistance += section.length

    cache_key = f"c:{current}v:{str(visited)}"
    if cache_key in cache:
        d = cache[cache_key]
        distance_travelled = d["dist"]
        new_order = d["order"]
        found_solution = d["found"]
        currentDistance += distance_travelled
        node_order.extend(new_order)
        return found_solution, currentDistance, node_order

    indices = np.argsort(edges[current]).astype(int)
    test = edges[current, indices]
    best_order = node_order
    found_solution = True
    all_visited = True
    best_sub_dist = float('inf')
    # print(f"Depth:{len(node_order)}, order[-10]={node_order if len(node_order) < 11 else node_order[-10:]},"
    #       f" dist={currentDistance}, BEST: {global_best_dist}")
    # Set default d_cache
    d_cache = {
        "dist": best_sub_dist,
        "order": [best_sub_dist],
        "found": False
    }
    for neighbour_index in indices:
        if visited[neighbour_index] == 1:
            continue
        all_visited = False
        distance = edges[current, neighbour_index]
        if distance == -1:
            continue
        newDistance = currentDistance + distance
        finished, resulting_dist, resulting_order = move_to(neighbour_index, newDistance,
                                                            visited.copy(),
                                                            node_order.copy(),
                                                            global_best_dist, edges,
                                                            points_to_sections, point_to_index,
                                                            points)
        # if resulting_order == [4, 5, 0, 1, 17, 16, 9, 10, 8, 7, 21, 20, 6, 13, 14, 12, 11, 15, 18, 19, 3, 2]:
        #     debug = 0
        #     # print(f"c:{currentDistance}, nd:{newDistance}, rd:{resulting_dist}, ro:{resulting_order}")
        #     print(f"index: {current}, c: {currentDistance}, d:{distance}")

        # We need to cache result of each branch to find the best branch to go:
        if finished and resulting_dist < best_sub_dist:
            if resulting_dist < 0:
                debug = 0
            if len(resulting_order) < len(node_order):
                debug = 0
            if len(resulting_order) != 22:
                debug = 0
            else:
                debug = 0
            d_cache = {
                "dist": resulting_dist - currentDistance,
                "order": [x for x in resulting_order if x not in node_order],
                "found": finished
            }
            best_sub_dist = resulting_dist
            best_order = resulting_order

        # # Find if it is better than global best
        if found_solution and global_best_dist > resulting_dist:
            global_best_dist = resulting_dist
        #     best_order = resulting_order

    # We are done with this section,
    if all_visited:
        d_cache = {
            "dist": 0,
            "order": [],
            "found": len(best_order) == len(edges[0])
        }
        cache[cache_key] = d_cache
        if len(best_order) != len(edges[0]):
            cache[str(visited)] = False
        else:
            debug = 0
        data["current_path"] = node_order
        # data[save_key] = {"cache": cache, "max_dist": max_dist}
        # write_shortest_path(data)
        return len(best_order) == len(edges[0]), currentDistance, best_order
    else:
        if d_cache is None:
            debug = 0
        cache[cache_key] = d_cache

    return found_solution, best_sub_dist, best_order


def shortest_path(sections: List[Section], multiplier=10, turn_cost=0, test_all_starting_positions=False):
    global cache
    # Change normals to not go in z
    m_sections = []
    for s in sections:
        m_sections.append(s.scale_using_normal(multiplier))
    best_dist = float('inf')
    points_to_sections = {}
    points = []
    normals = []
    points_str = []  # We dont want to convert np arrays on the fly so do it once here
    for s in m_sections:
        n = s.normal
        points_to_sections[s.p1_str] = s
        points_to_sections[s.p2_str] = s
        points.append(s.p1)
        points_str.append(s.p1_str)
        normals.append(n)
        if np.any(s.p1 != s.p2):
            points.append(s.p2)
            points_str.append(s.p2_str)
            normals.append(n)

    # unscaled version:
    points_to_sections_us = {}
    # points_us = []
    points_us_str = []
    for s in sections:
        points_to_sections_us[s.p1_str] = s
        points_to_sections_us[s.p2_str] = s
        # points_us.append(s.p1)
        points_us_str.append(s.p1_str)
        if np.any(s.p1 != s.p2):
            # points_us.append(s.p2)
            points_us_str.append(s.p2_str)

    # Compute cost between nodes (turn cost, dist cost, invalid turns=-1)
    edges = np.zeros((len(points), len(points)))
    for i, p in enumerate(points):
        n1 = normals[i]
        for j in range(i + 1, len(points)):
            p2 = points[j]
            n2 = normals[j]
            if np.all(n1 + n2 == np.zeros(3)):
                d = -1
            else:
                d = dist(p, p2) + (turn_cost if np.all(n1 != n2) else 0)
            edges[i, j] = d
            edges[j, i] = d

    start_node = np.array([-0.035274, -0.137386, 1.745499])
    start_node += np.array([-68, 32, 70])  # Offset
    start_node_cost = []
    for p in points:
        d = dist(p, start_node)
        start_node_cost.append(d)
    shortest_path_start = np.argsort(start_node_cost)
    for start_index in shortest_path_start:
        print(f"new_start = {start_index}")
        # cache = {}
        current = start_index
        currentDistance = start_node_cost[start_index]

        visited = np.zeros((len(points))).tolist()
        point_to_index = {}
        for i, p in enumerate(points_str):
            point_to_index[p] = i
        # unvisited[current] = currentDistance
        m1 = time.perf_counter()
        found_solution, resulting_dist, resulting_order = move_to(current, currentDistance, visited,
                                                                  [], best_dist,
                                                                  edges, points_to_sections,
                                                                  point_to_index, points_str)
        print(f"Time: {time.perf_counter() - m1}")
        get_full_order(multiplier, resulting_order, resulting_dist, points_str, points_to_sections, start_node)
        if found_solution and resulting_dist < best_dist:
            best_dist = resulting_dist
            best_order = resulting_order
            print(f"Best: order={str(best_order)}, dist={best_dist}")
        if not test_all_starting_positions:
            break
    print("Done")
    print(f"Best: order={str(best_order)}, dist={best_dist}")
    resulting_points = []
    point_normals = []
    skip = False
    for index in best_order:
        if skip:
            skip = False
            continue
        p_str = points_us_str[index]
        section_unscaled = points_to_sections_us[p_str]
        ps = section_unscaled.get_ordered_points(p_str)
        # t = points_str[index]
        # s = points_to_sections[t]
        # p = s.get_ordered_points(t)

        resulting_points.extend(ps)
        for _ in ps:
            point_normals.append(section_unscaled.normal)
        skip = not section_unscaled.is_one_point_section
    # plot_data(np.array(resulting_points), np.array(resulting_points), "testing_results")
    # Draw each section with index number
    # debug = 0
    # skip = False
    # for index in best_order:
    #     # if skip:
    #     #     skip = False
    #     #     continue
    #     p_str = points_str[index]
    #     section = points_to_sections[p_str]
    #     ps = section.get_ordered_points(p_str)
    #     with_start = [start_node]
    #     with_start.extend(ps)
    #     plot_data(np.array(resulting_points), np.array(with_start), f"show_order_{index}")
    #     skip = not section.is_one_point_section
    return resulting_points, point_normals


def get_full_order(m, test_order, dist, points_str, points_to_sections, start_node=None):
    resulting_points = []
    if start_node is not None:
        resulting_points.append(([start_node], get_random_color()))
    skip = False
    for index in test_order:
        if skip:
            skip = False
            continue
        p_str = points_str[index]
        section = points_to_sections[p_str]
        c = section.color
        ps = section.get_ordered_points(p_str)
        resulting_points.append((ps, c))
        skip = not section.is_one_point_section
    plot_data_color_connected(resulting_points, f"m_{m}_d_{dist}_o_{str(test_order)}", dpi=300)
    debug = 0


def grouping(obj):
    centers = obj.center
    normals = obj.normals

    grp = {}
    # Group by normals
    string_to_normal = {}
    for i, n in enumerate(normals):
        n += np.zeros(3)
        n = np.round(n, 2)
        if str(n) not in grp:
            grp[str(n)] = []
            string_to_normal[str(n)] = n
        grp[str(n)].append(centers[i])

    grps = []
    lowest_amount_of_triangles = float('inf')
    for k, g in grp.items():
        if len(g) < lowest_amount_of_triangles:
            lowest_amount_of_triangles = len(g)

    for k, g in grp.items():
        if len(g) == lowest_amount_of_triangles:
            # The groups pointing up
            grps.append((string_to_normal[k], g))
        elif len(g) == lowest_amount_of_triangles + 1:
            # the end of the turbines
            # Find the longest dist to between nodes:
            distance = np.full((len(g), len(g)), float("inf"))
            for i in range(len(g)):
                for j in range(i + 1, len(g)):
                    d = dist(g[i], g[j])
                    distance[i, j] = d
                    distance[j, i] = d
            outlier_index = np.min(distance, axis=0).argmax()
            grps.append((string_to_normal[k], [g[outlier_index]]))
            del g[outlier_index]
            grps.append((string_to_normal[k], g))
        else:
            number_of_groups = len(g) / lowest_amount_of_triangles
            np_g = np.array(g)
            clf = KMeansConstrained(
                n_clusters=int(number_of_groups),
                size_min=lowest_amount_of_triangles,
                size_max=lowest_amount_of_triangles,
                random_state=0
            )
            clf.fit_predict(np_g)
            print(clf.cluster_centers_)
            print(clf.labels_)

            for i in range(int(number_of_groups)):
                sub_grp = []
                for node_index, label in enumerate(clf.labels_):
                    if label == i:
                        sub_grp.append(g[node_index])
                grps.append((string_to_normal[k], sub_grp))

            debug = 0
    # plot_data_color(grps, "yes")
    return grps


def plot_data_color(data, title, save=False, show=True, dpi=600):
    # now lets plot it!
    fig = plt.figure(dpi=dpi)
    ax = Axes3D(fig)
    ax.grid(False)
    ax.set_facecolor('white')
    for d_grp in data:
        d = np.array(d_grp).transpose()
        ax.plot(d[0], d[1], d[2], label='Original Global Path', lw=2,
                c=(np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9)))
    ax.legend()
    # plt.xlim(-50, -110)
    plt.savefig(f'data/fig/{title}_plot.png')
    if show:
        plt.show()
        plt.clf()
    return ax


def plot_data_color_connected(data, title, save=False, show=True, dpi=600):
    # now lets plot it!
    plt.clf()
    fig = plt.figure(dpi=dpi)
    try:
        ax = Axes3D(fig, auto_add_to_figure=False)
        fig.add_axes(ax)
    except:
        ax = Axes3D(fig)
    ax.grid(False)
    ax.set_facecolor('white')
    last_point = None
    for d_grp, c in data:
        d = np.array(d_grp).transpose()
        if last_point is not None:
            last_point[0].append(d[0, 0])
            last_point[1].append(d[1, 0])
            last_point[2].append(d[2, 0])
            ax.plot(last_point[0], last_point[1], last_point[2], lw=2, c=color)
        color = c
        ax.plot(d[0], d[1], d[2], lw=2, c=color)
        last_point = [[d[0, -1]], [d[1, -1]], [d[2, -1]]]
    ax.legend()
    # plt.xlim(-50, -110)
    plt.savefig(f'data/fig/{title}_plot.png')
    if show:
        plt.show()
        plt.clf()
    return ax


def plot_data_color_sections(sections, title, save=False, show=True, dpi=600):
    # now lets plot it!
    fig = plt.figure(dpi=dpi)
    ax = Axes3D(fig)
    ax.grid(False)
    ax.set_facecolor('white')
    for d_grp in sections:
        d = d_grp.points.transpose()
        ax.plot(d[0], d[1], d[2], label='Original Global Path', lw=2,
                c=(np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9)))
    ax.legend()
    # plt.xlim(-50, -110)
    plt.savefig(f'data/fig/{title}_plot.png')
    if show:
        plt.show()
        plt.clf()
    return ax


def write_shortest_path(info):
    # Directly from dictionary
    with open('data/out/shortest_path.json', 'w') as outfile:
        json.dump(info, outfile)


def midpoint(p1, p2):
    return (p1 + p2) / 2


def find_dist_of_order(order, sections, multiplier, turn_cost):
    global cache
    cache = {}
    m_sections = []
    for s in sections:
        m_sections.append(s.scale_using_normal(multiplier))
    best_dist = float('inf')
    points_to_sections = {}
    points = []
    normals = []
    points_str = []  # We dont want to convert np arrays on the fly so do it once here
    for s in m_sections:
        n = s.normal
        points_to_sections[s.p1_str] = s
        points_to_sections[s.p2_str] = s
        points.append(s.p1)
        points_str.append(s.p1_str)
        normals.append(n)
        if np.any(s.p1 != s.p2):
            points.append(s.p2)
            points_str.append(s.p2_str)
            normals.append(n)

    edges = np.full((len(points), len(points)), -1., float)
    for index in range(len(order)):
        if index == 0:
            continue
        from_index = order[index - 1]
        to_index = order[index]
        n1 = normals[from_index]
        n2 = normals[to_index]
        p = points[from_index]
        p2 = points[to_index]
        d = dist(p, p2) + (turn_cost if np.all(n1 != n2) else 0)
        edges[from_index, to_index] = d

    start_node = np.array([-0.035274, -0.137386, 1.745499])
    start_node += np.array([-68, 32, 70])  # Offset
    start_cost = dist(points[order[0]], start_node)
    print(f"testing order = {order}")
    # cache = {}
    current = order[0]
    currentDistance = start_cost

    visited = np.zeros((len(points))).tolist()
    point_to_index = {}
    for i, p in enumerate(points_str):
        point_to_index[p] = i
    # unvisited[current] = currentDistance
    m1 = time.perf_counter()
    found_solution, resulting_dist, resulting_order = move_to(current, currentDistance, visited,
                                                              [], best_dist,
                                                              edges, points_to_sections,
                                                              point_to_index, points_str)
    print(f"Time: {time.perf_counter() - m1}")
    get_full_order(multiplier, resulting_order, resulting_dist, points_str, points_to_sections, start_node)
    print("Done")
    print(f"Best: order={str(resulting_order)}, dist={resulting_dist}")


def save_traj(filename, traj):
    if traj.shape[0] != 3:
        data = traj.transpose()
    else:
        data = traj
    writefile(f"data/out/{filename}_x.txt", data[0])
    writefile(f"data/out/{filename}_y.txt", data[1])
    writefile(f"data/out/{filename}_z.txt", data[2])


if __name__ == '__main__':
    seed = 214
    np.random.seed(seed)
    random.seed(seed)
    wt = Wireframe.from_stl_path('data/in/turbine_v6.stl')
    r = Rotation.from_euler("XYZ", [0, 0, 90], degrees=True).as_matrix()

    r = TRotation().set_matrix(r, "XYZ")
    t = Transform(np.expand_dims(np.array([-80, 0, 20]), axis=1), r,
                  translate_before_rotate=False)
    wt = wt.transform(t)
    gps = grouping(wt)
    fit = []
    points = []
    sections = []
    for i, (n, g) in enumerate(gps):
        sections.append(Section(g, n, get_distinct_color(i)))
    # plot_data_color_sections(sections, "te")
    m = 10
    tc = 0

    traj, normals = shortest_path(sections, multiplier=m, turn_cost=tc)
    save_traj(f"p162", np.array(traj))
    save_traj(f"n162", np.array(normals))

    test_order_GP = [4, 5, 0, 1, 17, 16, 9, 10, 8, 7, 21, 20, 6, 13, 14, 12, 11, 15, 18, 19, 3, 2]
    find_dist_of_order(test_order_GP, sections, multiplier=m, turn_cost=tc)

    tc_0_best_order = [4, 5, 0, 1, 8, 7, 21, 20, 6, 13, 14, 9, 10, 17, 16, 12, 11, 15, 18, 19, 3, 2]
    find_dist_of_order(tc_0_best_order, sections, multiplier=m, turn_cost=tc)

    debug = 0
