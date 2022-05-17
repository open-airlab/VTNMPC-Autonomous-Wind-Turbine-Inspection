import os
import time

import numpy as np
from matplotlib import pyplot as plt
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D
from typing import List


class Section:
    def __init__(self, list_of_points, merged=False):
        self.points = list_of_points.transpose()
        self.number_of_points = len(self.points[0])
        self.fit = {}
        self.color = (np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9))
        self.double_section = merged

    def get_fit(self, deg):
        if deg not in self.fit:
            t = np.array(np.linspace(0, 1, self.number_of_points))
            fit_x = np.polyfit(t, self.points[0], deg)
            fit_y = np.polyfit(t, self.points[1], deg)
            fit_z = np.polyfit(t, self.points[2], deg)
            self.fit[deg] = (fit_x, fit_y, fit_z)
        return self.fit[deg]

    def interpolate(self, n: int, deg: int):
        if self.number_of_points == 1:
            return np.array(self.points)
        N = np.array(np.linspace(0, 1, n))
        if deg not in self.fit:
            self.get_fit(deg)
        result = []
        fit_x, fit_y, fit_z = self.fit[deg]
        for fit in [fit_x, fit_y, fit_z]:
            fit_eq = 0
            for i in range(deg):
                fit_eq += np.power(N, (deg - i)) * fit[i]
            fit_eq += fit[-1]
            result.append(fit_eq)
        return np.array(result)

    def interpolate_dist(self, meter=0.2, deg=1):
        if self.number_of_points == 1:
            return np.array(self.points)
        N = np.array(np.linspace(0, 1, self.number_of_points))
        if deg not in self.fit:
            self.get_fit(deg)
        result = []
        fit_x, fit_y, fit_z = self.fit[deg]
        for fit in [fit_x, fit_y, fit_z]:
            fit_eq = 0
            for i in range(deg):
                fit_eq += np.power(N, (deg - i)) * fit[i]
            fit_eq += fit[-1]
            result.append(fit_eq)
        points = np.array(result).transpose()
        dist = 0
        cur = points[0]
        for p in points:
            dist += np.linalg.norm(cur - p)
            cur = p
        number = int(np.rint(dist / meter))
        res = self.interpolate(number, deg)
        return res

    def try_fit(self, deg, max_tolerance_meter):
        if self.number_of_points == 1:
            return True
        dist, mean = self.eval_fit(deg)
        return mean <= max_tolerance_meter  # If below tolerance then success

    def eval_fit(self, deg):
        fit_points = self.interpolate(self.number_of_points, deg).transpose()
        points = self.points.transpose()
        dist = []
        for fp, p in zip(fit_points, points):
            dist.append(np.linalg.norm(fp - p))
        m = np.mean(dist)  # Find mean or avg for all dist
        return dist, m

    def split(self, deg, max_tolerance_meter, title=""):
        min_fit = float("inf")
        min_sections = []
        points = self.points.transpose()
        for i in range(2, self.number_of_points - 2):
            s1 = Section(points[:i])
            s2 = Section(points[i:])
            d1, m1 = s1.eval_fit(deg)
            d2, m2 = s2.eval_fit(deg)
            if m1 <= max_tolerance_meter and m2 <= max_tolerance_meter:
                # if title != "0":
                #     s1.plot_3d(title + f"_S1_m{m1}", deg, dpi=100)
                #     s2.plot_3d(title + f"_S2_m{m2}", deg, dpi=100)
                if m1 + m2 < min_fit:
                    min_sections = [s1, s2]
                    min_fit = m1 + m2
        assert len(min_sections) > 0
        return min_sections

    def plot_3d(self, title, deg, dpi=600):
        fig = plt.figure(dpi=dpi)
        ax = Axes3D(fig)
        ax.grid(False)
        ax.set_facecolor('white')
        ax.plot(self.points[0], self.points[1], self.points[2], label='Original Global Path', lw=2, c='blue')
        new = self.interpolate(self.number_of_points, deg)
        ax.plot(new[0], new[1], new[2], label=f'Interpolated Trajectory {title}', lw=2, c='black')

        ax.legend()
        ax.set_title(title, fontsize=18)
        # plt.xlim(-50, -110)
        # plt.savefig(f'data/fig/{title}_plot.png')
        plt.show()
        plt.clf()

    def add_to_plot_3d(self, ax, show=False):
        new = self.interpolate(self.number_of_points, deg)
        ax.plot(new[0], new[1], new[2], lw=2, c=self.color)
        if show:
            plt.show()

    def recompute(self):
        self.fit = {}
        self.number_of_points = len(self.points[0])


class SectionOneAxis:
    def __init__(self, list_of_points, merged=False):
        self.points = list_of_points.transpose()
        self.number_of_points = len(self.points)
        self.fit = {}
        self.color = (np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9))
        self.double_section = merged

    def get_fit(self, deg):
        if deg not in self.fit:
            t = np.array(np.linspace(0, 1, self.number_of_points))
            fit = np.polyfit(t, self.points, deg)
            self.fit[deg] = fit
        return self.fit[deg]

    def interpolate(self, n: int, deg: int):
        if self.number_of_points == 1:
            return np.array(self.points)
        N = np.array(np.linspace(0, 1, n))
        if deg not in self.fit:
            self.get_fit(deg)
        result = []
        fit = self.fit[deg]

        fit_eq = 0
        for i in range(deg):
            fit_eq += np.power(N, (deg - i)) * fit[i]
        fit_eq += fit[-1]
        return fit_eq

    def interpolate_dist(self, meter=0.2, deg=1):
        # if self.glue_section:
        #     deg = 50
        if self.number_of_points == 1:
            return np.array(self.points)
        N = np.array(np.linspace(0, 1, self.number_of_points))
        if deg not in self.fit:
            self.get_fit(deg)
        result = []
        fit_x, fit_y, fit_z = self.fit[deg]
        for fit in [fit_x, fit_y, fit_z]:
            fit_eq = 0
            for i in range(deg):
                fit_eq += np.power(N, (deg - i)) * fit[i]
            fit_eq += fit[-1]
            result.append(fit_eq)
        points = np.array(result).transpose()
        dist = 0
        cur = points[0]
        for p in points:
            dist += np.linalg.norm(cur - p)
            cur = p
        number = int(np.rint(dist / meter))
        res = self.interpolate(number, deg)
        # if self.glue_section:
        #     i = int(np.rint(number / 4))
        #     res = res.transpose()[i:-i].transpose()
        return res

    def try_fit(self, deg, max_tolerance_meter):
        if self.number_of_points == 1:
            return True
        dist, mean = self.eval_fit(deg)
        return mean <= max_tolerance_meter  # If below tolerance then success

    def eval_fit(self, deg):
        fit_points = self.interpolate(self.number_of_points, deg).transpose()
        points = self.points.transpose()
        dist = []
        for fp, p in zip(fit_points, points):
            dist.append(np.linalg.norm(fp - p))
        m = np.mean(dist)  # Find mean or avg for all dist
        return dist, m

    def split(self, deg, max_tolerance_meter, title=""):
        print("Try split")
        min_fit = float("inf")
        min_sections = []
        points = self.points
        for i in range(2, self.number_of_points - 2):
            s1 = SectionOneAxis(points[:i])
            s2 = SectionOneAxis(points[i:])
            d1, m1 = s1.eval_fit(deg)
            d2, m2 = s2.eval_fit(deg)
            print(m1, m2)
            # s1.plot_2d(title + f"_S1_m{m1}", deg, dpi=100)
            # s2.plot_2d(title + f"_S2_m{m2}", deg, dpi=100)
            if m1 <= max_tolerance_meter and m2 <= max_tolerance_meter:
                if m1 + m2 < min_fit:
                    min_sections = [s1, s2]
                    min_fit = m1 + m2
        assert len(min_sections) > 0
        return min_sections

    def plot_3d(self, title, deg, dpi=600):
        fig = plt.figure(dpi=dpi)
        ax = Axes3D(fig)
        ax.grid(False)
        ax.set_facecolor('white')
        ax.plot(self.points[0], self.points[1], self.points[2], label='Original Global Path', lw=2, c='blue')
        new = self.interpolate(self.number_of_points, deg)
        ax.plot(new[0], new[1], new[2], label=f'Interpolated Trajectory {title}', lw=2, c='black')

        ax.legend()
        ax.set_title(title, fontsize=18)
        # plt.xlim(-50, -110)
        # plt.savefig(f'data/fig/{title}_plot.png')
        plt.show()
        plt.clf()

    def add_to_plot_3d(self, ax, show=False):
        new = self.interpolate(self.number_of_points, deg)
        ax.plot(new[0], new[1], new[2], lw=2, c=self.color)
        if show:
            plt.show()

    def plot_2d(self, title, deg, dpi=600):
        plt.figure(dpi=dpi)
        new = self.interpolate(self.number_of_points, deg)
        plt.plot(np.linspace(0, 1, len(self.points)), self.points, label=f'Original Global Path - y', lw=2, c='blue')
        plt.plot(np.linspace(0, 1, len(new)), new, label=f'Interpolated Trajectory - y', lw=2, c='black')
        plt.show()

    def recompute(self):
        self.fit = {}
        self.number_of_points = len(self.points[0])


def readfile(filename, expand=False):
    file1 = open(filename, 'r')
    Lines = file1.readlines()

    count = 0
    # Strips the newline character
    lines = []
    for line in Lines:
        value = float(line.strip())
        if value == -0:
            value = 0
        lines.append(value)
    lines = np.array(lines)
    if expand:
        lines = np.expand_dims(lines, axis=1)
    return lines


def readfile_multiple_values(filename, split_char=" ", expand=False):
    file1 = open(filename, 'r')
    Lines = file1.readlines()

    count = 0
    # Strips the newline character
    lines = []
    for line in Lines:
        line = line.strip()
        l_values = line.split(split_char)
        values = []
        for v in l_values:
            value = float(v)
            if value == -0:
                value = 0
            values.append(value)
        lines.append(values)
    lines = np.array(lines)
    if expand:
        lines = np.expand_dims(lines, axis=1)
    return lines


def writefile(filename, data):
    with open(filename, 'w') as file:
        for data_point in data:
            L = [f"{str(data_point)}\n"]
            file.writelines(L)


def writefile_multiple_values(filename, data, split_char=" "):
    with open(filename, 'w') as file:
        for data_points in data:
            s = split_char.join(data_points.astype(str))
            L = [f"{str(s)}\n"]
            file.writelines(L)


def save_traj(filename, traj):
    if traj.shape[0] != 3:
        data = traj.transpose()
    else:
        data = traj
    writefile(f"data/out/{filename}_x.txt", data[0])
    writefile(f"data/out/{filename}_y.txt", data[1])
    writefile(f"data/out/{filename}_z.txt", data[2])


def pf(data, deg, smooth):
    data = remove_duplicates(data)
    titles = []
    data_list = []
    for degrees in deg:
        for smoothnes in smooth:
            titles.append(f"d{degrees}s{smoothnes}")
            tck, u = interpolate.splprep(data, k=degrees, s=smoothnes)
            # here we generate the new interpolated dataset,
            # increase the resolution by increasing the spacing, 500 in this example
            new = interpolate.splev(np.linspace(0, 1, 5000), tck, der=0)
            data_list.append(new)

    # now lets plot it!
    for new, t in zip(data_list, titles):
        plot_data(data, new, t, save=True, dpi=100)
        plot_axis(data, new, t, dpi=100)
        plt.clf()


def polynomial_fit(deg: list, smooth: list):
    from scipy.interpolate import splprep, splev
    px = readfile("data/in/gp_output/px_160.txt", expand=True)
    py = readfile("data/in/gp_output/py_160.txt", expand=True)
    pz = readfile("data/in/gp_output/pz_160.txt", expand=True)

    data = np.concatenate((px, py), axis=1)
    data = np.concatenate((data, pz), axis=1)

    data = data.astype(float)
    # Remove duplicates:
    last_point = None
    points = []
    for point in data:
        if np.all(point == last_point):
            continue
        points.append(point)
        last_point = point

    data = np.array(points)
    data = data.transpose()

    # now we get all the knots and info about the interpolated spline
    titles = []
    data_list = []
    for degrees in deg:
        for smoothnes in smooth:
            titles.append(f"d{degrees}s{smoothnes}")
            tck, u = interpolate.splprep(data, k=degrees, s=smoothnes)
            # here we generate the new interpolated dataset,
            # increase the resolution by increasing the spacing, 500 in this example
            new = interpolate.splev(np.linspace(0, 1, 5000), tck, der=0)
            data_list.append(new)

    # now lets plot it!
    for new, t in zip(data_list, titles):
        plot_data(data, new, t, save=True)
        plot_axis(data, new, t)
        plt.clf()


def trunc(values, decs=0):
    return np.trunc(values * 10 ** decs) / (10 ** decs)


def split_data_into_sections_by_normals(data):
    data_points = data[:, :3]
    normals = data[:, 3:]
    splitted_points = []
    tmp_list = []  # tmp_list is the list containing the points that are grouped
    old_n = None
    for data_point, normal in zip(data_points, normals):
        n = trunc(normal, 3)  # We truncate to estimate the normals instead of using exact values
        if np.any(n != old_n):  # If the normal changes we start a new group
            if len(tmp_list) != 0:  # Check if we should add old group
                splitted_points.append(Section(np.array(tmp_list.copy())))
                tmp_list = []
            old_n = n
        tmp_list.append(data_point)
    if len(tmp_list) != 0:  # Check if we should add old group
        splitted_points.append(Section(np.array(tmp_list.copy())))
    return splitted_points


def split_data_into_sections_by_normals_ax(data):
    data_points = data[:, :3]
    normals = data[:, 3:]
    splitted_points_x = []
    splitted_points_y = []
    splitted_points_z = []
    tmp_list = []  # tmp_list is the list containing the points that are grouped
    old_n = None

    for data_point, normal in zip(data_points, normals):
        n = trunc(normal, 3)  # We truncate to estimate the normals instead of using exact values
        if np.any(n != old_n):  # If the normal changes we start a new group
            if len(tmp_list) != 0:  # Check if we should add old group
                tmp_list = np.array(tmp_list.copy()).transpose()
                splitted_points_x.append(SectionOneAxis(tmp_list[0]))
                splitted_points_y.append(SectionOneAxis(tmp_list[1]))
                splitted_points_z.append(SectionOneAxis(tmp_list[2]))
                tmp_list = []
            old_n = n
        tmp_list.append(data_point)
    if len(tmp_list) != 0:  # Check if we should add old group
        tmp_list = np.array(tmp_list.copy()).transpose()
        splitted_points_x.append(SectionOneAxis(tmp_list[0]))
        splitted_points_y.append(SectionOneAxis(tmp_list[1]))
        splitted_points_z.append(SectionOneAxis(tmp_list[2]))
    return splitted_points_x, splitted_points_y, splitted_points_z


def split_data_into_sections_by_fit(sections, deg, max_tolerance_meter):
    result = []
    for section_nr, section in enumerate(sections):
        if not section.try_fit(deg, max_tolerance_meter):
            split_sections = section.split(deg, max_tolerance_meter, title=str(section_nr))
            for s in split_sections:
                result.append(s)
        else:
            result.append(section)
    return result


def split_data_into_sections_by_fit_ax(sections: List[SectionOneAxis], deg, max_tolerance_meter):
    result = []
    for section_nr, section in enumerate(sections):
        if not section.try_fit(deg, max_tolerance_meter):
            split_sections = section.split(deg, max_tolerance_meter, title=str(section_nr))
            for s in split_sections:
                result.append(s)
        else:
            result.append(section)
    return result


def merge_sections(sections, deg):
    for i, section in enumerate(sections):
        if i == 0:
            continue
        last_section = sections[i - 1]
        points = np.concatenate([last_section.points.transpose(), section.points.transpose()])
        test_section = Section(points, merged=True)
        fit_points = test_section.interpolate(test_section.number_of_points, deg).transpose()
        points = test_section.points.transpose()
        dist_x = []
        dist_y = []
        dist_z = []
        for fp, p in zip(fit_points, points):
            dist_x.append(np.linalg.norm(fp[0] - p[0]))
            dist_y.append(np.linalg.norm(fp[1] - p[1]))
            dist_z.append(np.linalg.norm(fp[2] - p[2]))
        m_x = np.mean(dist_x)  # Find mean or avg for all dist
        m_y = np.mean(dist_y)  # Find mean or avg for all dist
        m_z = np.mean(dist_z)  # Find mean or avg for all dist

        debug = 0


def merge_sections_ax(sections, deg, max_tolerance_meter):
    new_added = True
    while new_added:
        result = []
        new_added = False
        skip = False
        for i, section in enumerate(sections):
            if i == 0:
                continue
            if skip:
                skip = False
                if i == len(sections) - 1:
                    result.append(section)
                continue
            last_section = sections[i - 1]
            points = np.concatenate([last_section.points.transpose(), section.points.transpose()])
            test_section = SectionOneAxis(points, merged=True)
            # fit_points = test_section.interpolate(test_section.number_of_points, deg).transpose()
            dist, m = test_section.eval_fit(deg)
            worst_case = max(dist)
            # test_section.plot_2d("", 1, 100)
            if worst_case < max_tolerance_meter:
                debug = 0
                new_added = True
                skip = True
                result.append(test_section)
            else:
                result.append(last_section)
                if i == len(sections) - 1:
                    result.append(section)
            debug = 0
        sections = result
    return sections


def split_data_into_sections(data):
    # Extract points and normals
    data_points = data[:, :3]
    normals = data[:, 3:]

    # Splitted_points is the resulting list
    splitted_points = []
    tmp_list = []  # tmp_list is the list containing the points that are grouped
    old_n = None
    old_uv = None
    points_to_remember = []
    new_section = False
    for data_point, normal in zip(data_points, normals):
        n = trunc(normal, 3)  # We truncate to estimate the normals instead of using exact values
        if np.any(n != old_n):  # If the normal changes we start a new group
            new_section = True
        elif len(tmp_list) > 1:
            p = tmp_list[-2]
            v = p - data_point
            unit_vector = v / np.linalg.norm(v)
            if old_uv is None:
                old_uv = unit_vector
            else:
                if not np.allclose(old_uv, unit_vector, 0.001):
                    debug = 0
                    print("SHIFT")
                    new_section = False
        if new_section:
            if len(tmp_list) != 0:  # Check if we should add old group
                splitted_points.append(np.array(tmp_list.copy()))
                tmp_list = []
            old_n = n
            new_section = False
        tmp_list.append(data_point)
    return splitted_points


def np_polyfit(deg, folder="data/in/gp_output",
               filenames=["px_160.txt", "py_160.txt", "pz_160.txt", "nx_160.txt", "ny_160.txt", "nz_160.txt"],
               out_prefix="", skip_rest=True):
    px = readfile(f"{folder}/{filenames[0]}", expand=True)
    py = readfile(f"{folder}/{filenames[1]}", expand=True)
    pz = readfile(f"{folder}/{filenames[2]}", expand=True)
    data_points = np.concatenate((px, py, pz), axis=1)
    # data_points = np.concatenate((data_points, pz), axis=1)
    # Normals
    nx = readfile(f"{folder}/{filenames[3]}", expand=True)
    ny = readfile(f"{folder}/{filenames[4]}", expand=True)
    nz = readfile(f"{folder}/{filenames[5]}", expand=True)
    normals = np.concatenate((nx, ny, nz), axis=1)
    data = np.concatenate((data_points, normals), axis=1)

    #############################################################
    # Remove duplicates:
    #############################################################
    data = remove_duplicates(data)
    data_points = data[:, :3]
    data_normals = data[:, 3:]

    #############################################################
    # split into sections (by axis)
    #############################################################
    sections_x, sections_y, sections_z = split_data_into_sections_by_normals_ax(data)
    sections_axis = []
    for sections in [sections_x, sections_y, sections_z]:
        sections = split_data_into_sections_by_fit_ax(sections, deg=1, max_tolerance_meter=0.8)
        sections = merge_sections_ax(sections, deg, max_tolerance_meter=0.5)
        sections_axis.append(sections)
        debug = 0

    traj = []
    for sections in sections_axis:
        new_traj = []
        for sec in sections:
            new_traj.append(sec.interpolate(sec.number_of_points, deg=1))
        new_traj = np.concatenate(new_traj, axis=0)
        traj.append(new_traj)
    traj = np.array(traj)
    save_traj(f"{out_prefix}fit_noiterp", traj)
    save_traj(f"{out_prefix}no_fit", data_points)
    plot_axis(data.transpose(), traj, f"{out_prefix}", dpi=100)
    dist_in_m = 0.01
    # for i in range(2, 10):
    #     dist_in_m = i / 100
    traj, interp_normals = interpolate_dist(dist_in_m, traj, data_normals)
    d = traj[:-1] - traj[1:]
    save_traj(f"{out_prefix}d{str(int(dist_in_m * 100))}cm_interp", traj)
    save_traj(f"{out_prefix}d{str(int(dist_in_m * 100))}cm_interp_n", interp_normals)
    debug = 0
    ax = plot_data(data_points.transpose(), traj, "TEST", show=True)

    if skip_rest:
        return

    #############################################################
    # split into sections (Normal way)
    #############################################################
    sections = split_data_into_sections_by_normals(data)
    sections = split_data_into_sections_by_fit(sections, deg=1, max_tolerance_meter=1.2)
    # sections = merge_sections(sections, deg=1)
    # Plot x,y,z separately
    # new_traj = []
    # for section in sections:
    #     new_traj.append(section.interpolate(section.number_of_points, deg=1))
    # new_traj = np.concatenate(new_traj, axis=1)
    # plot_axis(data.transpose(), new_traj, "axis_plot")

    #############################################################
    # # Make transitions (Add endpoint to next section)
    #############################################################
    # for i, section in enumerate(sections):
    #     if i != 0:
    #         i1 = sections[i - 1].interpolate(n=2, deg=1).transpose()
    #         section.points = np.concatenate([np.array([i1[-1]]).transpose(), section.points], axis=1)
    #         section.recompute()
    #         debug = 0
    # new_traj = []
    # for section in sections:
    #     new_traj.append(section.interpolate(section.number_of_points, deg=1))
    # new_traj = np.concatenate(new_traj, axis=1)
    # ax = plot_data(data.transpose(), new_traj, "TEST", show=False)
    # for section in sections:
    #     section.add_to_plot_3d(ax)
    # plt.show()

    #############################################################
    # Make transitions (Add sections in between normal sections)
    #############################################################
    new_sections = []
    for i, section in enumerate(sections):
        if i != 0:
            i1 = sections[i - 1].interpolate(n=2, deg=1).transpose()
            i2 = section.interpolate(n=2, deg=1).transpose()
            # dp = np.concatenate([i1, i2], axis=0)
            dp = np.concatenate([[i1[-1]], [i2[0]]], axis=0)
            new_sections.append(Section(dp))
        new_sections.append(section)
    sections = new_sections

    #############################################################
    # Dist trajectory
    #############################################################
    new_traj = []
    for sec in sections:
        new_traj.append(sec.interpolate_dist(meter=0.1))
    new_traj = np.concatenate(new_traj, axis=1)
    debug = 0
    ax = plot_data(data.transpose(), new_traj, "TEST", show=True)

    #############################################################
    # Smoothing of the trajectory
    #############################################################
    # new_traj = []
    # for sec in sections:
    #     new_traj.append(sec.interpolate_dist(meter=1, deg=1))
    # new_traj = np.concatenate(new_traj, axis=1)
    # new_traj.astype(float)
    # # pf(new_traj, [4,5], [None, 0])

    # new_traj = remove_duplicates(new_traj)
    # tck, u = interpolate.splprep(new_traj, k=2, s=80)
    # # here we generate the new interpolated dataset,
    # # increase the resolution by increasing the spacing, 500 in this example
    # new = interpolate.splev(np.linspace(0, 1, 5000), tck, der=0)
    #
    # # now lets plot it!
    # plot_data(data.transpose(), new, "test", save=False, show=True)
    # # plot_axis(data.transpose(), new, "test")
    # plt.clf()

    #############################################################
    # Draw color 3d plot
    #############################################################
    new_traj = []
    for section in sections:
        new_traj.append(section.interpolate(section.number_of_points, deg=1))
    new_traj = np.concatenate(new_traj, axis=1)
    ax = plot_data(data.transpose(), new_traj, "TEST", show=False)
    for section in sections:
        section.add_to_plot_3d(ax)
    plt.savefig(f'data/fig/axis3d_plot_color.png')
    plt.show()

    #############################################################
    # Draw color 2d plot
    #############################################################
    # plot_axis(data.transpose(), new_traj, "TEST")
    plot_axis_color(sections, "axis_plot_color")
    debug = 0


def remove_duplicates(data, expected_values=3):
    transpose = False
    if data.shape[0] == expected_values:
        transpose = True
        data = data.transpose()
    last_point = data[0]
    points = [last_point]
    for point in data:
        if np.allclose(point, last_point, 0.001):
            continue
        points.append(point)
        last_point = point
    data = np.array(points)
    if transpose:
        data = data.transpose()
    return data


def polyfit_np_axis(d, data, full_dist, deg):
    fit = np.polyfit(d, data, deg)
    fit_eq = 0
    for i in range(deg):
        fit_eq += np.power(full_dist, (deg - i)) * fit[i]
    fit_eq += fit[-1]
    return fit_eq


def plot_data(data, new, title, save=False, show=True, dpi=600):
    # now lets plot it!
    if data.shape[0] != 3:
        data = data.transpose()
    if new.shape[0] != 3:
        new = new.transpose()
    fig = plt.figure(dpi=dpi)
    ax = Axes3D(fig)
    ax.grid(False)
    ax.set_facecolor('white')
    ax.plot(data[0], data[1], data[2], label='Original Global Path', lw=2, c='blue')
    ax.plot(new[0], new[1], new[2], label='Interpolated Trajectory', lw=2, c='black')
    ax.legend()
    # plt.xlim(-50, -110)
    plt.savefig(f'data/fig/{title}_plot.png')
    if show:
        plt.show()
        plt.clf()

    if save:
        writefile(f"data/out/px_B_spline_interp_{title}.txt", new[0])
        writefile(f"data/out/py_B_spline_interp_{title}.txt", new[1])
        writefile(f"data/out/pz_B_spline_interp_{title}.txt", new[2])
        plt.clf()
    return ax


def plot_axis(data, new, title, dpi=600):
    plt.figure(dpi=dpi)
    plt.plot(np.linspace(0, 1, len(data[0])), data[0], label=f'Original Global Path - x', lw=2, c='blue')
    plt.plot(np.linspace(0, 1, len(new[0])), new[0], label=f'Interpolated Trajectory - x', lw=2, c='black')
    plt.title('X ' + title)
    plt.savefig(f'data/fig/{title}_x.png')
    # plt.clf()
    plt.show()
    plt.figure(dpi=dpi)
    plt.plot(np.linspace(0, 1, len(data[1])), data[1], label=f'Original Global Path - y', lw=2, c='blue')
    plt.plot(np.linspace(0, 1, len(new[1])), new[1], label=f'Interpolated Trajectory - y', lw=2, c='black')
    plt.title('Y' + title)
    plt.savefig(f'data/fig/{title}_y.png')
    # plt.clf()
    plt.show()
    plt.figure(dpi=dpi)
    plt.plot(np.linspace(0, 1, len(data[2])), data[2], label=f'Original Global Path - z', lw=2, c='blue')
    plt.plot(np.linspace(0, 1, len(new[2])), new[2], label=f'Interpolated Trajectory - z', lw=2, c='black')
    plt.title('Z')
    plt.savefig(f'data/fig/{title}_z.png')
    plt.show()
    # plt.clf()


def plot_one_axis(data, new, title, dpi=600):
    plt.figure(dpi=dpi)
    plt.plot(np.linspace(0, 1, len(data)), data, label=f'Original Global Path - x', lw=2, c='blue')
    if new:
        plt.plot(np.linspace(0, 1, len(new)), new, label=f'Interpolated Trajectory - x', lw=2, c='black')
    plt.title(title)
    plt.savefig(f'data/fig/{str(title).replace(" ", "_")}.png')
    plt.show()


def plot_axis_color(sections, title):
    data = []
    for section in sections:
        data.append(section.points)
    data = np.concatenate(data, axis=1)
    t = np.linspace(0, 1, len(data[0]))
    for axis in [0, 1, 2]:
        plt.figure(dpi=600)
        plt.plot(t, data[axis], label=f'Original Global Path', lw=2, c='blue')
        index = 0
        for section in sections:
            points = section.interpolate(section.number_of_points, 1)
            points = points[axis]
            plt.plot(t[index:index + len(points)], points, lw=2, c=section.color)
            index += len(points)
        debug = 0
        plt.savefig(f'data/fig/{title}_A{axis}.png')
        plt.show()

    # plt.title('X ' + title)
    # plt.savefig(f'data/fig/{title}_x.png')
    # # plt.clf()
    # plt.show()
    # plt.figure(dpi=600)
    # plt.plot(np.linspace(0, 1, len(data[1])), data[1], label=f'Original Global Path - y', lw=2, c='blue')
    # plt.plot(np.linspace(0, 1, len(new[1])), new[1], label=f'Interpolated Trajectory - y', lw=2, c='black')
    # plt.title('Y' + title)
    # plt.savefig(f'data/fig/{title}_y.png')
    # # plt.clf()
    # plt.show()
    # plt.figure(dpi=600)
    # plt.plot(np.linspace(0, 1, len(data[2])), data[2], label=f'Original Global Path - z', lw=2, c='blue')
    # plt.plot(np.linspace(0, 1, len(new[2])), new[2], label=f'Interpolated Trajectory - z', lw=2, c='black')
    # plt.title('Z')
    # plt.savefig(f'data/fig/{title}_z.png')
    # plt.show()
    # # plt.clf()


def interpolate_dist(dist_in_m, traj, normals, include_small_sections=False, expected_number_of_values=3):
    transposed = False
    if traj.shape[0] == expected_number_of_values:
        traj = traj.transpose()
        transposed = True
    result = []
    result_ns = []
    numbers = []
    last_dist = 0
    for i, (point, n) in enumerate(zip(traj, normals)):
        if i == 0:
            last_point = traj[i]
            last_normal = n
            continue

        dist = np.linalg.norm(point - last_point)
        number = int(np.rint(dist / dist_in_m))

        if dist / number < dist_in_m - 0.01 or dist / number > dist_in_m + 0.01:
            debug = 0
            continue
        last_dist = 0
        if include_small_sections:
            number = max(number, 1)
        numbers.append(number)
        if number == 0:
            print(f"Small section detected: dist={dist}, is_added={include_small_sections}")
            debug = 0
        interp = np.linspace(last_point, point, number)
        ns = np.linspace(last_normal, last_normal, number)
        if np.all(point == traj[-1]):
            result.append(interp)
            result_ns.append(ns)
        else:
            if number == 1:
                result.append(interp)
                result_ns.append(ns)
            else:
                result.append(interp[:-1])
                result_ns.append(ns[:-1])
        last_point = point
        last_normal = n
        debug = 0
    result = np.concatenate(result, axis=0)
    result_ns = np.concatenate(result_ns, axis=0)
    diff = np.linalg.norm(result[:-1] - result[1:], axis=1)
    plot_one_axis(diff * 100, None, f"Distance difference in cm between points {dist_in_m}")
    print(diff)
    print(f"min: {min(diff)}, max: {max(diff)}")
    if transposed:
        debug = 0
        result = result.transpose()
        result_ns = result_ns.transpose()
    return result, result_ns


def interpolate_dist_rotation(dist_in_m, traj, roations, include_small_sections=False, expected_number_of_values=3,
                              plot_prefix=""):
    transposed = False
    if traj.shape[0] == expected_number_of_values:
        traj = traj.transpose()
        transposed = True
    result = []
    numbers = []
    for i, point in enumerate(traj):
        if i == 0:
            continue
        last_point = traj[i - 1]
        dist = np.linalg.norm(point - last_point)
        if dist < dist_in_m - 1 or dist > dist_in_m + 1:
            debug = 0
        number = int(np.rint(dist / dist_in_m))
        if include_small_sections:
            number = max(number, 1)
        numbers.append(number)
        if number == 0:
            print(f"Small section detected: dist={dist}, is_added={include_small_sections}")
            debug = 0
        interp_pos = np.linspace(last_point, point, number)
        rot = roations[i]
        last_rot = roations[i - 1]
        interp_rot = np.linspace(last_rot, rot, number)
        interp = np.concatenate([interp_pos, interp_rot], axis=1)
        if np.all(point == traj[-1]):
            result.append(interp)
        else:
            if number == 1:
                result.append(interp)
            else:
                result.append(interp[:-1])
        debug = 0
    result = np.concatenate(result, axis=0)
    pos = result[:, :3]

    diff = np.linalg.norm(pos[:-1] - pos[1:], axis=1)
    plot_one_axis(diff * 100, None, "GP Distance difference in cm between points")
    print(diff)
    print(f"min: {min(diff)}, max: {max(diff)}")
    if transposed:
        debug = 0
        result = result.transpose()
    return result


if __name__ == '__main__':
    deg = 1
    np_polyfit(deg, folder="data/in/mp_output",
               filenames=["p162_x.txt", "p162_y.txt", "p162_z.txt", "n162_x.txt", "n162_y.txt", "n162_z.txt"],
               out_prefix="mp_")
    debug = 0
    # values = readfile_multiple_values("data/in/gp_path.txt", expand=False)
    # values = remove_duplicates(values, 6)
    # pos = values[:, :3]
    # angles = values[:, 3:]
    # debug = 0
    # interp_dist = 0.05
    # result = interpolate_dist_rotation(0.05, pos, angles, plot_prefix="gp_path")
    # writefile_multiple_values(f"data/out/gp_path_d{str(int(interp_dist * 100))}cm_interp.txt",
    #                           result)
    # debug = 0

# polynomial_fit([5, 3, 2, 1], [10, 20, 30, 80])
