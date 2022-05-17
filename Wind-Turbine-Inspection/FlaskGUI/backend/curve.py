import time

import cv2
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def gaussian(x, mu, sigma):
    return 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (x - mu) ** 2 / (2 * sigma ** 2))


def plot3D(x, y, z, old_x, old_y, old_z, turb_mag_limit=5):
    # Creating an empty canvas(figure)
    fig = plt.figure()
    # Using the gca function, we are defining
    # the current axes as a 3D projection
    ax = fig.add_subplot(111, projection='3d')
    # Labelling X-Axis
    ax.set_xlabel('longitude-Axis')
    # Labelling Y-Axis
    ax.set_ylabel('latitude-Axis')
    # Labelling Z-Axis
    ax.set_zlabel('Z-Axis')
    ax.set_xlim(-turb_mag_limit, turb_mag_limit)

    ax.set_ylim(-turb_mag_limit, turb_mag_limit)

    ax.set_zlim(-turb_mag_limit, turb_mag_limit)

    # Plot point:
    # ax.scatter(x, y, z, c='r')
    # ax.scatter(old_x, old_y, old_z, c='b')
    ax.quiver(0, 0, 0, x, y, z, color='r')
    ax.quiver(0, 0, 0, old_x, old_y, old_z, color='b')
    # plt.show()
    img = _convert_to_cv_img(fig)
    return img


def _convert_to_cv_img(fig):
    fig.canvas.draw()
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
    # img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
    #                     sep='')
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


def get_step_size(TI, scalar):
    """
    Compute variance in gaussian distribution
    :param TI: Turbulence intensity
    :param dt: Delta time
    :return:
    """
    variance = (TI / scalar)
    step_size = variance
    return step_size


def time_ms():
    return time.time() * 1000


if __name__ == '__main__':
    current_pos = 0
    TI = 1.0
    rate = 100  # ms
    scalar = 3.5
    dt = 100
    # low_steps, aggresive_steps = 0.1, 0.5
    # step_size = get_step_size(TI, scalar)
    # mu, sigma = 0, 0.1
    # s = np.random.normal(mu, sigma, 3000)
    # count, bins, ignored = plt.hist(s, 30, density=True)
    # plt.plot(bins, gaussian(bins, mu, sigma), linewidth=2, color='r')
    # plt.show()

    old_orientation = np.array([0, 0, 1])
    old_turb_mag = 0
    turb_mag_limit = 5

    old_t_ms = time_ms()
    # new_orientation = np.random.normal(old_orientation, sigma)
    # plot3D(old_orientation[0], old_orientation[1], old_orientation[2], s[0], s[1], s[2])
    while True:
        new_time_ms = time_ms()
        epoch = (new_time_ms - old_t_ms) / rate
        orientation = old_orientation
        step_size = get_step_size(TI, scalar)
        turb_mag = old_turb_mag
        # print("\repochs", epoch, end='')
        for i in range(max(int(np.ceil(epoch)), 1)):
            new_orientation = np.random.normal(orientation, step_size)
            new_orientation = new_orientation / np.linalg.norm(new_orientation)
            orientation = new_orientation
            new_turb_mag = np.random.normal(turb_mag, step_size)
            if new_turb_mag < 0:
                new_turb_mag = abs(new_turb_mag)
            if new_turb_mag > turb_mag_limit:
                new_turb_mag = new_turb_mag - (new_turb_mag - turb_mag_limit)
            new_turb_mag = np.clip(new_turb_mag, 0, turb_mag_limit)
            turb_mag = new_turb_mag

        plot_new_vector = new_orientation * new_turb_mag
        plot_old_vector = old_orientation * old_turb_mag
        img = plot3D(plot_new_vector[0], plot_new_vector[1], plot_new_vector[2],
                     plot_old_vector[0], plot_old_vector[1], plot_old_vector[2], turb_mag_limit)
        cv2.imshow("test", img)
        k = cv2.waitKey(dt)
        if k == 27:  # Esc key to stop
            break
        elif k == ord("t"):
            TI = float(input("\nInput new TI: "))
        elif k == ord("s"):
            scalar = float(input("\nInput new Scalar: "))
        elif k == ord("d"):
            dt = int(input("\nInput new delta time (ms): "))
        elif k == -1:  # normally -1 returned,so don't print it
            # new_pos_g = gaussian(new_orientation, old_orientation, step_size)
            # print(new_pos_g, new_orientation)
            old_orientation = new_orientation
            old_turb_mag = new_turb_mag
        print(f"\rTI: {TI}, dt: {dt}, scalar: {scalar}, step_size(sigma): {step_size}, epochs: {epoch}, mag: {new_turb_mag}", end='')
        old_t_ms = new_time_ms
