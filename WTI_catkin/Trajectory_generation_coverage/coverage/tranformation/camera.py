import cv2
import numpy as np
from scipy.spatial.transform.rotation import Rotation

from tranformation.transform import Transform, TRotation


class CameraInfo:
    def __init__(self, focal: float, FOV: float,
                 image_width: int, image_height: int,
                 render_width, render_height,
                 # camera_state_in_drone_coord: State,
                 sensor_width: float = 2.,
                 sensor_height: float = 1.125
                 ):
        """
        :param focal: in mm
        :param FOV: in degrees
        :param image_width: in pixels
        :param image_height: in pixels
        :param sensor_width: in mm
        :param sensor_height: in mm
        """
        self.focal_length = focal
        self.FOV = FOV
        self.image_width = image_width
        self.image_height = image_height
        self.render_size = (render_width, render_height)
        self.sensor_w = sensor_width
        self.sensor_h = sensor_height
        # camera_sensor_w = 6.17  # mm
        # camera_sensor_h = 4.55  # mm

        self.ratio = image_width / image_height
        if self.ratio != render_width / render_height:
            print(
                f"Camera image size and rendersize is not exactly the same ratio: im_ratio={self.ratio} and render_ratio={image_width / image_height} ")
        self.t_w2c: Transform = None
        self.t_c2w = None

    def update_transform(self, world_to_camera):
        self.t_w2c = world_to_camera
        # self.t_c2w = np.linalg.inv(world_to_camera)
        self.t_c2w = world_to_camera.inverse()

    def transform_to_camera(self, point):
        return self.t_w2c(point)

        # return self.t_w2c @ np.concatenate((point, [1])).reshape(4, 1)

    def perspective_projection(self, location: np.ndarray):
        """
         NOTE UNREAL x = Depth, y = Width, z = Height,
         so we start converting these for easy understanding
         :returns Film Coords x=width y=height (x=0 , y=0 is center) in mm
        """
        X = location[1]
        Y = location[2]
        Z = location[0]

        x = self.focal_length * X / Z
        y = self.focal_length * Y / Z
        return x, y

    def perspective_projection_array(self, location: np.ndarray):
        """
         NOTE UNREAL x = Depth, y = Width, z = Height,
         so we start converting these for easy understanding
         :returns Film Coords x=width y=height (x=0 , y=0 is center) in mm
        """
        l = location.transpose()
        X = l[1]
        Y = l[2]
        Z = l[0]

        xs = self.focal_length * X / Z
        ys = self.focal_length * Y / Z
        return xs, ys

    def pixel_coord(self, x: float, y: float):
        """
        x = 0 is center of image
        y = 0 is center of image
        https://www.cse.psu.edu/~rtc12/CSE486/lecture13.pdf mx, my = m_aff
        https://youtu.be/qByYk6JggQU?t=170 mx, my
        procentage"""
        camera_sensor_w = self.sensor_w  # mm
        camera_sensor_h = self.sensor_h  # mm
        w = self.image_width
        h = self.image_height
        mx = w / camera_sensor_w  # pixel / mm
        my = h / camera_sensor_h  # pixel / mm
        u = (mx * x) + (w / 2)
        v = (my * y) + (h / 2)
        # u = (mx * x / self.image_width) + offset_x / 2  # * width
        # v = -((my * y / self.image_height) + offset_y / 2)  # * height
        return u / w, v / h

    def pixel_coord_array(self, x: float, y: float):
        """
        x = 0 is center of image
        y = 0 is center of image
        procentage"""
        camera_sensor_w = self.sensor_w  # mm
        camera_sensor_h = self.sensor_h  # mm
        w = self.image_width
        h = self.image_height
        mx = w / camera_sensor_w  # pixel / mm
        my = h / camera_sensor_h  # pixel / mm
        u = (mx * x) + (w / 2)
        v = (my * y) + (h / 2)
        # u = (mx * x / self.image_width) + offset_x / 2  # * width
        # v = -((my * y / self.image_height) + offset_y / 2)  # * height
        return u / w, v / h

    def transform_cam_to_image(self, point):
        "if u and v is in range [0,1] then it is inside the frame"
        x, y = self.perspective_projection(point)
        u, v = self.pixel_coord(x, y)
        if isinstance(u, float):
            return u * self.render_size[0], v * self.render_size[1]
        return u[0] * self.render_size[0], v[0] * self.render_size[1]

    def transform_cam_to_image_array(self, points):
        "if u and v is in range [0,1] then it is inside the frame"
        xs, ys = self.perspective_projection_array(points)
        us, vs = self.pixel_coord(xs, ys)
        us = np.expand_dims(us.transpose(), axis=2)
        vs = np.expand_dims(vs.transpose(), axis=2)
        return us * self.render_size[0], vs * self.render_size[1]

    def transform_world_to_image(self, point):
        point = self.transform_to_camera(point)
        u, v = self.transform_cam_to_image(point)
        return u, v

    def compute_transform(self, rpy, translate, translate_before_rotate):
        R = Rotation.from_euler("XYZ", rpy, degrees=True).as_matrix()
        rotate = TRotation().set_matrix(R, "XYZ")
        T_world2camera = Transform(np.expand_dims(-translate, axis=1), rotate,
                                   translate_before_rotate=translate_before_rotate)
        T_camera2world = T_world2camera.inverse()
        # Update transform
        self.t_w2c = T_world2camera
        self.t_c2w = T_camera2world

        return T_world2camera, T_camera2world


if __name__ == '__main__':
    ci = CameraInfo(focal=22, FOV=94, image_width=1280, image_height=720)

    point = np.array([1, 0, 0.1])
    for i in range(20):
        point += np.array([i, 0, 0])
        p1 = point + np.array([0, 0.1, 0])
        p2 = point + np.array([0, -0.5, 0])
        im = np.zeros((720, 1280))
        for p in [p1, p2]:
            u, v = ci.transform_cam_to_image(p)
            if 0 <= u <= 1279 and 0 <= v <= 719:
                im[int(v), int(u)] = 1
        im[int(720 / 2), int(1280 / 2)] = 1
        cv2.imshow("test", im)
        if cv2.waitKey(0):
            debuh = 0
