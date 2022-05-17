import time

import airsim
import numpy as np

import geopy.distance
import yaml
from msgpackrpc import error
from functools import partial
from scipy.spatial.transform import Rotation as R


class AirSimBackend(object):
    def __init__(self, debug_mode=False):
        self.objects = {}
        self.offline_mode = debug_mode
        self.wind_direction = 0.0
        self.wind_str = 0.0
        self.wind_change = None

    @staticmethod
    def client_check(func):
        client = airsim.MultirotorClient()
        try:
            client.ping()  # Check if connected
            func(client)
            return True
        except error.RPCError:
            print("No connection established.")
            return False

    def request_data(self):
        self.objects = {}
        if self.offline_mode:
            self.objects = offline_test_set
            return self.objects
        client = airsim.MultirotorClient()
        try:
            client.ping()  # Check if connected
            gps_vehicle = client.getGpsData()
            ue4_vehicle = client.simGetVehiclePose()
            object_names = [f"WindTurbine{i}" for i in range(50)]
            object_names.append("DroneLandingPlatform")
            for object_name in object_names:
                pose = client.simGetObjectPose(object_name)
                if not pose.containsNan():
                    result_dict = {}
                    lat, lng = self.convert_ue4_to_gps_coord(gps_vehicle, ue4_vehicle, pose)
                    result_dict["name"] = object_name
                    result_dict["gps"] = {"lat": lat,
                                          "lng": lng,
                                          "wind_direction": "??"}
                    result_dict["ue4_pose"] = {"x": round(pose.position.x_val, 4),
                                               "y": round(pose.position.y_val, 4),
                                               "yaw": round(pose.orientation.z_val, 4)}
                    result_dict["state"] = "UNKNOWN"  # Used to determine blade rotation so state 1,2, and unknown
                    result_dict["height"] = "100"  # Meters
                    result_dict["blade_radius"] = "40"  # Meters
                    result_dict["info_text"] = yaml.dump(result_dict)
                    self.objects[object_name] = result_dict

            client = None
            return self.objects
        except error.RPCError:
            print("No connection established.")
            self.objects = offline_test_set
            return self.objects

    def convert_ue4_to_gps_coord(self, gps_anchor: airsim.GpsData, ue4_anchor: airsim.Pose, ue4_pose: airsim.Pose):
        """
        http://www.edwilliams.org/avform147.htm#LL
        :param gps_anchor:
        :param ue4_anchor:
        :param ue4_pose:
        :return:
        """
        # Anchor
        lat1 = gps_anchor.gnss.geo_point.latitude
        lon1 = gps_anchor.gnss.geo_point.longitude

        # Compute distance:
        relative_position = ue4_pose.position - ue4_anchor.position
        relative_position = np.array([relative_position.x_val, relative_position.y_val])
        relative_position_unit = relative_position / np.linalg.norm(relative_position)
        distance = np.linalg.norm(relative_position)

        # angle
        angle = np.arctan2(relative_position_unit[1], relative_position_unit[0]) - np.arctan2(1, 0)
        return self.gps_given_distance(lat1, lon1, distance, angle)

    def gps_given_distance(self, lat1, lon1, distance, angle):
        """
        :param lat1: degrees
        :param lon1: degrees
        :param distance: in meters
        :param angle: in radians
        :return:
        """
        # Define starting point.
        start = geopy.Point(lat1, lon1)

        # Define a general distance object, initialized with a distance of 1 km.
        d = geopy.distance.distance(kilometers=distance / 1000)

        # Use the `destination` method with a bearing of 0 degrees (which is north)
        # in order to go from point `start` 1 km to north.
        final = d.destination(point=start, bearing=np.rad2deg(angle))
        return final.latitude, final.longitude

    def set_wind_angle(self, deg):
        self.set_wind(deg, self.set_wind_str)

    def set_wind_str(self, str):
        self.set_wind(self.wind_direction, str)

    def set_wind(self, wind_angle, wind_str, turb_info_drone_pos=np.array([0, 0, 0])):
        if self.offline_mode:
            return False

        def set_wind_func(wind_angle, wind_str, turb_info_drone_pos, client):
            wind = self.deg_to_vector2d(wind_angle)
            wind = self.apply_strength(wind, wind_str)
            wind = np.append(wind, 0) * -1 + turb_info_drone_pos
            wind = airsim.Vector3r(wind[0], wind[1], wind[2])
            if self.wind_change is None or wind.x_val != self.wind_change.x_val or wind.y_val != self.wind_change.y_val:
                print("Airsim_bg: Wind: ", wind)
                self.wind_change = wind
            client.simSetWind(wind)
            self.wind_str = wind_str
            self.wind_direction = wind_angle
            self.rotate_wind_turbines()

        return self.client_check(partial(set_wind_func, wind_angle, wind_str, turb_info_drone_pos))

    def deg_to_vector2d(self, deg):
        rad = self.deg_to_rad(deg)
        return np.array([np.cos(rad), np.sin(rad)])

    @staticmethod
    def apply_strength(wind_vec, strength):
        norm1 = wind_vec / np.sum(np.abs(wind_vec))
        return norm1 * strength

    def takeoff(self):
        if self.offline_mode:
            return False

        def takeoff_func(client):
            client.enableApiControl(True)
            state = client.getMultirotorState()
            print("Takeoff received")
            if state.landed_state == airsim.LandedState.Landed:
                print("taking off...")
                client.takeoffAsync(timeout_sec=5).join()
            else:
                client.hoverAsync().join()

        return self.client_check(takeoff_func)

    def rotate_wind_turbines(self):
        if self.offline_mode:
            return False

        def rotate_wind_turbines(client: airsim.MultirotorClient):
            for name, data in self.objects.items():
                if "Turbine" in name:
                    old_pose = data["ue4_pose"]
                    client.simSetObjectPose(name,
                                            airsim.Pose(
                                                position_val=airsim.Vector3r(old_pose["x"], old_pose["y"], 16.25),
                                                orientation_val=airsim.to_quaternion(0, 0, self.deg_to_rad(
                                                    (self.wind_direction - 90) % 360))))

        return self.client_check(rotate_wind_turbines)

    @staticmethod
    def deg_to_rad(deg):
        return deg * (np.pi / 180)

    def get_drone_position(self):
        client = airsim.MultirotorClient()
        try:
            client.ping()  # Check if connected
            pose = client.simGetVehiclePose()
            return pose.position.x_val, pose.position.y_val, pose.position.z_val
        except error.RPCError:
            print("No connection established.")
            drone_pos = offline_test_set['DroneLandingPlatform']['ue4_pose']
            return drone_pos['x'], drone_pos['y'], 0.0


offline_test_set = {'WindTurbine1': {'name': 'WindTurbine1',
                                     'gps': {'lat': 47.644405024486666, 'lng': -122.14203758607212,
                                             'wind_direction': '??'}, 'ue4_pose': {'x': 140.7, 'y': 326.55, 'yaw': 0.0},
                                     'state': 'UNKNOWN', 'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.644405024486666\n  lng: -122.14203758607212\n  wind_direction: ??\nheight: '100'\nname: WindTurbine1\nstate: UNKNOWN\nue4_pose:\n  x: 140.7\n  y: 326.55\n  yaw: 0.0\n"},
                    'WindTurbine2': {'name': 'WindTurbine2',
                                     'gps': {'lat': 47.64307299105318, 'lng': -122.14203753846934,
                                             'wind_direction': '??'}, 'ue4_pose': {'x': 140.7, 'y': 178.45, 'yaw': 0.0},
                                     'state': 'UNKNOWN', 'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.64307299105318\n  lng: -122.14203753846934\n  wind_direction: ??\nheight: '100'\nname: WindTurbine2\nstate: UNKNOWN\nue4_pose:\n  x: 140.7\n  y: 178.45\n  yaw: 0.0\n"},
                    'WindTurbine3': {'name': 'WindTurbine3', 'gps': {'lat': 47.641827301124, 'lng': -122.14203749395502,
                                                                     'wind_direction': '??'},
                                     'ue4_pose': {'x': 140.7, 'y': 39.95, 'yaw': 0.0}, 'state': 'UNKNOWN',
                                     'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.641827301124\n  lng: -122.14203749395502\n  wind_direction: ??\nheight: '100'\nname: WindTurbine3\nstate: UNKNOWN\nue4_pose:\n  x: 140.7\n  y: 39.95\n  yaw: 0.0\n"},
                    'WindTurbine4': {'name': 'WindTurbine4',
                                     'gps': {'lat': 47.64440503963954, 'lng': -122.13999331300536,
                                             'wind_direction': '??'}, 'ue4_pose': {'x': -12.9, 'y': 326.55, 'yaw': 0.0},
                                     'state': 'UNKNOWN', 'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.64440503963954\n  lng: -122.13999331300536\n  wind_direction: ??\nheight: '100'\nname: WindTurbine4\nstate: UNKNOWN\nue4_pose:\n  x: -12.9\n  y: 326.55\n  yaw: 0.0\n"},
                    'WindTurbine5': {'name': 'WindTurbine5',
                                     'gps': {'lat': 47.643073006205555, 'lng': -122.1399933173698,
                                             'wind_direction': '??'}, 'ue4_pose': {'x': -12.9, 'y': 178.45, 'yaw': 0.0},
                                     'state': 'UNKNOWN', 'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.643073006205555\n  lng: -122.1399933173698\n  wind_direction: ??\nheight: '100'\nname: WindTurbine5\nstate: UNKNOWN\nue4_pose:\n  x: -12.9\n  y: 178.45\n  yaw: 0.0\n"},
                    'WindTurbine6': {'name': 'WindTurbine6',
                                     'gps': {'lat': 47.64182731627592, 'lng': -122.13999332145107,
                                             'wind_direction': '??'}, 'ue4_pose': {'x': -12.9, 'y': 39.95, 'yaw': 0.0},
                                     'state': 'UNKNOWN', 'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.64182731627592\n  lng: -122.13999332145107\n  wind_direction: ??\nheight: '100'\nname: WindTurbine6\nstate: UNKNOWN\nue4_pose:\n  x: -12.9\n  y: 39.95\n  yaw: 0.0\n"},
                    'WindTurbine7': {'name': 'WindTurbine7',
                                     'gps': {'lat': 47.64440501514529, 'lng': -122.13778800026614,
                                             'wind_direction': '??'},
                                     'ue4_pose': {'x': -178.6, 'y': 326.55, 'yaw': 0.0}, 'state': 'UNKNOWN',
                                     'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.64440501514529\n  lng: -122.13778800026614\n  wind_direction: ??\nheight: '100'\nname: WindTurbine7\nstate: UNKNOWN\nue4_pose:\n  x: -178.6\n  y: 326.55\n  yaw: 0.0\n"},
                    'WindTurbine8': {'name': 'WindTurbine8',
                                     'gps': {'lat': 47.64307298171209, 'lng': -122.13778806069158,
                                             'wind_direction': '??'},
                                     'ue4_pose': {'x': -178.6, 'y': 178.45, 'yaw': 0.0}, 'state': 'UNKNOWN',
                                     'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.64307298171209\n  lng: -122.13778806069158\n  wind_direction: ??\nheight: '100'\nname: WindTurbine8\nstate: UNKNOWN\nue4_pose:\n  x: -178.6\n  y: 178.45\n  yaw: 0.0\n"},
                    'WindTurbine9': {'name': 'WindTurbine9', 'gps': {'lat': 47.6418272917832, 'lng': -122.1377881171966,
                                                                     'wind_direction': '??'},
                                     'ue4_pose': {'x': -178.6, 'y': 39.95, 'yaw': 0.0}, 'state': 'UNKNOWN',
                                     'height': '100', 'blade_radius': '40',
                                     'info_text': "blade_radius: '40'\ngps:\n  lat: 47.6418272917832\n  lng: -122.1377881171966\n  wind_direction: ??\nheight: '100'\nname: WindTurbine9\nstate: UNKNOWN\nue4_pose:\n  x: -178.6\n  y: 39.95\n  yaw: 0.0\n"},
                    'DroneLandingPlatform': {'name': 'DroneLandingPlatform',
                                             'gps': {'lat': 47.641468449707624, 'lng': -122.140165,
                                                     'wind_direction': '??'},
                                             'ue4_pose': {'x': 0.0, 'y': 0.05, 'yaw': 0.0}, 'state': 'UNKNOWN',
                                             'height': '100', 'blade_radius': '40',
                                             'info_text': "blade_radius: '40'\ngps:\n  lat: 47.641468449707624\n  lng: -122.140165\n  wind_direction: ??\nheight: '100'\nname: DroneLandingPlatform\nstate: UNKNOWN\nue4_pose:\n  x: 0.0\n  y: 0.05\n  yaw: 0.0\n"}}

if __name__ == '__main__':
    bg = AirSimBackend()
    bg.set_wind(0, 10)
    bg.set_wind(45, 10)
    bg.set_wind(90, 10)
    bg.set_wind(135, 10)
    bg.set_wind(180, 10)
    bg.set_wind(225, 10)
    # while 1:
    #     distance = input("Enter distance:")
    #     angle = input("Enter angle:")
    #     test = bg.gps_given_distance(45, 45, int(distance), np.deg2rad(float(angle)))
    #     print(test)
