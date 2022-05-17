import base64
import threading

import cv2
import numpy as np
from flask import Flask, url_for, render_template, Response, request, redirect, Blueprint
from matplotlib import pyplot as plt

from backend.airsim_backend import AirSimBackend
from backend.windprofile import WindFarm

#####################################################################################################################
#                                           INITIATE BLUEPRINT                                                      #
#####################################################################################################################
inspection_page_bp = Blueprint("inspection_page_bp",
                               __name__,
                               static_folder="../../static",
                               template_folder="../../templates")
#####################################################################################################################
#                                              DEBUG MODE                                                           #
#####################################################################################################################
DEBUG_MODE = False
windfarm_fast_version = True
#####################################################################################################################
#                                           GLOBAL VARIABLES                                                        #
#####################################################################################################################
airsim_backend = AirSimBackend(debug_mode=DEBUG_MODE)
drone_pos = None
mission_info_dict = {"target": ""}
wind_direction = 0
wind_speed = 0
wind_ti = 0
turb_img = ""
turb_vector = (0, 0, 0)

wind_info_drone_pos = (wind_speed, wind_direction)
windfarm = WindFarm(wind_speed=0.1, wind_direction=0, debug_mode=windfarm_fast_version or DEBUG_MODE)
wind_sim = False
use_2D_mode = True or DEBUG_MODE

# lock to control access to variable
dataLock = threading.Lock()
# thread handler
yourThread = threading.Thread()
POOL_TIME = 0.2  # Seconds


#####################################################################################################################
#                                           FUNCTIONALITY                                                           #
#####################################################################################################################
@inspection_page_bp.route('/')
def inspection_page():
    global wind_sim, drone_pos
    wind_sim = False
    windfarm.wts_list = []
    canvas_size = (800, 800)
    # AirSim, check for windmill poses
    data = airsim_backend.request_data()
    drone_pos = airsim_backend.get_drone_position()
    drawing_data = []
    for k, v in data.items():
        drawing_data.append([v["gps"]["lat"], v["gps"]["lng"]])
    drawing_data = np.array(drawing_data)
    centroid = np.mean(drawing_data, axis=0)
    drawing_data_center = drawing_data[:] - centroid

    # Compute drawing position
    buffer = 40  # To make them not spawn on the border
    circle_size = 25  # To remove the size of the dot from borders
    sprite_size = 10
    w, h = canvas_size  # To know how big the canvas is and how much we can scale each point
    x_axis = drawing_data_center[:, 0]
    y_axis = drawing_data_center[:, 1]
    scale_x = (w - (circle_size + buffer)) / (np.max(x_axis) - np.min(x_axis))
    scale_y = (h - (circle_size + buffer)) / (np.max(y_axis) - np.min(y_axis))
    scale = min(scale_x, scale_y)
    drawing_data_center = drawing_data_center * scale - np.array([np.min(x_axis) * scale, np.min(y_axis) * scale])
    if scale_x > scale_y:
        center = (np.max(x_axis * scale) - np.min(x_axis * scale)) / 2
        offset = w / 2 - center
        drawing_data_center += [offset, 0]
    else:
        center = (np.max(y_axis * scale) - np.min(y_axis * scale)) / 2
        offset = h / 2 - center
        drawing_data_center += [0, offset]

    for i, object_name in enumerate(data.keys()):
        x, y = drawing_data_center[i]
        data[object_name]["drawing_data"] = {"canvas_pos": [x + buffer / 2, y + buffer / 2]}

    # Compute default wind-flow
    for k, v in data.items():
        if "Turbine" in k:
            windfarm.add_wind_turbine(name=v["name"],
                                      type_name="SimTurbine",
                                      diameter=float(v['blade_radius']) * 2,
                                      hub_height=float(v['height']),
                                      xy_pos=airsim_to_canvas_xy(v['ue4_pose']['x'], v['ue4_pose']['y']))
    flow_map = windfarm.generate_flow_map(map_size=canvas_size[0], gen_flow_box=False)
    wake_map, ti_map, flow_data, flow_data_blue = extract_maps(flow_map, 800)

    w, h = canvas_size  # To know how big the canvas is and how much we can scale each point
    for i, object_name in enumerate(data.keys()):
        x, y, = airsim_to_canvas_xy(data[object_name]['ue4_pose']['x'], data[object_name]['ue4_pose']['y'])
        x, y, = windfarm.get_pixel_index(x, y)
        x, y = x - circle_size / 2, (h - y) - circle_size / 2
        data[object_name]["drawing_data"] = {"canvas_pos": [x, y]}
    # Handle drone pos:
    drone_canvas_pos_x, drone_canvas_pos_y = get_drone_canvas_pos(sprite_size=sprite_size, h=h)

    page_info = {
        'windturbine_data': data,
        'flow_map': convert_to_utf8(wake_map),
        'ti_map': convert_to_utf8(ti_map),
        'flow_data_blue': convert_to_utf8(flow_data_blue),
        'drone_pos': (drone_canvas_pos_x, drone_canvas_pos_y)
    }
    return render_template("inspection.html", page_info=page_info, canvas_size=canvas_size)


def get_drone_canvas_pos(sprite_size=10, h=800):
    global drone_pos, wind_sim
    if drone_pos is None:
        return 0, 0
    if not wind_sim:
        x, y, z = airsim_backend.get_drone_position()
        drone_pos = airsim_to_canvas_xyz(x, y, z)
    drone_canvas_pos_x, drone_canvas_pos_y = windfarm.get_pixel_index(drone_pos[0], drone_pos[1])
    drone_canvas_pos_x -= sprite_size / 2
    drone_canvas_pos_y = (h - drone_canvas_pos_y) - (sprite_size / 2)
    return drone_canvas_pos_x, drone_canvas_pos_y


def airsim_to_canvas_xyz(x, y, z):
    new_x, new_y = airsim_to_canvas_xy(x, y)
    return new_x, new_y, -z


def airsim_to_canvas_xy(x, y):
    return y, x


def plot_map(plot_func, map_size, dpi):
    fig = plt.figure(figsize=(int(map_size / dpi), int(map_size / dpi)), dpi=dpi)
    plot_func()
    fig.canvas.draw()
    img = _convert_to_cv_img(fig)
    plt.clf()
    return img


def _convert_to_cv_img(fig):
    img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
                        sep='')
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


def extract_maps(flow_map, map_size, dpi=100):
    img_ti = plot_map(flow_map.plot_ti_map, map_size, dpi)
    img_wake = plot_map(flow_map.plot_wake_map, map_size, dpi)

    flow_data = flow_map.WS_eff.values.squeeze()
    flow_data_visual = np.copy(flow_data)
    if flow_data_visual.min() < 0:
        flow_data_visual += abs(flow_data_visual.min())
    # flow_data_visual = flow_data_visual * min(255 / 30, 255 / flow_data_visual.max())
    flow_data_visual = flow_data_visual * (255 / flow_data_visual.max())
    flow_data_visual = (flow_data_visual / 2) + (255 / 2)
    flow_data_visual = flow_data_visual.astype(np.uint8)
    flow_data_visual = np.flip(flow_data_visual, axis=0)
    # flow_data_visual = 255 - flow_data_visual.astype(np.uint8)

    # layer = np.zeros(flow_data_visual.shape[:2])
    # flow_data_blue = np.stack((flow_data_visual, layer, layer), axis=-1)
    flow_data_blue = cv2.applyColorMap(flow_data_visual, cv2.COLORMAP_OCEAN)
    return img_wake, img_ti, flow_data, flow_data_blue


@inspection_page_bp.route("/drone_canvas_pos", methods=["GET"])
def drone_canvas_pos():
    x, y = get_drone_canvas_pos()
    return {"responds": (x, y)}


@inspection_page_bp.route("/get_wind_info", methods=["GET"])
def get_wind_info():
    global wind_info_drone_pos, turb_img, wind_ti, turb_vector
    if not wind_sim:
        wind_info_drone_pos = (wind_speed, wind_direction)
    ws, wd = wind_info_drone_pos
    return {"responds": {"ws": ws,
                         "wd": wd,
                         "wti": wind_ti,
                         "wturb": str(turb_vector),
                         "turb_img": turb_img}}


@inspection_page_bp.route("/mission_info", methods=["POST", "GET"])
def mission_info():
    if request.method == "POST":
        pass
    # Do POST stuff
    elif request.method == "GET":
        if mission_info_dict["target"] != "":
            responds = {"target_name": mission_info_dict["target"],
                        "map_info": airsim_backend.objects}
            return responds
        else:
            return {"responds": "NO mission found"}


@inspection_page_bp.route("/update_backend/<object_name>", methods=["POST"])
def update_backend(object_name):
    wind_turbines = airsim_backend.objects
    if object_name in wind_turbines:
        wind_turbines[object_name]["state"] = request.form["state"]
        wind_turbines[object_name]["height"] = request.form["turbine_height"]
        wind_turbines[object_name]["blade_radius"] = request.form["blade_radius"]
        mission_info_dict["target"] = object_name
    return ("updated", 204)


@inspection_page_bp.route("/set_wind_direction", methods=["POST"])
def set_wind_direction():
    global wind_speed, wind_direction
    wind_direction = float(request.form["direction"])
    wind_speed = float(request.form['power'])
    print(wind_direction, wind_speed)
    success = airsim_backend.set_wind(wind_direction, wind_speed)
    print("request status: ", success)
    return {"responds": 200 if success else 404}


@inspection_page_bp.route("/flow_map", methods=["GET"])
def compute_flow_map():
    global wind_speed, wind_direction
    flow_map = windfarm.generate_flow_map(wind_speed=wind_speed,
                                          wind_direction=wind_direction, gen_flow_box=False)

    if not use_2D_mode:
        windfarm.generate_flow_map(wind_speed=wind_speed,
                                   wind_direction=wind_direction, gen_flow_box=True)
    wake_map, ti_map, flow_data, flow_data_blue = extract_maps(flow_map, 800)

    page_info = {
        'flow_map': convert_to_utf8(wake_map),
        'ti_map': convert_to_utf8(ti_map),
        'flow_data_blue': convert_to_utf8(flow_data_blue)
    }
    return page_info


def convert_to_utf8(img):
    retval, buffer = cv2.imencode('.png', img)
    img_as_text = base64.b64encode(buffer)
    return img_as_text.decode('utf-8')


@inspection_page_bp.route("/airsim_takeoff", methods=["GET"])
def takeoff():
    success = airsim_backend.takeoff()
    return {"responds": 200 if success else 404}


@inspection_page_bp.route("/set_turb_info", methods=["POST"])
def set_turb_info():
    global windfarm
    if "scalar" in request.form:
        windfarm.scalar = float(request.form["scalar"])
    if "mag" in request.form:
        windfarm.turb_mag_limit = float(request.form["mag"])

    print("set_turb_info value: ", request.form)
    return {"responds": 200}


@inspection_page_bp.route("/start_wind_sim", methods=["GET"])
def start_wind_sim():
    global wind_sim
    success = False
    if not wind_sim and not airsim_backend.offline_mode:
        wind_sim = True
        print("Wind sim is started")
        yourThread = threading.Timer(POOL_TIME, wind_sim_tread, ())
        yourThread.start()
        success = wind_sim
    return {"responds": 200 if success else 404}


@inspection_page_bp.route("/stop_wind_sim", methods=["GET"])
def stop_wind_sim():
    global wind_sim
    success = False
    if wind_sim and not airsim_backend.offline_mode:
        wind_sim = False
        print("Wind sim is stopped")
        success = not wind_sim
    return {"responds": 200 if success else 404}


def wind_sim_tread():
    global airsim_backend, drone_pos
    global yourThread, wind_sim
    global wind_direction, wind_speed, wind_ti, windfarm, wind_info_drone_pos, turb_img, turb_vector

    with dataLock:
        print("Update wind", flush=True)
        # Do your stuff with commonDataStruct Here
        x, y, z = airsim_backend.get_drone_position()
        x, y, z = airsim_to_canvas_xyz(x, y, z)
        drone_pos = (x, y, z)
        if x is None:
            print("Sim failed", flush=True)
            wind_sim = False

        ws, wd = windfarm.get_wind_at_pos(x, y, z, use_2d_mode=use_2D_mode)
        ws, wd = float(ws.squeeze()), float(wd.squeeze())
        wind_info_drone_pos = (ws, wd)
        turb_info_drone_pos, turb_mag, new_turb_img, wind_ti = windfarm.get_turbulence_at_pos(x, y, z,
                                                                                              use_2d_mode=use_2D_mode)
        turb_vector = tuple(np.round(turb_info_drone_pos, 3)) + (np.round(turb_mag, 2),)
        airsim_backend.set_wind(wd, ws, turb_info_drone_pos)
        turb_img = convert_to_utf8(new_turb_img)
        # Set the next thread to happen
        if wind_sim:
            yourThread = threading.Timer(POOL_TIME, wind_sim_tread, ())
            yourThread.start()
        else:
            print("Thread stopped", flush=True)
