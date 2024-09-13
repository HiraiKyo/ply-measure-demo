from python_app_utils.config import ConfigBase


cam_presets = {
    "default": [-1, -1, 1],
    "front": [0, 0, -1],
    "top": [0, -1, 0],
    "right": [-1, 0, 0],
    "left": [1, 0, 0],
    "back": [0, 0, 1],
    "bottom": [0, 1, 0],
    "l45": [1, -1, 1],
    "r45": [-1, -1, 1],
    "l135": [1, -1, -1],
    "r135": [-1, -1, -1],
}

RGB_TABLE=[
    [1.0, 0.0, 0.0], # red
    [0.0, 1.0, 0.0], # green
    [0.0, 0.0, 1.0], # blue
    [1.0, 1.0, 0.0], # yellow
    [1.0, 0.0, 1.0], # purple
    [0.0, 1.0, 1.0], # cyan
    [1.0, 0.5, 0.5], # orange
    [0.5, 1.0, 0.5],
    [0.5, 0.5, 1.0],
    [1.0, 1.0, 0.5],
    [1.0, 0.5, 1.0],
    [0.5, 1.0, 1.0]
]

ROS_SUB_TOPIC_SIM = "/sensors/capt_pc2"

class Config(ConfigBase):
    PLACEHOLDER = "placeholder"
    CAM_FRONT = cam_presets["default"]
    CAM_ZOOM = 0.2
    ROS_SUB_TOPIC = "/phoxi_camera/pointcloud"
    # ROS_SUB_TOPIC = ROS_SUB_TOPIC_SIM
    ROS_PUB_TOPIC_RESULT = "/ply_measure_demo/result"
    ROS_PUB_TOPIC_POINTCLOUD = "/ply_measure_demo/pointcloud"
    RGB_TABLE = RGB_TABLE