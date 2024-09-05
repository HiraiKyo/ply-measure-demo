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


class Config(ConfigBase):
    PLACEHOLDER = "placeholder"
    CAM_FRONT = cam_presets["default"]
    CAM_ZOOM = 0.2