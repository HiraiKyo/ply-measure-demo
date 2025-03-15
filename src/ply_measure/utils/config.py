import json
import pprint
from python_app_utils.config import ConfigBase

pp = pprint.PrettyPrinter(indent=2)

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
    [0.5, 1.0, 1.0],
    [0, 0, 0],
    [0.25, 0.25, 0.25],
    [0.5, 0.5, 0.5],
    [0.75, 0.75, 0.75]
]

class Config(ConfigBase):
    PLACEHOLDER = "placeholder"
    CAM_FRONT = cam_presets["l135"]
    CAM_UP = [0, 0, -1]
    CAM_ZOOM = 0.5
    ROS_SUB_TOPIC = "/sensors/capt_pc2"
    ROS_PUB_TOPIC_RESULT = "/ply_measure_demo/result"
    ROS_PUB_TOPIC_POINTCLOUD = "/ply_measure_demo/pointcloud"
    ROS_PUB_TOPIC_IMAGE = "/ply_measure_demo/image"
    RGB_TABLE = RGB_TABLE
    BASE_PLANE_INDEX = 0
    MIL_PLANE_INDEX = 2
    CIRCLE_PLANE_INDEX = 5
    BASE_EXPECTED_EDGES = 4
    MIL_EXPECTED_EDGES = 4
    CONVEX_HULL_EPSILON = 5
    MIN_PLANE_POINTS = 500

    def to_json(self):
        return {
            "CAM_FRONT": self.CAM_FRONT,
            "CAM_ZOOM": self.CAM_ZOOM,
            "CAM_UP": self.CAM_UP,
            "ROS_SUB_TOPIC": self.ROS_SUB_TOPIC,
            "ROS_PUB_TOPIC_RESULT": self.ROS_PUB_TOPIC_RESULT,
            "ROS_PUB_TOPIC_POINTCLOUD": self.ROS_PUB_TOPIC_POINTCLOUD,
            "ROS_PUB_TOPIC_IMAGE": self.ROS_PUB_TOPIC_IMAGE,
            "RGB_TABLE": self.RGB_TABLE,
            "BASE_PLANE_INDEX": self.BASE_PLANE_INDEX,
            "MIL_PLANE_INDEX": self.MIL_PLANE_INDEX,
            "CIRCLE_PLANE_INDEX": self.CIRCLE_PLANE_INDEX,
            "BASE_EXPECTED_EDGES": self.BASE_EXPECTED_EDGES,
            "MIL_EXPECTED_EDGES": self.MIL_EXPECTED_EDGES,
            "CONVEX_HULL_EPSILON": self.CONVEX_HULL_EPSILON,
            "MIN_PLANE_POINTS": self.MIN_PLANE_POINTS,
            "LOG_LEVEL": self.LOG_LEVEL,
            "MODE": self.MODE
        }

    def update(self, json_string):
        jsonObject = json.loads(json_string)
        pp.pprint(jsonObject)

        for key in jsonObject.keys():
            if key in self.__dict__.keys():
                setattr(self, key, jsonObject[key])