import json
import pprint
from python_app_utils.singleton import Singleton

pp = pprint.PrettyPrinter(indent=2)

class Settings(Singleton):
    def __init__(self):
        super().__init__()
        self.MODE = "ubuntu"

    def to_json(self):
        return {
            "MODE": self.MODE
        }

    def update(self, json_string):
        jsonObject = json.loads(json_string)
        pp.pprint(jsonObject)

        for key in jsonObject.keys():
            if key in self.__dict__.keys():
                setattr(self, key, jsonObject[key])