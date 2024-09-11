from datastore import DataStore
from node import ROSNodeManager
from utils.config import Config
from gui import open_gui
from python_app_utils.log import Logger
import rospy

def main():
    # App Initialization
    Config()
    logger = Logger()
    logger.init(Config)
    DataStore()

    # Open ROS Node
    node = ROSNodeManager()

    open_gui()

if __name__ == "__main__":
    main()