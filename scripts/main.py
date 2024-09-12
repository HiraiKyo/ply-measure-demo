import datastore
import rosnode
from utils import config
import gui
from python_app_utils.log import Logger
import rospy

def main():
    # App Initialization
    config.Config()
    logger = Logger()
    logger.init(config.Config)
    datastore.DataStore()

    # Open ROS Node
    rosnode.ROSNodeManager()

    gui.open_gui()

if __name__ == "__main__":
    main()