"""
PubSub Manager for ROS
"""

from python_app_utils.log import Logger
from python_app_utils.singleton import Singleton
import rospy
from datastore import DataStore
from actions import rossub_callback
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

dataStore = DataStore()
logger = Logger()

class ROSNodeManager(Singleton):
  def __init__(self):
    rospy.init_node("ply_measure_demo", anonymous=True)

    # Publishers
    self.pub = rospy.Publisher("/ply_measure_demo/result", String, queue_size=1)

    # Subscribers
    self.sub = rospy.Subscriber("/phoxi_camera/pointcloud", PointCloud2, rossub_callback)

    rospy.spin()

  def publish(self, json_str: str):
    self.pub.publish(str)