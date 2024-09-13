"""
PubSub Manager for ROS
"""

from python_app_utils.log import Logger
from python_app_utils.singleton import Singleton
import rospy
import datastore
import actions
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

dataStore = datastore.DataStore()
logger = Logger()

class ROSNodeManager(Singleton):
  def __init__(self):
    rospy.init_node("ply_measure_demo", anonymous=True)

    # Publishers
    self.pub_result = rospy.Publisher("/ply_measure_demo/result", String, queue_size=1)
    self.pub_pointcloud = rospy.Publisher("/ply_measure_demo/pointcloud", PointCloud2, queue_size=1)

    # Subscribers
    self.sub = rospy.Subscriber("/phoxi_camera/pointcloud", PointCloud2, actions.rossub_callback)

  def publish_result(self, msg):
    logger.info("Publishing Result")
    self.pub_result.publish(msg)

  def publish_pointcloud(self, msg):
    logger.info("Publishing PointCloud")
    self.pub_pointcloud.publish(msg)