import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2

def take_snapshot():
    """
    Take a snapshot by PhoXi Camera, passing ros package
    """
    pcd = o3d.io.read_point_cloud("models/capture.ply")
    # TODO: phoxi_cameraとの連携実装
    return pcd
