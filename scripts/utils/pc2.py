from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import numpy as np
from numpy.typing import NDArray

def pc2_to_numpy(ros_point_cloud: PointCloud2) -> NDArray[np.floating]:
    """
    Convert a ROS PointCloud2 message to a NumPy array.
    """
    xyz = np.array([[0,0,0]])
    #self.lock.acquire()
    gen = pc2.read_points(ros_point_cloud, skip_nans=True)
    int_data = list(gen)

    for x in int_data:
        # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
    return xyz

def numpy_to_pc2(points: NDArray[np.floating]) -> PointCloud2:
    """
    Convert a NumPy array to a ROS PointCloud2 message.
    """
