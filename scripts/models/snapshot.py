import open3d as o3d

def take_snapshot():
    """
    Take a snapshot by PhoXi Camera, passing ros package
    """
    pcd = o3d.io.read_point_cloud("models/sample.ply")
    # TODO: phoxi_cameraとの連携実装
    return pcd