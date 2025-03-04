type Vector3 = [number, number, number];
type RGBColor = [number, number, number];

export interface CamPresets {
  default: Vector3;
  front: Vector3;
  top: Vector3;
  right: Vector3;
  left: Vector3;
  back: Vector3;
  bottom: Vector3;
  l45: Vector3;
  r45: Vector3;
  l135: Vector3;
  r135: Vector3;
}

export interface Config {
  PLACEHOLDER: string;
  CAM_FRONT: Vector3;
  CAM_PRESETS: CamPresets;
  CAM_ZOOM: number;
  ROS_SUB_TOPIC: string;
  ROS_PUB_TOPIC_RESULT: string;
  ROS_PUB_TOPIC_POINTCLOUD: string;
  ROS_PUB_TOPIC_IMAGE: string;
  RGB_TABLE: RGBColor[];
  BASE_PLANE_INDEX: number;
  MIL_PLANE_INDEX: number;
  CIRCLE_PLANE_INDEX: number;
  BASE_EXPECTED_EDGES: number;
  MIL_EXPECTED_EDGES: number;
  CONVEX_HULL_EPSILON: number;
  MIN_PLANE_POINTS: number;

  LOG_LEVEL: string;
  MODE: string;
}
