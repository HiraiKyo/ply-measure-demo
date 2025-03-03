export interface DistanceSet {
  distance: number;
  line_segment_points: [number[], number[]];
  group: number;
  image_path: string | null;
}

export interface MeasureResult {
  center: number[];
  radius: number;
  normal: number[];
  distances: DistanceSet[];
  plane_indices: number[];
}
