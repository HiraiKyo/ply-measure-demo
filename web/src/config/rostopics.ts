export interface RosPublishTopic {
  name: string;
  "m-type": string;
  initialValue?: any;
};

interface RosPublishTopicJson {
  [key: string]: RosPublishTopic
}

export const ROSPublishTopics: RosPublishTopicJson = {
  "/snapshot": {
    name: "/ply-process/measure/start",
    "m-type": "std_msgs.msg.Bool",
    initialValue: true
  },
}

export interface RosSubscribeTopic {
  name: string;
  "m-type": string;
  initialValue?: any;
};

interface RosSubscribeTopicJson {
  [key: string]: RosSubscribeTopic
}

export const ROSSubscribeTopics: RosSubscribeTopicJson = {
  "/measure/result": {
    name: "/ply_measure_demo/result",
    "m-type": "std_msgs.msg.String",
  },
  "/measure/pointcloud": {
    name: "/ply_measure_demo/pointcloud",
    "m-type": "sensor_msgs.msg.PointCloud2",
  },
  "/measure/image": {
    name: "/ply_measure_demo/image",
    "m-type": "sensor_msgs.msg.Image",
  },
}

export interface RosParamTopic {
  name: string;
  "m-type": string;
  initialValue?: any;
}

interface RosParamTopicJson {
  [key: string]: RosParamTopic
}

export const ROSParamTopics: RosParamTopicJson = {
  "/measure/param": {
    name: "/ply-process/measure/param",
    "m-type": "std_msgs.msg.String",
  }
}