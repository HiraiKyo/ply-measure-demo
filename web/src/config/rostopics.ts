export interface RosPublishTopic {
  name: string;
  "m-type": string;
  initialValue?: any;
};

interface RosPublishTopicJson {
  [key: string]: RosPublishTopic
}

export const ROSPublishTopics: RosPublishTopicJson = {
  "/measure/start": {
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
  "/measure/results": {
    name: "/ply-process/measure/results",
    "m-type": "std_msgs.msg.String",
  }
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