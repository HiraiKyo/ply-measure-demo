globalThis.ROS_EVENTS = {
  Publish: "Publish", // { name, type, value },
  Subscribe: "Subscribe", // { name, type },
  SubscribedValue: "SubscribedValue", // { name, value },
  Param: "Param", // { name },
  ParamSet: "ParamSet", // { name, type, value },
  ParamUpdated: "ParamUpdated", // { name, value },
}

document.addEventListener("DOMContentLoaded", async () => {
  /**
   * ROS PUBLISHER
   * カスタムイベントPublishEvent を購読して、ROSにPublishする
   **/
  document.addEventListener(ROS_EVENTS.Publish, async (e) => {
    const { name, type, value } = e.detail;
    console.debug(`[CA] Publish: ${name} (${type}, ${value})`);
    eel.ros_publish(name, type, value);
  });

  /**
   * ROS SUBSCRIBER
   * カスタムイベントSubscribeEvent を購読して、ROSにSubscribeする
   * サブスクライバーが受信したデータをカスタムイベントSubscribedValue で通知する
   **/
  document.addEventListener(ROS_EVENTS.Subscribe, async (e) => {
    const { name, type } = e.detail;
    console.debug(`[CA] Subscribe: ${name} (${type})`);
    eel.ros_subscribe(name, type);
  });

  /**
   * ROS Parameters
   * カスタムイベントParamEvent を購読して、ROSにパラメータを登録する
   **/
  document.addEventListener(ROS_EVENTS.Param, async (e) => {
    const { name } = e.detail;
    console.debug(`[CA] Param: ${name}`);
    eel.ros_register_param(name);
  });

  /**
   * ROS Parameters Setter
   * カスタムイベントParamSet を購読して、ROSのパラメータを設定
   */
  document.addEventListener(ROS_EVENTS.ParamSet, async (e) => {
    const { name, type, value } = e.detail;
    console.debug(`[CA] ParamSet: ${name} (${type}, ${value})`);
    eel.ros_set_param(name, type, value);
  });

  /**
   * ヘルスチェック
  */
  eel.health(document.title);
});


globalThis.ROSInterface = {
  getParam: async function(name) {
    return await eel.ros_get_param(name);
  }
}

function updateSubscribedValue(name, value) {
  const event = new CustomEvent(ROS_EVENTS.SubscribedValue, {
    detail: { name, value },
  });
  document.dispatchEvent(event);
};
eel.expose(updateSubscribedValue);


function updateParam(name, value) {
  console.log(name, value);
  const event = new CustomEvent(ROS_EVENTS.ParamUpdated, {
    detail: { name, value },
  });
  document.dispatchEvent(event);
};
eel.expose(updateParam);

function health(value) {
  console.log("[CA] Python -> JS: OK");
  return document.title + ": " + value;
};
eel.expose(health);