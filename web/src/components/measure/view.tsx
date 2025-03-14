import React from "react";
import { Paper, styled } from "@mui/material";
import { ROSSubscribeTopics } from "../../config/rostopics";
import { ROSSubscribeImage } from "../common/ros/subscriber/image";

const ViewContainer = styled(Paper)({
  flex: 2,
  backgroundColor: "#2d2d2d",
  padding: "16px",
  borderRadius: "4px",
  display: "flex",
  flexDirection: "column",
});

export const MeasureView = () => {
  return (
    <ViewContainer>
      <ROSSubscribeImage
        name={ROSSubscribeTopics["/measure/image"].name}
        messageType={ROSSubscribeTopics["/measure/image"]["m-type"]}
        maxWidth={1280}
        maxHeight="calc(100vh - 200px)"
      />
    </ViewContainer>
  );
};
