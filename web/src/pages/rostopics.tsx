import React from "react";
import { Box } from "@mui/material";
import { PublisherList } from "../components/ros/publisher-list";
import { SubscriberList } from "../components/ros/subscriber-list";
import { RosParamList } from "../components/ros/rosparam-list";

export const RosTopicsPage = () => {
  return (
    <Box sx={{ p: 3 }}>
      <PublisherList />
      <SubscriberList />
      <RosParamList />
    </Box>
  );
};
