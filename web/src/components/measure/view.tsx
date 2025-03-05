import React from "react";
import { Paper, styled } from "@mui/material";
import { ROSSubscribeTopics } from "../../config/rostopics";
import { ROSSubscribeImage } from "../common/ros/subscriber/image";

const ViewContainer = styled(Paper)({
	flex: 2,
	backgroundColor: "#2d2d2d",
	padding: "16px",
	borderRadius: "4px",
});

export const MeasureView = () => {
	return (
		<ViewContainer>
			<ROSSubscribeImage
				name={ROSSubscribeTopics["/measure/image"].name}
				messageType={ROSSubscribeTopics["/measure/image"]["m-type"]}
				width={1280}
				height={960}
			/>
		</ViewContainer>
	);
};
