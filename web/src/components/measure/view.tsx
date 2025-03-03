import React from "react";
import { Box, Paper, styled } from "@mui/material";

const ViewContainer = styled(Paper)({
	flex: 2,
	backgroundColor: "#2d2d2d",
	padding: "16px",
	borderRadius: "4px",
});

const Canvas = styled(Box)({
	width: "100%",
	height: "100%",
	backgroundColor: "#1e1e1e",
	border: "1px solid #404040",
	borderRadius: "2px",
});

export const MeasureView = () => {
	return (
		<ViewContainer>
			<Canvas id="measure-view" />
		</ViewContainer>
	);
};
