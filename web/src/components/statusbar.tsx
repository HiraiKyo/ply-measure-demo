import { Paper } from "@mui/material";
import { styled } from "@mui/material/styles";

export const StyledPaper = styled(Paper)({
	height: "22px",
	backgroundColor: "#007acc",
	color: "white",
	fontSize: "12px",
	display: "flex",
	alignItems: "center",
	padding: "0 8px",
	boxShadow: "none",
	borderRadius: 0,
});

export const StatusBar = () => {
	return <StyledPaper>ファイル: 1/1</StyledPaper>;
};
