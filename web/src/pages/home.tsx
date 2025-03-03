import { Box, styled } from "@mui/material";
import { MeasureView } from "../components/measure/view";
import { MeasureResults } from "../components/measure/results";
import { MeasureActions } from "../components/measure/actions";

const Container = styled(Box)({
	display: "flex",
	height: "calc(100vh - 32px)",
	backgroundColor: "#1e1e1e",
});

const MainContent = styled(Box)({
	flex: 1,
	display: "flex",
	flexDirection: "column",
	padding: "16px",
	gap: "16px",
});

export const HomePage = () => {
	return (
		<Container>
			<MainContent>
        <MeasureActions />
				<MeasureView />
				<MeasureResults />
			</MainContent>
		</Container>
	);
};
