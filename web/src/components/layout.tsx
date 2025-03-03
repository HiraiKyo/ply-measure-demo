import React from "react";
import { Box } from "@mui/material";
import { styled } from "@mui/material/styles";
import { Header } from "./header";
import { Sidebar } from "./sidebar";
import { StatusBar } from "./statusbar";

const Container = styled(Box)({
	display: "flex",
	flexDirection: "column",
	height: "100vh",
	backgroundColor: "#1e1e1e",
	color: "#cccccc",
});

const Main = styled(Box)({
	display: "flex",
	flex: 1,
	overflow: "hidden",
});

const Content = styled(Box)({
	flex: 1,
	display: "flex",
	flexDirection: "column",
	backgroundColor: "#252526",
});

export function Layout({ children }: { children: React.ReactNode }) {
	return (
		<Container>
			<Header />
			<Main>
				<Sidebar />
				<Content>{children}</Content>
			</Main>
			<StatusBar />
		</Container>
	);
}
