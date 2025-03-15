import React from "react";
import { Box, IconButton } from "@mui/material";
import { styled } from "@mui/material/styles";
import HomeOutlinedIcon from "@mui/icons-material/HomeOutlined";
import SettingsOutlinedIcon from "@mui/icons-material/SettingsOutlined";
import { BlockOutlined, ListOutlined, TableViewOutlined, ViewInAr, ViewInArOutlined } from "@mui/icons-material";
import { useNavigate, useLocation } from "react-router-dom";

const SidebarContainer = styled(Box)({
	width: "48px",
	height: "100%",
	backgroundColor: "#333333",
	borderRight: "1px solid #404040",
	display: "flex",
	flexDirection: "column",
	alignItems: "center",
	padding: "8px 0",
});

const SidebarButton = styled(IconButton, {
	shouldForwardProp: (prop) => prop !== "active",
})<{ active?: boolean }>(({ active }) => ({
	width: "48px",
	height: "48px",
	borderRadius: 0,
	color: active ? "#ffffff" : "#858585",
	borderLeft: active ? "2px solid #007ACC" : "2px solid transparent",
	"&:hover": {
		backgroundColor: "#404040",
	},
}));

export function Sidebar() {
	const navigate = useNavigate();
	const { pathname } = useLocation();

	const handleItemClick = (route: string) => {
		navigate(route);
	};

	return (
		<SidebarContainer>
			<SidebarButton
				active={pathname === "/"}
				onClick={() => handleItemClick("/")}
			>
				<HomeOutlinedIcon />
			</SidebarButton>
			<SidebarButton
				active={pathname === "/stlview"}
				onClick={() => handleItemClick("/stlview")}
			>
				<ViewInArOutlined />
			</SidebarButton>
			<SidebarButton
				active={pathname === "/topics"}
				onClick={() => handleItemClick("/topics")}
			>
				<TableViewOutlined />
			</SidebarButton>
			<SidebarButton
				active={pathname === "/config"}
				onClick={() => handleItemClick("/config")}
			>
				<ListOutlined />
			</SidebarButton>
			<Box sx={{ flexGrow: 1 }} />
			<SidebarButton
				active={pathname === "/settings"}
				onClick={() => handleItemClick("/settings")}
			>
				<SettingsOutlinedIcon />
			</SidebarButton>
		</SidebarContainer>
	);
}
