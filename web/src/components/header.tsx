import React from "react";
import {
	AppBar,
	Toolbar,
	Typography,
	IconButton,
	Menu,
	MenuItem,
} from "@mui/material";
import { styled } from "@mui/material/styles";
import MenuIcon from "@mui/icons-material/Menu";

const StyledAppBar = styled(AppBar)({
	backgroundColor: "#2d2d2d",
	position: "static",
	minHeight: "32px",
	boxShadow: "none",
	borderBottom: "1px solid #404040",
});

const StyledToolbar = styled(Toolbar)({
	minHeight: "32px !important",
	padding: "0 8px",
	display: "flex",
	gap: "16px",
	"@media (min-width: 600px)": {
		minHeight: "32px !important", // モバイルブレークポイント以上でも32pxを維持
	},
});

const StyledTypography = styled(Typography)({
	fontSize: "12px",
	color: "#cccccc",
});

const MenuButton = styled(IconButton)({
	padding: 4,
	marginRight: 8,
	color: "#cccccc",
	"&:hover": {
		backgroundColor: "#404040",
	},
});

export function Header() {
	const [anchorEl, setAnchorEl] = React.useState<null | HTMLElement>(null);

	const handleMenuClick = (event: React.MouseEvent<HTMLButtonElement>) => {
		setAnchorEl(event.currentTarget);
	};

	const handleClose = () => {
		setAnchorEl(null);
	};

	return (
		<StyledAppBar>
			<StyledToolbar>
				<MenuButton size="small" edge="start" onClick={handleMenuClick}>
					<MenuIcon fontSize="small" />
				</MenuButton>
				<StyledTypography>3D点群処理アプリケーション(デモ)</StyledTypography>
				<Menu
					anchorEl={anchorEl}
					open={Boolean(anchorEl)}
					onClose={handleClose}
				>
					<MenuItem onClick={handleClose}>ヘルプ</MenuItem>
				</Menu>
			</StyledToolbar>
		</StyledAppBar>
	);
}
