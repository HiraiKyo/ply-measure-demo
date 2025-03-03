import { createTheme } from "@mui/material/styles";
import { colors } from "./colors";

export const theme = createTheme({
	palette: {
		mode: "dark",
		primary: {
			main: colors.brand.primary,
			light: colors.brand.secondary,
			dark: colors.brand.tertiary,
		},
		secondary: {
			main: colors.sea[800],
			light: colors.sea[600],
			dark: colors.sea[900],
		},
		error: {
			main: colors.sun[800],
			light: colors.sun[600],
			dark: colors.sun[900],
		},
		warning: {
			main: colors.wood[800],
			light: colors.wood[600],
			dark: colors.wood[900],
		},
		success: {
			main: colors.forest[600],
			light: colors.forest[400],
			dark: colors.forest[800],
		},
		grey: colors.gray,
		background: {
			default: colors.gray[900],
			paper: colors.gray[800],
		},
		text: {
			primary: colors.gray[200],
			secondary: colors.gray[400],
			disabled: colors.gray[600],
		},
		action: {
			active: colors.sea[300],
			hover: colors.sea[200],
			selected: colors.sea[400],
			disabled: colors.gray[600],
		},
	},
	typography: {
		fontFamily: "'Noto Sans JP', -apple-system, BlinkMacSystemFont, sans-serif",
		fontSize: 14,
		fontWeightLight: 400,
		fontWeightRegular: 400,
		fontWeightMedium: 700,
		fontWeightBold: 700,
		h1: {
			fontSize: "3rem", // 48px
			lineHeight: "4.2rem", // 1.4
			fontWeight: 700,
			letterSpacing: "0.04rem",
		},
		h2: {
			fontSize: "1.5rem", // 24px
			lineHeight: "2.1rem", // 1.4
			fontWeight: 700,
			letterSpacing: "0.04rem",
		},
		h3: {
			fontSize: "1.25rem", // 20px
			lineHeight: "1.75rem", // 1.4
			fontWeight: 700,
			letterSpacing: "0.04rem",
		},
		h4: {
			fontSize: "1rem", // 16px
			lineHeight: "1.4rem", // 1.4
			fontWeight: 700,
			letterSpacing: "0.04rem",
		},
		h5: {
			fontSize: "0.875rem", // 14px
			lineHeight: "1.2rem", // 1.4
			fontWeight: 700,
			letterSpacing: "0.04rem",
		},
		body1: {
			fontSize: "1rem", // std-16n-170
			lineHeight: "1.7rem",
			fontWeight: 400,
			letterSpacing: "0.04rem",
		},
		body2: {
			fontSize: "0.875rem", // dns-14n-120
			lineHeight: "1.2rem",
			fontWeight: 400,
			letterSpacing: "0.04rem",
		},
		subtitle1: {
			fontSize: "1rem", // dns-16b-120
			lineHeight: "1.2rem",
			fontWeight: 700,
			letterSpacing: "0.04rem",
		},
		subtitle2: {
			fontSize: "0.875rem", // dns-14b-120
			lineHeight: "1.2rem",
			fontWeight: 700,
			letterSpacing: "0.04rem",
		},
		button: {
			fontSize: "1rem", // oln-16b-100
			lineHeight: "1rem",
			fontWeight: 700,
			letterSpacing: "0.04rem",
			textTransform: "none",
		},
		caption: {
			fontSize: "0.875rem",
			lineHeight: "1.2rem",
			fontWeight: 400,
			letterSpacing: "0.04rem",
		},
		overline: {
			fontSize: "0.75rem",
			lineHeight: "1rem",
			fontWeight: 400,
			letterSpacing: "0.04rem",
		},
	},
	components: {
		MuiButton: {
			styleOverrides: {
				root: {
					textTransform: "none",
					minWidth: "96px",
					minHeight: "48px",
				},
			},
		},
		MuiInput: {
			styleOverrides: {
				root: {
					fontSize: "0.875rem",
					lineHeight: "1.2rem",
				},
			},
		},
		MuiTypography: {
			defaultProps: {
				variantMapping: {
					subtitle1: "p",
					subtitle2: "p",
					body1: "p",
					body2: "p",
				},
			},
		},
	},
});

// カスタムタイポグラフィバリアントの型定義
declare module "@mui/material/styles" {
	interface TypographyVariants {
		"mono-16b": React.CSSProperties;
		"mono-16n": React.CSSProperties;
	}

	interface TypographyVariantsOptions {
		"mono-16b"?: React.CSSProperties;
		"mono-16n"?: React.CSSProperties;
	}
}

declare module "@mui/material/Typography" {
	interface TypographyPropsVariantOverrides {
		"mono-16b": true;
		"mono-16n": true;
	}
}
