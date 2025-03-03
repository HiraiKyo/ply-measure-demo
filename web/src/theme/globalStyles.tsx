import { Global } from "@emotion/react";

export const GlobalStyles = () => (
	<Global
		styles={{
			"@import":
				"url('https://fonts.googleapis.com/css2?family=Noto+Sans+JP:wght@400;700&family=Noto+Sans+Mono:wght@400;700&display=swap')",
			"html, body": {
				margin: 0,
				padding: 0,
				backgroundColor: "#1e1e1e",
				color: "#cccccc",
				fontFamily:
					"'Noto Sans JP', -apple-system, BlinkMacSystemFont, sans-serif",
			},
			a: {
				color: "inherit",
				textDecoration: "none",
				"&:hover": {
					textDecoration: "underline",
				},
			},
			".mono-16b": {
				fontFamily: "'Noto Sans Mono', monospace",
				fontSize: "1rem",
				lineHeight: "1.5rem",
				fontWeight: 700,
			},
			".mono-16n": {
				fontFamily: "'Noto Sans Mono', monospace",
				fontSize: "1rem",
				lineHeight: "1.5rem",
				fontWeight: 400,
			},
			// 必要最小限のリセットCSSとベーススタイル
		}}
	/>
);
