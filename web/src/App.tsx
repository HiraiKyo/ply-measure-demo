import { ThemeProvider } from "@mui/material/styles";
import { CssBaseline } from "@mui/material";
import { theme } from "./theme/theme";
import { GlobalStyles } from "./theme/globalStyles";
import { Layout } from "./components/layout";
import "./App.css";
import { BrowserRouter, Route, Routes } from "react-router-dom";
import { HomePage } from "./pages/home";
import { RosTopicsPage } from "./pages/rostopics";
import { SettingsPage } from "./pages/settings";

export const eel = (window as any).eel;

function App() {
	return (
		<ThemeProvider theme={theme}>
			<CssBaseline />
			<GlobalStyles />
			<BrowserRouter>
				<Layout>
					<Routes>
						<Route path="/" element={<HomePage />} />
						<Route path="/topics" element={<RosTopicsPage />} />
						<Route path="/settings" element={<SettingsPage />} />
					</Routes>
				</Layout>
			</BrowserRouter>
		</ThemeProvider>
	);
}

export default App;
