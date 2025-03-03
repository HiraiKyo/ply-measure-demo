import React from "react";
import {
	Box,
	Paper,
	Table,
	TableBody,
	TableCell,
	TableContainer,
	TableHead,
	TableRow,
	styled,
} from "@mui/material";

const ResultsContainer = styled(Paper)({
	flex: 1,
	backgroundColor: "#2d2d2d",
	padding: "16px",
	borderRadius: "4px",
});

const StyledTableCell = styled(TableCell)({
	color: "#cccccc",
	borderBottom: "1px solid #404040",
	padding: "8px 16px",
	"&.header": {
		backgroundColor: "#333333",
		fontWeight: "bold",
	},
});

export const MeasureResults = () => {
	const measurements = [
		{ id: 1, color: "#ff0000", name: "中心軸A", x: 100.5, y: 200.3, z: 50.2 },
		{ id: 2, color: "#00ff00", name: "中心軸B", x: 150.7, y: 180.4, z: 45.8 },
	];

	return (
		<ResultsContainer>
			<TableContainer>
				<Table size="small">
					<TableHead>
						<TableRow>
							<StyledTableCell className="header">項目</StyledTableCell>
							<StyledTableCell className="header">X (mm)</StyledTableCell>
							<StyledTableCell className="header">Y (mm)</StyledTableCell>
							<StyledTableCell className="header">Z (mm)</StyledTableCell>
						</TableRow>
					</TableHead>
					<TableBody>
						{measurements.map((row) => (
							<TableRow key={row.id}>
								<StyledTableCell>
									<Box
										component="span"
										sx={{
											display: "inline-block",
											width: 12,
											height: 12,
											backgroundColor: row.color,
											marginRight: 1,
											borderRadius: "2px",
										}}
									/>
									{row.name}
								</StyledTableCell>
								<StyledTableCell>{row.x.toFixed(3)}</StyledTableCell>
								<StyledTableCell>{row.y.toFixed(3)}</StyledTableCell>
								<StyledTableCell>{row.z.toFixed(3)}</StyledTableCell>
							</TableRow>
						))}
					</TableBody>
				</Table>
			</TableContainer>
		</ResultsContainer>
	);
};
