import React, { useEffect, useState } from "react";
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
import { ROSSubscribeTopics } from "../../config/rostopics";
import { MeasureResult } from "../../types/measure";
import { useConfig } from "../../hooks/useConfig";

const ROS_EVENTS = (window as any).ROS_EVENTS;

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

const RGBColorToColor = (rgbColor: [number, number, number]) => {
	// rgbColor: [r, g, b], 0 <= r, g, b <= 1
	const r = Math.round(rgbColor[0] * 255);
	const g = Math.round(rgbColor[1] * 255);
	const b = Math.round(rgbColor[2] * 255);
	return `rgb(${r}, ${g}, ${b})`;
}

export const MeasureResults = () => {
	const { config } = useConfig();

	const topicName = "/measure/result";

	const [result, setResult] = useState<MeasureResult>();

	useEffect(() => {
		// ROSトピックの購読を設定
		const subscribeEvent = new CustomEvent(ROS_EVENTS.Subscribe, {
			detail: {
				name: ROSSubscribeTopics[topicName].name,
				type: ROSSubscribeTopics[topicName]["m-type"]
			},
		});
		document.dispatchEvent(subscribeEvent);

		// メッセージ受信時の処理
		const handleSubscribedValue = (e: CustomEvent) => {
			const { name, value } = e.detail;
			if (name === ROSSubscribeTopics[topicName].name && value?.data) {
				try {
					const result: MeasureResult = JSON.parse(value.data);
					setResult(result);
				} catch (error) {
					console.error("Failed to parse measure result:", error);
				}
			}
		};

		// イベントリスナーを登録
		document.addEventListener(
			ROS_EVENTS.SubscribedValue,
			handleSubscribedValue as EventListener
		);

		return () => {
			document.removeEventListener(
				ROS_EVENTS.SubscribedValue,
				handleSubscribedValue as EventListener
			);
		};
	}, []);

	return (
		<ResultsContainer>
			<TableContainer>
				<Table size="small">
					<TableHead>
						<TableRow>
							<StyledTableCell className="header">項目</StyledTableCell>
							<StyledTableCell className="header">中心座標</StyledTableCell>
							<StyledTableCell className="header">半径</StyledTableCell>
							<StyledTableCell className="header">ベクトル</StyledTableCell>
						</TableRow>
					</TableHead>
					<TableBody>
						{result && (
							<TableRow>
								<StyledTableCell>ピン中心</StyledTableCell>
								<StyledTableCell>&#40;{result?.center.join(",")}&#41;</StyledTableCell>
								<StyledTableCell>{result?.radius}</StyledTableCell>
								<StyledTableCell>&#40;{result?.normal.join(",")}&#41;</StyledTableCell>
							</TableRow>
						)}
						</TableBody>
				</Table>
				<Table size="small">
					<TableHead>
						<TableRow>
							<StyledTableCell className="header">項目</StyledTableCell>
							<StyledTableCell className="header">始点 - 終点座標</StyledTableCell>
							<StyledTableCell className="header">距離</StyledTableCell>
							<StyledTableCell className="header"></StyledTableCell>
						</TableRow>
					</TableHead>
					<TableBody>
						{result?.distances.map((distance, i) => (
							<TableRow key={`d-${i}`}>
								<StyledTableCell>
									<Box
										component="span"
										sx={{
											display: "inline-block",
											width: 12,
											height: 12,
											backgroundColor: config ? RGBColorToColor(config.RGB_TABLE[i]) : "gray",
											marginRight: 1,
											borderRadius: "2px",
										}}
									/>
									エッジ
								</StyledTableCell>
								<StyledTableCell>&#40;{distance.line_segment_points[0].join(",")}&#41; - &#40;{distance.line_segment_points[1].join(",")}&#41;</StyledTableCell>
								<StyledTableCell>{distance.distance}</StyledTableCell>
								<StyledTableCell></StyledTableCell>
							</TableRow>
						))}
					</TableBody>
				</Table>
			</TableContainer>
		</ResultsContainer>
	);
};
