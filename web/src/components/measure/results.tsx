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

export const MeasureResults = () => {
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
							<StyledTableCell className="header">座標</StyledTableCell>
							<StyledTableCell className="header">半径 or 距離</StyledTableCell>
							<StyledTableCell className="header">ベクトル</StyledTableCell>
						</TableRow>
					</TableHead>
					<TableBody>
						{result && (
							<TableRow>
								<StyledTableCell>ピン中心</StyledTableCell>
								<StyledTableCell>{result?.center.join(",")}</StyledTableCell>
								<StyledTableCell>{result?.radius}</StyledTableCell>
								<StyledTableCell>{result?.normal.join(",")}</StyledTableCell>
							</TableRow>
						)}
						{result?.distances.map((distance, i) => (
							<TableRow key={`d-${i}`}>
								<StyledTableCell>
									<Box
										component="span"
										sx={{
											display: "inline-block",
											width: 12,
											height: 12,
											backgroundColor: "red",
											marginRight: 1,
											borderRadius: "2px",
										}}
									/>
									エッジ
								</StyledTableCell>
								<StyledTableCell>{distance.line_segment_points[0].join(",")} - {distance.line_segment_points[1].join(",")}</StyledTableCell>
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
