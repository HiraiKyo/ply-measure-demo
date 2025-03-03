import React, { useEffect, useState } from "react";
import { Box, ImageListItem } from "@mui/material";
import { styled } from "@mui/material/styles";

const ROS_EVENTS = (window as any).ROS_EVENTS;

interface ROSSubscribeImageProps {
  name: string;
  messageType?: string;
  width?: number;
  height?: number;
}

const ImageContainer = styled(Box)(({ theme }) => ({
  width: "100%",
  height: "100%",
  display: "flex",
  justifyContent: "center",
  alignItems: "center",
  backgroundColor: theme.palette.background.paper,
  borderRadius: theme.shape.borderRadius,
  overflow: "hidden",
}));

const StyledImageListItem = styled(ImageListItem)(({ theme }) => ({
  "&:hover": {
    "& img": {
      transform: "scale(1.02)",
      transition: "transform 0.3s ease-in-out",
    },
  },
  "& img": {
    transition: "transform 0.3s ease-in-out",
    objectFit: "contain",
  },
}));

export const ROSSubscribeImage: React.FC<ROSSubscribeImageProps> = ({
  name,
  messageType = "sensor_msgs.msg.Image",
  width = 640,
  height = 480
}) => {
  const [imageData, setImageData] = useState<string>("");

  useEffect(() => {
    // ROSにSubscribeを登録
    const subscribeEvent = new CustomEvent(ROS_EVENTS.Subscribe, {
      detail: { name, type: messageType },
    });
    document.dispatchEvent(subscribeEvent);

    // 画像データ受信時の処理
    const handleSubscribedValue = (e: CustomEvent) => {
      const { name: topicName, value } = e.detail;
      if (topicName === name && value?.data) {
        setImageData(value.data);
      }
    };

    // イベントリスナーを登録
    document.addEventListener(
      ROS_EVENTS.SubscribedValue,
      handleSubscribedValue as EventListener
    );

    // クリーンアップ関数
    return () => {
      document.removeEventListener(
        ROS_EVENTS.SubscribedValue,
        handleSubscribedValue as EventListener
      );
    };
  }, [name, messageType]);

  return (
    <ImageContainer>
      <StyledImageListItem sx={{ width, height }}>
        <img
          src={imageData || "placeholder.png"}
          alt="ROS Topic"
          loading="lazy"
          style={{
            width: "100%",
            height: "100%",
          }}
        />
      </StyledImageListItem>
    </ImageContainer>
  );
};