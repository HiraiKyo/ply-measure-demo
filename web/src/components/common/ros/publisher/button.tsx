import React from 'react';
import { Button, ButtonProps } from '@mui/material';

type MessageType = 'std_msgs.msg.Bool' | 'std_msgs.msg.Int32' | 'std_msgs.msg.String';

interface PublisherProps extends ButtonProps {
  name: string;
  messageType: string | MessageType;
  messageValue: boolean | number | string;
  initialValue?: boolean | number | string;
}

const ROS_EVENTS = {
  Publish: 'Publish'
} as const;

export const PublisherButton: React.FC<PublisherProps> = ({
  name,
  messageValue,
  messageType,
  children, ...props
}) => {
  const handleClick = () => {
    const event = new CustomEvent(ROS_EVENTS.Publish, {
      detail: {
        name,
        type: messageType,
        value: { data: messageValue }
      }
    });
    document.dispatchEvent(event);
  };

  return (
    <Button
      onClick={handleClick}
      size="medium"
      {...props}
    >
      {children}
    </Button>
  );
};
