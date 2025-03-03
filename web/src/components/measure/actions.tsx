import React from 'react';
import {
  Box,
  List, ListItem, Paper, Stack, styled,
  Typography
} from '@mui/material';
import { Send as SendIcon } from '@mui/icons-material';
import { PublisherButton } from '../common/ros/publisher/button';
import { ROSPublishTopics } from '../../config/rostopics';
import { eel } from '../../App';
import { CallEelFunctionButton } from '../common/eel/call-function';

export interface ActionItemProps {
  id: string;
  name: string;
  topic: string;
  message: any;
}

const StyledPaper = styled(Paper)(({ theme }) => ({
  width: '100%',
  backgroundColor: theme.palette.grey[900],
  padding: theme.spacing(1),
  borderRadius: theme.shape.borderRadius,
}));

const StyledList = styled(List)(({ theme }) => ({
  display: 'flex',
  flexDirection: 'row',
  gap: theme.spacing(1),
  padding: theme.spacing(0),
}));

const StyledListItem = styled(ListItem)({
  width: 'auto',
  flex: '0 0 auto',
  padding: 0,
  gap: 4,
});

const buttonSx = {
  minWidth: '200px',
  height: '100%',
  border: '1px solid',
  borderColor: 'grey.800',
  borderRadius: 1,
  px: 2,
  py: 1.5,
  transition: 'all 0.2s',
  '&:hover': {
    backgroundColor: 'grey.800',
    borderColor: 'grey.700',
    transform: 'translateY(-1px)',
  },
  '&:active': {
    transform: 'translateY(0)',
  }
};

const IconWrapper = styled(Box)(({ theme }) => ({
  display: 'flex',
  alignItems: 'center',
  gap: theme.spacing(1),
  color: theme.palette.primary.main,
  '& svg': {
    fontSize: '1.2rem',
  }
}));

export function MeasureActions() {
  return (
    <StyledPaper elevation={2}>
      <StyledList>
        <StyledListItem disablePadding>
          <PublisherButton
            key={ROSPublishTopics["/snapshot"].name}
            name={ROSPublishTopics["/snapshot"].name}
            messageValue={ROSPublishTopics["/snapshot"].initialValue}
            messageType={ROSPublishTopics["/snapshot"]['m-type']}
            variant="text"
            sx={buttonSx}
            >
            <ActionButtonContent
              mainText="3D点群撮影"
              subTexts={[
                'ROS Publisher',
                ROSPublishTopics["/snapshot"].name
              ]}
            />
          </PublisherButton>
          <CallEelFunctionButton
            callback={() => eel.open_filebrowser()()}
            onSuccess={(filepath) => console.log('Selected file:', filepath)}
            variant="text"
            sx={buttonSx}
          >
            <ActionButtonContent
              mainText="ファイル読込"
              subTexts={[
                'Python Function',
              ]}
            />
          </CallEelFunctionButton>
        </StyledListItem>
      </StyledList>
    </StyledPaper>
  );
}

interface ActionButtonContentProps {
  mainText: string;
  subTexts: string[];
}

const ActionButtonContent: React.FC<ActionButtonContentProps> = ({ mainText, subTexts }) => (
  <Stack alignItems="flex-start">
    <IconWrapper>
      <SendIcon />
      <Typography variant="body1" fontWeight="medium">
        {mainText}
      </Typography>
    </IconWrapper>
    {subTexts.map((text, index) => (
      <Typography
        key={index}
        variant="body2"
        color="text.secondary"
        sx={{
          opacity: 0.7,
        }}
        >
        {text}
      </Typography>
    ))}
  </Stack>
);