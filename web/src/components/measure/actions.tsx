import React from 'react';
import {
  Box,
  List, ListItem, Paper, Stack, styled,
  Typography
} from '@mui/material';
import { Send as SendIcon } from '@mui/icons-material';
import { PublisherButton } from '../common/ros/publisher/button';
import { ROSPublishTopics } from '../../config/rostopics';

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
});

const buttonSx = {
  minWidth: '200px',
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
  const actions = [
    {
      ...ROSPublishTopics["/measure/start"],
      label: "ピン計測開始",
    }
  ]

  return (
    <StyledPaper elevation={2}>
      <StyledList>
        <StyledListItem disablePadding>
          {actions.map((action) => (
            <PublisherButton
              key={action.name}
              name={action.name}
              messageValue={action.initialValue}
              messageType={action['m-type']}
              variant="text"
              sx={buttonSx}
            >
              <Stack alignItems="flex-start">
                <IconWrapper>
                  <SendIcon />
                  <Typography variant="body1" fontWeight="medium">
                    {action.label}
                  </Typography>
                </IconWrapper>
                <Typography
                  variant="caption"
                  color="text.secondary"
                  sx={{
                    opacity: 0.7,
                  }}
                >
                  {action.name}
                </Typography>
              </Stack>
            </PublisherButton>
          ))}
        </StyledListItem>
      </StyledList>
    </StyledPaper>
  );
}
