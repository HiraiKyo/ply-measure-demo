import React, { useState } from "react";
import { Box, Typography, TextField, Stack, Paper, Button } from "@mui/material";
import SaveIcon from '@mui/icons-material/Save';
import { ROSSubscribeTopics } from "../../config/rostopics";
import { FeatureFlags } from "../../config/features";

export const SubscriberList = () => {
  const [subscribers, setSubscribers] = useState(ROSSubscribeTopics);
  const [editingSubscribers, setEditingSubscribers] = useState(ROSSubscribeTopics);

  const handleChange = (key: string, field: string, value: string) => {
    setEditingSubscribers(prev => ({
      ...prev,
      [key]: {
        ...prev[key],
        [field]: value
      }
    }));
  };

  const handleSave = (key: string) => {
    setSubscribers(prev => ({
      ...prev,
      [key]: editingSubscribers[key]
    }));
  };

  return (
    <Box sx={{ mb: 1 }}>
      <Typography variant="h6">
        Subscribers
      </Typography>
      {Object.entries(ROSSubscribeTopics).map(([key, topic]) => (
        <Paper key={key} sx={{ p: 2, mb: 1 }}>
          <Stack spacing={1}>
            <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <Typography variant="subtitle1">
                Alias: {key}
              </Typography>
              {FeatureFlags.enableRosTopicEdit && (
                <Button
                  variant="contained"
                  size="small"
                  startIcon={<SaveIcon />}
                  onClick={() => handleSave(key)}
                >
                  Save
                </Button>
              )}
            </Box>
            <Stack direction={{ xs: 'column', md: 'row' }} spacing={2}>
              <TextField
                fullWidth
                label="Topic Name"
                value={topic.name}
                onChange={(e) => handleChange(key, 'name', e.target.value)}
                variant="outlined"
                size="small"
                disabled={!FeatureFlags.enableRosTopicEdit}
              />
              <TextField
                fullWidth
                label="Message Type"
                value={topic["m-type"]}
                onChange={(e) => handleChange(key, 'm-type', e.target.value)}
                variant="outlined"
                size="small"
                disabled={!FeatureFlags.enableRosTopicEdit}
              />
              <TextField
                fullWidth
                label="Initial Value"
                value={topic.initialValue}
                onChange={(e) => handleChange(key, 'initialValue', e.target.value)}
                variant="outlined"
                size="small"
                disabled={!FeatureFlags.enableRosTopicEdit}
              />
            </Stack>
          </Stack>
        </Paper>
      ))}
    </Box>
  );
};
