import React, { useState } from "react";
import { Box, Typography, TextField, Stack, Paper, Button } from "@mui/material";
import SaveIcon from '@mui/icons-material/Save';
import { ROSPublishTopics　} from "../../config/rostopics";
import { FeatureFlags } from "../../config/features";

export const PublisherList = () => {
  const [publishers, setPublishers] = useState(ROSPublishTopics);
  const [editingPublishers, setEditingPublishers] = useState(ROSPublishTopics);

  const handleChange = (key: string, field: string, value: string) => {
    setEditingPublishers(prev => ({
      ...prev,
      [key]: {
        ...prev[key],
        [field]: value
      }
    }));
  };

  const handleSave = (key: string) => {
    setPublishers(prev => ({
      ...prev,
      [key]: editingPublishers[key]
    }));
  };

  return (
    <Box sx={{ mb: 1 }}>
      <Typography variant="h6">
        Publishers
      </Typography>
      {Object.entries(editingPublishers).map(([key, topic]) => (
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
