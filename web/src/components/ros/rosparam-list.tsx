import React, { useState } from "react";
import { Box, Typography, TextField, Stack, Paper, Button } from "@mui/material";
import SaveIcon from '@mui/icons-material/Save';
import { ROSParamTopics } from "../../config/rostopics";
import { FeatureFlags } from "../../config/features";

export const RosParamList = () => {
  const [params, setParams] = useState(ROSParamTopics);
  const [editingParams, setEditingParams] = useState(ROSParamTopics);

  const handleChange = (key: string, field: string, value: string) => {
    setEditingParams(prev => ({
      ...prev,
      [key]: {
        ...prev[key],
        [field]: value
      }
    }));
  };

  const handleSave = (key: string) => {
    setParams(prev => ({
      ...prev,
      [key]: editingParams[key]
    }));
  };

  return (
    <Box sx={{ mb: 1 }}>
      <Typography variant="h6">
        ROS Parameters
      </Typography>
      {Object.entries(params).map(([key, topic]) => (
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
                label="Parameter Name"
                value={topic.name}
                variant="outlined"
                size="small"
                onChange={(e) => handleChange(key, 'name', e.target.value)}
                disabled={!FeatureFlags.enableRosTopicEdit}
              />
              <TextField
                fullWidth
                label="Message Type"
                value={topic["m-type"]}
                variant="outlined"
                size="small"
                onChange={(e) => handleChange(key, 'm-type', e.target.value)}
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
