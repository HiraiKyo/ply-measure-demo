import React, { useEffect } from 'react';
import {
  Box,
  TextField,
  Button,
  Typography,
  CircularProgress,
  Alert,
  Paper,
} from '@mui/material';
import { Config } from '../types/config';
import { eel } from '../App';

export const ConfigPage = () => {
  const [config, setConfig] = React.useState<Config | null>(null);
  const [loading, setLoading] = React.useState(true);
  const [error, setError] = React.useState<string | null>(null);

  useEffect(() => {
    (async () => {
      try {
        const json = await eel.read_config()()
        setConfig(json);
        setLoading(false);
      } catch (error) {
        setError('設定の読み込みに失敗しました');
        setLoading(false);
      }
    })();
  }, []);

  if (loading) {
    return (
      <Box sx={{ p: 3, display: 'flex', justifyContent: 'center' }}>
        <CircularProgress />
      </Box>
    );
  }

  if (!config) {
    return (
      <Box sx={{ p: 3 }}>
        <Alert severity="error">設定の読み込みに失敗しました</Alert>
      </Box>
    );
  }

  const handleSubmit = async (event: React.FormEvent) => {
    event.preventDefault();
    const formData = new FormData(event.target as HTMLFormElement);
    
    try {
      const newConfig = {
        ...config,
        CAM_FRONT: JSON.parse(formData.get('CAM_FRONT') as string),
        CAM_ZOOM: parseFloat(formData.get('CAM_ZOOM') as string),
        ROS_SUB_TOPIC: formData.get('ROS_SUB_TOPIC') as string,
        ROS_PUB_TOPIC_RESULT: formData.get('ROS_PUB_TOPIC_RESULT') as string,
        ROS_PUB_TOPIC_POINTCLOUD: formData.get('ROS_PUB_TOPIC_POINTCLOUD') as string,
        ROS_PUB_TOPIC_IMAGE: formData.get('ROS_PUB_TOPIC_IMAGE') as string,
        BASE_PLANE_INDEX: parseInt(formData.get('BASE_PLANE_INDEX') as string),
        MIL_PLANE_INDEX: parseInt(formData.get('MIL_PLANE_INDEX') as string),
        CIRCLE_PLANE_INDEX: parseInt(formData.get('CIRCLE_PLANE_INDEX') as string),
        BASE_EXPECTED_EDGES: parseInt(formData.get('BASE_EXPECTED_EDGES') as string),
        MIL_EXPECTED_EDGES: parseInt(formData.get('MIL_EXPECTED_EDGES') as string),
        CONVEX_HULL_EPSILON: parseInt(formData.get('CONVEX_HULL_EPSILON') as string),
        MIN_PLANE_POINTS: parseInt(formData.get('MIN_PLANE_POINTS') as string),
        LOG_LEVEL: formData.get('LOG_LEVEL') as string,
        MODE: formData.get('MODE') as string,
      };

      const success = await eel.update_config(newConfig)();
      if (success) {
        alert('設定を更新しました');
        // 設定を再読み込み
        const updatedConfig = await eel.read_config()();
        setConfig(updatedConfig);
      }
    } catch (error) {
      setError('設定の更新に失敗しました');
    }
  };

  return (
    <Box sx={{ p: 3 }}>
      <Typography variant="h5" sx={{ mb: 3 }}>設定</Typography>
      {error && <Alert severity="error" sx={{ mb: 2 }}>{error}</Alert>}
      <Paper sx={{ p: 3 }}>
        <form onSubmit={handleSubmit}>
          <Typography variant="h6" sx={{ mb: 2 }}>カメラ設定</Typography>
          <TextField
            fullWidth
            label="CAM_ZOOM"
            type="number"
            value={config.CAM_ZOOM}
            onChange={(e) => setConfig({...config, CAM_ZOOM: parseFloat(e.target.value)})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="CAM_FRONT"
            value={JSON.stringify(config.CAM_FRONT)}
            helperText="形式: [x, y, z]"
            onChange={(e) => {
              try {
                const value = JSON.parse(e.target.value);
                if (Array.isArray(value) && value.length === 3) {
                  setConfig({...config, CAM_FRONT: value as [number, number, number]});
                }
              } catch (e) {}
            }}
            sx={{ mb: 2 }}
          />

          <Typography variant="h6" sx={{ mt: 3, mb: 2 }}>ROSトピック設定</Typography>
          <TextField
            fullWidth
            label="ROS_SUB_TOPIC"
            value={config.ROS_SUB_TOPIC}
            onChange={(e) => setConfig({...config, ROS_SUB_TOPIC: e.target.value})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="ROS_PUB_TOPIC_RESULT"
            value={config.ROS_PUB_TOPIC_RESULT}
            onChange={(e) => setConfig({...config, ROS_PUB_TOPIC_RESULT: e.target.value})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="ROS_PUB_TOPIC_POINTCLOUD"
            value={config.ROS_PUB_TOPIC_POINTCLOUD}
            onChange={(e) => setConfig({...config, ROS_PUB_TOPIC_POINTCLOUD: e.target.value})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="ROS_PUB_TOPIC_IMAGE"
            value={config.ROS_PUB_TOPIC_IMAGE}
            onChange={(e) => setConfig({...config, ROS_PUB_TOPIC_IMAGE: e.target.value})}
            sx={{ mb: 2 }}
          />

          <Typography variant="h6" sx={{ mt: 3, mb: 2 }}>処理パラメータ</Typography>
          <TextField
            fullWidth
            label="BASE_PLANE_INDEX"
            type="number"
            value={config.BASE_PLANE_INDEX}
            onChange={(e) => setConfig({...config, BASE_PLANE_INDEX: parseInt(e.target.value)})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="MIL_PLANE_INDEX"
            type="number"
            value={config.MIL_PLANE_INDEX}
            onChange={(e) => setConfig({...config, MIL_PLANE_INDEX: parseInt(e.target.value)})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="CIRCLE_PLANE_INDEX"
            type="number"
            value={config.CIRCLE_PLANE_INDEX}
            onChange={(e) => setConfig({...config, CIRCLE_PLANE_INDEX: parseInt(e.target.value)})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="BASE_EXPECTED_EDGES"
            type="number"
            value={config.BASE_EXPECTED_EDGES}
            onChange={(e) => setConfig({...config, BASE_EXPECTED_EDGES: parseInt(e.target.value)})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="MIL_EXPECTED_EDGES"
            type="number"
            value={config.MIL_EXPECTED_EDGES}
            onChange={(e) => setConfig({...config, MIL_EXPECTED_EDGES: parseInt(e.target.value)})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="CONVEX_HULL_EPSILON"
            type="number"
            value={config.CONVEX_HULL_EPSILON}
            onChange={(e) => setConfig({...config, CONVEX_HULL_EPSILON: parseInt(e.target.value)})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="MIN_PLANE_POINTS"
            type="number"
            value={config.MIN_PLANE_POINTS}
            onChange={(e) => setConfig({...config, MIN_PLANE_POINTS: parseInt(e.target.value)})}
            sx={{ mb: 3 }}
          />

          {/* 新しいフィールドを追加 */}
          <Typography variant="h6" sx={{ mt: 3, mb: 2 }}>その他の設定</Typography>
          <TextField
            fullWidth
            label="LOG_LEVEL"
            value={config.LOG_LEVEL}
            onChange={(e) => setConfig({...config, LOG_LEVEL: e.target.value})}
            sx={{ mb: 2 }}
          />
          <TextField
            fullWidth
            label="MODE"
            value={config.MODE}
            onChange={(e) => setConfig({...config, MODE: e.target.value})}
            sx={{ mb: 3 }}
          />
          
          <Button type="submit" variant="contained" color="primary">
            設定を保存
          </Button>
        </form>
      </Paper>
    </Box>
  );
};
