import React, { useEffect } from 'react';
import {
  Box,
  Button,
  Typography,
  CircularProgress,
  Alert,
  Paper,
} from '@mui/material';
import { Config } from '../types/config';
import { eel } from '../App';
import { Vector3Field } from '../components/common/input/vector3';
import { NumberField } from '../components/common/input/number-field';
import { StringField } from '../components/common/input/string-field';

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
        CAM_UP: JSON.parse(formData.get('CAM_UP') as string),
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

      const success = await eel.update_config(JSON.stringify(newConfig))();
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
          <NumberField
            name="CAM_ZOOM"
            label="CAM_ZOOM"
            value={config.CAM_ZOOM}
            onChange={(value) => setConfig({...config, CAM_ZOOM: value})}
            sx={{ mb: 2 }}
          />
          <Vector3Field
            name="CAM_FRONT"
            label="CAM_FRONT"
            value={config.CAM_FRONT}
            onChange={(value) => setConfig({...config, CAM_FRONT: value})}
            sx={{ mb: 2 }}
          />
          <Vector3Field
            name="CAM_UP"
            label="CAM_UP"
            value={config.CAM_UP}
            onChange={(value) => setConfig({...config, CAM_UP: value})}
            sx={{ mb: 2 }}
          />

          <Typography variant="h6" sx={{ mt: 3, mb: 2 }}>ROSトピック設定</Typography>
          <StringField
            name="ROS_SUB_TOPIC"
            label="ROS_SUB_TOPIC"
            value={config.ROS_SUB_TOPIC}
            onChange={(value) => setConfig({...config, ROS_SUB_TOPIC: value})}
            sx={{ mb: 2 }}
          />
          <StringField
            name="ROS_PUB_TOPIC_RESULT"
            label="ROS_PUB_TOPIC_RESULT"
            value={config.ROS_PUB_TOPIC_RESULT}
            onChange={(value) => setConfig({...config, ROS_PUB_TOPIC_RESULT: value})}
            sx={{ mb: 2 }}
          />
          <StringField
            name="ROS_PUB_TOPIC_POINTCLOUD"
            label="ROS_PUB_TOPIC_POINTCLOUD"
            value={config.ROS_PUB_TOPIC_POINTCLOUD}
            onChange={(value) => setConfig({...config, ROS_PUB_TOPIC_POINTCLOUD: value})}
            sx={{ mb: 2 }}
          />
          <StringField
            name="ROS_PUB_TOPIC_IMAGE"
            label="ROS_PUB_TOPIC_IMAGE"
            value={config.ROS_PUB_TOPIC_IMAGE}
            onChange={(value) => setConfig({...config, ROS_PUB_TOPIC_IMAGE: value})}
            sx={{ mb: 2 }}
          />

          <Typography variant="h6" sx={{ mt: 3, mb: 2 }}>処理パラメータ</Typography>
          <NumberField
            name="BASE_PLANE_INDEX"
            label="BASE_PLANE_INDEX"
            value={config.BASE_PLANE_INDEX}
            onChange={(value) => setConfig({...config, BASE_PLANE_INDEX: value})}
            sx={{ mb: 2 }}
          />
          <NumberField
            name="MIL_PLANE_INDEX"
            label="MIL_PLANE_INDEX"
            value={config.MIL_PLANE_INDEX}
            onChange={(value) => setConfig({...config, MIL_PLANE_INDEX: value})}
            sx={{ mb: 2 }}
          />
          <NumberField
            name="CIRCLE_PLANE_INDEX"
            label="CIRCLE_PLANE_INDEX"
            value={config.CIRCLE_PLANE_INDEX}
            onChange={(value) => setConfig({...config, CIRCLE_PLANE_INDEX: value})}
            sx={{ mb: 2 }}
          />
          <NumberField
            name="BASE_EXPECTED_EDGES"
            label="BASE_EXPECTED_EDGES"
            value={config.BASE_EXPECTED_EDGES}
            onChange={(value) => setConfig({...config, BASE_EXPECTED_EDGES: value})}
            sx={{ mb: 2 }}
          />
          <NumberField
            name="MIL_EXPECTED_EDGES"
            label="MIL_EXPECTED_EDGES"
            value={config.MIL_EXPECTED_EDGES}
            onChange={(value) => setConfig({...config, MIL_EXPECTED_EDGES: value})}
            sx={{ mb: 2 }}
          />
          <NumberField
            name="CONVEX_HULL_EPSILON"
            label="CONVEX_HULL_EPSILON"
            value={config.CONVEX_HULL_EPSILON}
            onChange={(value) => setConfig({...config, CONVEX_HULL_EPSILON: value})}
            sx={{ mb: 2 }}
          />
          <NumberField
            name="MIN_PLANE_POINTS"
            label="MIN_PLANE_POINTS"
            value={config.MIN_PLANE_POINTS}
            onChange={(value) => setConfig({...config, MIN_PLANE_POINTS: value})}
            sx={{ mb: 3 }}
          />

          <Typography variant="h6" sx={{ mt: 3, mb: 2 }}>その他の設定</Typography>
          <StringField
            name="LOG_LEVEL"
            label="LOG_LEVEL"
            value={config.LOG_LEVEL}
            onChange={(value) => setConfig({...config, LOG_LEVEL: value})}
            sx={{ mb: 2 }}
          />
          <StringField
            name="MODE"
            label="MODE"
            value={config.MODE}
            onChange={(value) => setConfig({...config, MODE: value})}
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
