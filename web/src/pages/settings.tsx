import { Container, Typography, Box, Radio, RadioGroup, FormControlLabel, FormControl, FormLabel, Button, Alert, Snackbar } from "@mui/material";
import { useEffect, useState } from "react";
import { eel } from "../App";

interface Settings {
  MODE?: "ubuntu" | "wsl2"
}

export const SettingsPage = () => {
  const [settings, setSettings] = useState<Settings>({});
  const [showSuccess, setShowSuccess] = useState(false);
  const [showError, setShowError] = useState(false);

  const handleChange = async (key: keyof Settings, value: Settings[keyof Settings]) => {
    const newSettings = { ...settings, [key]: value };
    setSettings(newSettings);
    
    try {
      await eel.update_settings(JSON.stringify(newSettings))();
      setShowSuccess(true);
    } catch (error) {
      console.error('設定の保存に失敗しました:', error);
      setShowError(true);
    }
  };

  useEffect(() => {
    eel.read_settings()().then((result: Settings) => {
      console.log("Loading settings: ", result);
      setSettings(result);
    }).catch((err: Error) => {
      console.error(err);
      setShowError(true);
    });
  }, []);

  return (
    <Container>
      <Box sx={{ py: 4 }}>
        <Typography variant="h4" gutterBottom>Settings</Typography>
        
        <Box sx={{ my: 3 }}>
          <FormControl>
            <FormLabel>実行環境</FormLabel>
            <RadioGroup
              value={settings.MODE || ''}
              onChange={(e) => handleChange('MODE', e.target.value as Settings['MODE'])}
            >
              <FormControlLabel value="ubuntu" control={<Radio />} label="Ubuntu (Native)" />
              <FormControlLabel value="wsl2" control={<Radio />} label="WSL2" />
            </RadioGroup>
          </FormControl>
        </Box>

        <Snackbar open={showSuccess} autoHideDuration={3000} onClose={() => setShowSuccess(false)}>
          <Alert severity="success" onClose={() => setShowSuccess(false)}>
            設定を保存しました
          </Alert>
        </Snackbar>

        <Snackbar open={showError} autoHideDuration={3000} onClose={() => setShowError(false)}>
          <Alert severity="error" onClose={() => setShowError(false)}>
            エラーが発生しました
          </Alert>
        </Snackbar>
      </Box>
    </Container>
  );
};