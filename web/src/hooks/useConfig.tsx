import { useState, useEffect } from 'react';
import { Config } from '../types/config';
import { eel } from '../App';

export const useConfig = () => {
  const [config, setConfig] = useState<Config | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const loadConfig = async () => {
    try {
      const json = await eel.read_config()();
      setConfig(json);
      setError(null);
    } catch (error) {
      setError('設定の読み込みに失敗しました');
    } finally {
      setLoading(false);
    }
  };

  const updateConfig = async (newConfig: Config) => {
    try {
      console.log(newConfig);
      // Json stringに変換して送信
      const jsonString = JSON.stringify(newConfig);
      const success = await eel.update_config(jsonString)();
      if (success) {
        await loadConfig();  // 設定を再読み込み
        return true;
      }
      return false;
    } catch (error) {
      setError('設定の更新に失敗しました');
      return false;
    }
  };

  useEffect(() => {
    loadConfig();
  }, []);

  return {
    config,
    setConfig,
    loading,
    error,
    updateConfig,
  };
};
