import React, { useState } from 'react';
import { Button, ButtonProps, CircularProgress } from '@mui/material';

interface CallEelFunctionProps extends ButtonProps {
  callback: () => Promise<any>;
  onSuccess?: (data: any) => void;
}

export const CallEelFunctionButton: React.FC<CallEelFunctionProps> = ({
  callback,
  onSuccess,
  children,
  ...props
}) => {
  const [loading, setLoading] = useState(false);

  const handleClick = async () => {
    setLoading(true);
    try {
      const result = await callback();
      if(onSuccess) onSuccess(result);
    } catch (error) {
      console.error('Eel function error:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Button
      onClick={handleClick}
      disabled={loading || props.disabled}
      {...props}
    >
      {loading ? <CircularProgress size={24} /> : children}
    </Button>
  );
};
