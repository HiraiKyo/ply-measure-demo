import React from 'react';
import { Button, ButtonProps } from '@mui/material';

interface CallEelFunctionProps extends ButtonProps {
  callback: Function;
  onSuccess: (data: any) => void;
}

export const CallEelFunctionButton: React.FC<CallEelFunctionProps> = ({
  callback,
  onSuccess,
  children,
  ...props
}) => {
  const handleClick = async () => {
    try {
      const result = await callback();
      onSuccess(result);
    } catch (error) {
      console.error('Eel function error:', error);
    }
  };

  return (
    <Button
      onClick={handleClick}
      size="medium"
      {...props}
    >
      {children}
    </Button>
  );
};
