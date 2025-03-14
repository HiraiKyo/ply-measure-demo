import React from 'react';
import { TextField } from '@mui/material';

interface NumberFieldProps {
  name: string;
  label: string;
  value: number;
  onChange: (value: number) => void;
  sx?: any;
}

export const NumberField = ({ name, label, value, onChange, sx }: NumberFieldProps) => {
  return (
    <TextField
      fullWidth
      name={name}
      label={label}
      type="number"
      value={value}
      onChange={(e) => onChange(parseFloat(e.target.value) || 0)}
      sx={sx}
    />
  );
};
