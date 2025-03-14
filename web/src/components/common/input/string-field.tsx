import React from 'react';
import { TextField } from '@mui/material';

interface StringFieldProps {
  name: string;
  label: string;
  value: string;
  onChange: (value: string) => void;
  sx?: any;
}

export const StringField = ({ name, label, value, onChange, sx }: StringFieldProps) => {
  return (
    <TextField
      fullWidth
      name={name}
      label={label}
      value={value}
      onChange={(e) => onChange(e.target.value)}
      sx={sx}
    />
  );
};
