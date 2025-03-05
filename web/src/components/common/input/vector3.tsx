import React from 'react';
import { TextField, Box } from '@mui/material';
import { Vector3 } from '../../../types/config';

interface Vector3FieldProps {
  name: string;
  label: string;
  value: Vector3;
  onChange: (value: Vector3) => void;
  sx?: any;
}

export const Vector3Field = ({ name, label, value, onChange, sx }: Vector3FieldProps) => {
  const handleChange = (index: number, val: string) => {
    const newValue = [...value] as Vector3;
    newValue[index] = parseFloat(val) || 0;
    onChange(newValue);
  };

  return (
    <Box sx={{ ...sx }}>
      <Box sx={{ mb: 1 }}>{label}</Box>
      <Box sx={{ display: 'flex', gap: 1 }}>
        <TextField
          name={`${name}_x`}
          label="X"
          type="number"
          value={value[0]}
          onChange={(e) => handleChange(0, e.target.value)}
          size="small"
        />
        <TextField
          name={`${name}_y`}
          label="Y"
          type="number"
          value={value[1]}
          onChange={(e) => handleChange(1, e.target.value)}
          size="small"
        />
        <TextField
          name={`${name}_z`}
          label="Z"
          type="number"
          value={value[2]}
          onChange={(e) => handleChange(2, e.target.value)}
          size="small"
        />
      </Box>
      {/* フォームデータ用の隠しフィールド */}
      <input
        type="hidden"
        name={name}
        value={JSON.stringify(value)}
      />
    </Box>
  );
};