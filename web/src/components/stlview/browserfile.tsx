import { Button } from "@mui/material";
import { styled } from '@mui/material/styles';
import React, { Dispatch, SetStateAction, useState } from "react";
import { eel } from "../../App";

interface BrowseStlFileProps {
  setFilePath: Dispatch<SetStateAction<string>>;
  disabled?: boolean;
}

const StyledButton = styled(Button)({
  minWidth: '150px',
  '&.MuiButton-contained': {
    backgroundColor: '#1976d2',
    '&:hover': {
      backgroundColor: '#1565c0',
    },
  },
});

export const BrowseStlFile: React.FC<BrowseStlFileProps>= ({
  ...props
}) => {
  const [ loading, setLoading ] = useState(false);

  const handleClick = async () => {
    setLoading(true);
    try {
      const filepath = await eel.browse_file()();
      const result= await eel.load_stl(filepath)();
      if (result.success === false){
        throw new Error(result.error);
      }
      const binary = atob(result.data);
      const len = binary.length;
      const bytes = new Uint8Array(len);
      for (let i = 0; i < len; i++) {
        bytes[i] = binary.charCodeAt(i);
      }
      const blob = new Blob([bytes], { type: 'application/octet-stream' });
      const stl = URL.createObjectURL(blob);
      props.setFilePath(stl);
    } catch (error) {
      console.error('Eel function error:', error);
    }
    setLoading(false);
  }

  return (
    <StyledButton
      disabled={loading || props.disabled}
      variant="contained"
      onClick={handleClick}
    >
      Browse STL file
    </StyledButton>
  );
}