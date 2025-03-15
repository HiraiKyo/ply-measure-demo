import React, { useState } from 'react';
import { Box, Container, Typography } from '@mui/material';
import { styled } from '@mui/material/styles';
import { BrowseStlFile } from '../components/stlview/browserfile';
import { STLViewerComponent } from '../components/stlview/view';

const StlViewContainer = styled(Container)({
  height: '100vh',
});

const ContentWrapper = styled(Box)({
  display: 'flex',
  flexDirection: 'column',
  gap: '16px',
  padding: '32px 0',
});

const ActionBox = styled(Box)({
  display: 'flex',
  justifyContent: 'flex-end',
});

const ViewerBox = styled(Box)({
  width: '100%',
  height: '600px',
  backgroundColor: 'background.paper',
  borderRadius: '8px',
  overflow: 'hidden',
});

const HeaederBox = styled(Box)({
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
});

export const StlViewPage: React.FC = () => {
  const [filePath, setFilePath] = useState<string>('');

  return (
    <StlViewContainer maxWidth="lg">
      <ContentWrapper>
        <HeaederBox>
          <Typography variant="h4">STL Viewer</Typography>
          <ActionBox>
            <BrowseStlFile setFilePath={setFilePath} />
          </ActionBox>
        </HeaederBox>

        <Box sx={{ display: 'flex', gap: 2 }}>
          {filePath && (
            <ViewerBox>
              <STLViewerComponent
                url={filePath}
                width={800}
                height={600}
              />
            </ViewerBox>
          )}
        </Box>
      </ContentWrapper>
    </StlViewContainer>
  );
};