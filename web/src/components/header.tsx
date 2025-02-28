import React from 'react';
import styled from 'styled-components';

const HeaderContainer = styled.header`
  background-color: #1e1e1e;
  color: #cccccc;
  height: 32px;
  display: flex;
  align-items: center;
  padding: 0 12px;
  -webkit-app-region: drag;
  user-select: none;
`;

const Title = styled.h1`
  font-size: 12px;
  font-weight: normal;
  margin: 0;
`;

const MenuArea = styled.div`
  display: flex;
  gap: 16px;
  margin-left: 16px;
  -webkit-app-region: no-drag;
`;

const MenuItem = styled.span`
  font-size: 12px;
  cursor: pointer;
  &:hover {
    opacity: 0.8;
  }
`;

export function Header() {
  return (
    <HeaderContainer>
      <Title>3D点群処理アプリケーション</Title>
      <MenuArea>
        <MenuItem>ファイル</MenuItem>
        <MenuItem>編集</MenuItem>
        <MenuItem>表示</MenuItem>
        <MenuItem>ヘルプ</MenuItem>
      </MenuArea>
    </HeaderContainer>
  );
}