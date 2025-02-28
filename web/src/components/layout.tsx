import React from 'react';
import styled from 'styled-components';
import { Header } from './header';

const Container = styled.div`
  display: flex;
  flex-direction: column;
  height: 100vh;
  background-color: #1e1e1e;
  color: #cccccc;
`;

const Main = styled.main`
  display: flex;
  flex: 1;
  overflow: hidden;
`;

const Sidebar = styled.aside`
  width: 48px;
  background-color: #333333;
  display: flex;
  flex-direction: column;
  align-items: center;
  padding-top: 8px;
`;

const Content = styled.div`
  flex: 1;
  display: flex;
  flex-direction: column;
  background-color: #252526;
`;

const StatusBar = styled.footer`
  height: 22px;
  background-color: #007acc;
  color: white;
  font-size: 12px;
  display: flex;
  align-items: center;
  padding: 0 8px;
`;

export function Layout({ children }: { children: React.ReactNode }) {
  return (
    <Container>
      <Header />
      <Main>
        <Sidebar>
          {/* アイコンなどを配置 */}
        </Sidebar>
        <Content>
          {children}
        </Content>
      </Main>
      <StatusBar>
        Ready
      </StatusBar>
    </Container>
  );
}
