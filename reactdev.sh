#!/bin/bash

PACKAGE_NAME="ply-measure-demo"
WEB_DIR="/web"

# Dockerコンテナ内のnode_modulesのパス
cd /root/catkin_ws/src/$PACKAGE_NAME$WEB_DIR

# npmのキャッシュディレクトリの権限を設定
mkdir -p /root/.npm
chmod -R 777 /root/.npm

# node_modulesの権限を設定とreact-scriptsの実行権限確保
if [ -d "node_modules" ]; then
    chmod -R 777 node_modules
    chmod +x node_modules/.bin/react-scripts
fi

# パスを表示
echo "Current PATH: $PATH"

# node_modulesの.binディレクトリにパスを追加
NODE_BIN="/root/catkin_ws/src/$PACKAGE_NAME$WEB_DIR/node_modules/.bin"
export PATH="$NODE_BIN:$PATH"

# react-scriptsの場所を確認
REACT_SCRIPTS_AFTER=$(which react-scripts || echo "still not found")
echo "react-scripts location PATH : $REACT_SCRIPTS_AFTER"

# デバッグ情報を表示しながらフォアグラウンドで実行
cd /root/catkin_ws/src/$PACKAGE_NAME$WEB_DIR && \
NODE_ENV=development \
$NODE_BIN/react-scripts start > /tmp/react-server.log 2>&1 &