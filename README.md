# ply-measure-demo

点群データから寸法算出デモ

## Requirements

- Docker

## Run
```sh
docker compose up -d
docker compose exec ply-measure-demo bash
$ cd catkin_ws
$ roslaunch ply-measure-demo react.launch
```

## Options

### Show React Log
```sh
docker compose exec ply-measure-demo bash -c "tail -f /tmp/react-server.log"
```

### GUI Development
```sh
docker compose exec ply-measure-demo bash -c "cd catkin_ws && roslaunch ply-measure-demo react.dev.launch"
```

Connect localhost:3000 and hot reload is available.