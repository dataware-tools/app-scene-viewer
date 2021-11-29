# ros_nodes

## 使い方
```bash
$ docker-compose up
```

↑をすると `ws://localhost:9000` でrosbridgeが使えるようになります.  


## ディレクトリ構成
```bash
.
├── Dockerfile
├── Makefile
├── README.md
├── docker-compose.yaml
├── src
│   ├── scene_viewer            app-scene-viewer用のROS2ノード
│   └── scene_viewer_msgs       app-scene-viewer用のROS2メッセージおよびサービス
└── utils
    ├── entrypoint.sh
    ├── get-version.sh
    └── run.sh
```