# ros_nodes

app-scene-viewer 用の ROS2 パッケージです

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

## 使い方
1. サンプルデータをダウンロード
```bash
$ docker-compose up get-rosbag
```

2. ROSノードを起動し、サンプルデータを配信
```bash
$ docker-compose up sample
```

↑をすると `ws://localhost:9090` で rosbridge が使えるようになります.

## Developers' Guide
### 開発を始めるには？
ROSを使うため、開発は基本的にコンテナ内で行います。  
このディレクトリ内で以下のコマンドを叩くことでコンテナを起動します
```bash
$ docker-compose up

```

コンテナが立ち上がった状態で、別の端末から exec します
```bash
$ docker exec -it app-scene-viewer-ros-nodes-dev bash
<コンテナ内>
$ cd /opt/ros_nodes
$ source utils/entrypoint.sh
```

これで準備完了です

### ROS2ノードを作るには？
`scene_viewer` ディレクトリ内に任意の python スクリプトを作成します
```bash
$ cd /opt/ros_nodes/src/scene_viewer
$ cp template_node.py my_node.py
$ vim my_node.py
<いじる>
```
その後、ノードを実行可能にするために `pyproject.toml` の `[tool.poetry.script]` に以下の様なものを追加します
```toml
コマンド名 = "scene_viewer.<pythonスクリプト>:<スクリプト内の関数>"

(例)
my_node = "scene_viewer.my_node:cli"
```

依存ライブラリが増える場合は以下の手順で追加してください
```bash
<コンテナ内>
$ cd /opt/ros_nodes/src/scene_viewer
$ poetry add <追加したいパッケージ>
```

最後に、 `pyproject.toml` を `setup.py` に変換します
```bash
<コンテナ内>
$ cd /opt/ros_nodes/src/scene_viewer
$ ./pyproject_to_setup.sh
```


### ROS2ノードをビルドするには？
```bash
<コンテナ内>
$ source /opt/ros_nodes/utils/entrypoint.sh
$ cd /opt/ros_nodes
$ colcon build
```

### ROS2ノードをテストするには？
```bash
<コンテナ内>
$ source /opt/ros_nodes/utils/entrypoint.sh
$ cd /opt/ros_nodes
$ colcon test
```

### ROS2ノードを実行するには？
ビルドが完了した状態で以下を実行する
```bash
<コンテナ内>
$ source /opt/ros_nodes/utils/entrypoint.sh
$ cd /opt/ros_nodes
$ ros2 pkg list | grep scene_viewer   # scene_viewerパッケージが登録されていることを確認

### scene_viewer パッケージが出てこない場合
$ cd /opt/ros_nodes && colcon build         # パッケージをビルドしてインストール
$ source /opt/ros_nodes/install/setup.bash  # パッケージを読み込む

### scene_viewer パッケージが見れる場合
$ ros2 pkg executables scene_viewer   # 実行可能なコマンドの一覧が出てくる
$ ros2 run scene_viewer <コマンド>
```
