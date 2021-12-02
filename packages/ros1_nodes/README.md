# ros1_nodes

## 使い方
```bash
$ docker-compose up
```

↑をすると `ws://localhost:9090` でrosbridgeが使えるようになります.  

## Developers' Guide
### 開発を始めるには？
ROSを使うため、開発は基本的にコンテナ内で行います。  
このディレクトリ内で以下のコマンドを叩くことでコンテナを起動します
```bash
$ docker-compose up

```

コンテナが立ち上がった状態で、別の端末から exec します
```bash
$ docker exec -it app-scene-viewer-ros1-nodes-dev bash
<コンテナ内>
$ cd /opt/ros1_nodes
$ source utils/entrypoint.sh
```

これで準備完了です

### ROS1ノードを作るには？
`scene_viewer/scripts` ディレクトリ内に任意の python スクリプトを作成します
```bash
$ cd /opt/ros2_nodes/src/scene_viewer/scripts
$ cp template_node.py my_node.py
$ vim my_node.py
<いじる>
```

依存ライブラリが増える場合は `scene_viewer/requirements.txt` に追記してください


### ROS1ノードをビルドするには？
```bash
<コンテナ内>
$ source /opt/ros1_nodes/utils/entrypoint.sh
$ cd /opt/ros1_nodes/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

### ROS1ノードを実行するには？
ビルドが完了した状態で以下を実行する
```bash
<コンテナ内>
$ source /opt/ros1_nodes/utils/entrypoint.sh
$ cd /opt/ros1_nodes
$ source devel/setup.bash
$ rospack list | grep scene_viewer    # scene_viewerパッケージが登録されていることを確認

### scene_viewer パッケージが出てこない場合
$ cd /opt/ros1_nodes/src && catkin_init_workspace && cd .. && catkin_make         # パッケージをビルドしてインストール
$ source /opt/ros1_nodes/devel/setup.bash  # パッケージを読み込む

### scene_viewer パッケージが見れる場合
$ rosrun scene_viewer <Tabを2回打つと実行可能なコマンド一覧が表示される>
$ rosrun scene_viewer <コマンド>
```
