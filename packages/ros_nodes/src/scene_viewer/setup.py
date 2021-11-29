# -*- coding: utf-8 -*-
from setuptools import setup

packages = \
['scene_viewer']

package_data = \
{'': ['*']}

install_requires = \
['fire>=0.4.0,<0.5.0', 'numpy', 'pip>=20.0.0']

entry_points = \
{'console_scripts': ['template_node = scene_viewer.template_node:cli']}

setup_kwargs = {
    'name': 'scene-viewer',
    'version': '0.0.1',
    'description': 'Scene Viewer',
    'long_description': "# scene_viewer\n\napp-scene-viewer 用の ROS2 ノードです  \n\n\n## Developers' Guide\n### 開発を始めるには？\nROSを使うため、開発は基本的にコンテナ内で行います。  \n2つ上の階層で以下のコマンドを叩くことでコンテナを起動します  \n```bash\n$ docker-compose up\n\n```\n\nコンテナが立ち上がった状態で、別の端末から exec します  \n```bash\n$ docker exec -it app-scene-viewer-ros-nodes bash\n\n<コンテナ内>\n\n$ cd /opt/ros_nodes\n$ source utils/entrypoint.sh\n```\n\nこれで準備完了です\n\n### ROS2ノードを作るには？\n`scene_viewer` ディレクトリ内に任意の python スクリプトを作成します  \n依存ライブラリが増える場合は以下の手順で追加してください  \n```bash\n<コンテナ内>\n$ cd /opt/ros_nodes/src/scene_viewer\n$ poetry add <追加したいパッケージ>\n$ ./pyproject_to_setup.sh\n```\n\n\n### ROS2ノードをビルドするには？\n```bash\n<コンテナ内>\n$ source /opt/ros_nodes/utils/entrypoint.sh\n$ cd /opt/ros_nodes\n$ colcon build\n```\n\n\n### ROS2ノードを実行するには？\nビルドが完了した状態で以下を実行する  \n```bash\n<コンテナ内>\n$ source /opt/ros_nodes/utils/entrypoint.sh\n$ cd /opt/ros_nodes\n$ ros2 pkg list | grep scene_viewer   # scene_viewerパッケージが登録されていることを確認\n$ ros2 pkg executables scene_viewer   # 実行可能なコマンドの一覧が出てくる\n```",
    'author': 'Daiki Hayashi',
    'author_email': 'hayashi.daiki@hdwlab.co.jp',
    'maintainer': None,
    'maintainer_email': None,
    'url': 'https://github.com/dataware-tools/app-scene-viewer',
    'packages': packages,
    'package_data': package_data,
    'install_requires': install_requires,
    'entry_points': entry_points,
    'python_requires': '>=3.7,<4.0',
}

setup(
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    for package_name in packages
  ] + [
      ('share/' + package_name, ['package.xml']) for package_name in packages
  ],
  **setup_kwargs
)
