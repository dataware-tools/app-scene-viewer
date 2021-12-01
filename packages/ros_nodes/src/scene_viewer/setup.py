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
    'long_description': '# scene_viewer\n\napp-scene-viewer 用の ROS2 ノードです',
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
