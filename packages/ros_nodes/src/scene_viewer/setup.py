# -*- coding: utf-8 -*-
from setuptools import setup

packages = \
['scene_viewer']

package_data = \
{'': ['*']}

install_requires = \
['numpy', 'pip>=20.0.0']

setup_kwargs = {
    'name': 'scene-viewer',
    'version': '0.0.1',
    'description': 'Scene Viewer',
    'long_description': '# scene_viewer',
    'author': 'Daiki Hayashi',
    'author_email': 'hayashi.daiki@hdwlab.co.jp',
    'maintainer': None,
    'maintainer_email': None,
    'url': 'https://github.com/dataware-tools/app-scene-viewer',
    'packages': packages,
    'package_data': package_data,
    'install_requires': install_requires,
    'python_requires': '>=3.7,<4.0',
}


setup(**setup_kwargs)
