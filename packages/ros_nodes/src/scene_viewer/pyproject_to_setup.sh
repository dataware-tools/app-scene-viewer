#!/usr/bin/env bash

poetry build \
  && cd dist \
  && tar xf scene_viewer-*.tar.gz \
  && cat scene_viewer-*/setup.py | head -n -2 > ../setup.py \
  && cd ../ \
  && rm -rf dist

cat <<EOF >> setup.py
setup(
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    for package_name in packages
  ] + [
      ('share/' + package_name, ['package.xml']) for package_name in packages
  ],
  **setup_kwargs
)
EOF