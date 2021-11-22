#!/usr/bin/env bash

poetry build \
  && cd dist \
  && tar xf scene_viewer-*.tar.gz \
  && cp scene_viewer-*/setup.py ../ \
  && cd ../ \
  && rm -rf dist