#!/usr/bin/env python3
#
# Copyright 2021 Human Dataware Lab. Cc. Ltd.
# Created by Daiki Hayashi <hayashi.daiki@hdwlab.co.jp>
#

"""Generic file reader node."""

import json
from typing import Optional

import fire
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String

import pandas as pd
from pydtk.models import MetaDataModel
from pydtk.io import BaseFileReader


class FileReader(Node):
    """A generic file reader."""

    def __init__(self, file_path: str, mapping: Optional[dict] = None, filtering: Optional[list] = None):
        """A generic file reader node.
        Args:
            file_path (str): path to the file to load
            mapping (dict): a dictionary to map columns in the file
            filtering (list): a list to filter columns to publish

        """
        super().__init__('file_reader')
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.publisher = self.create_publisher(String, 'content', qos_profile=latching_qos)
        self.file_path = file_path
        self.mapping = mapping
        self.filtering = filtering
        df = self._read_file(file_path)
        df = self._map_columns(df, mapping)
        df = self._filter_columns(df, filtering)

        # convert to a list of dicts
        data = df.to_dict('records')

        # convert to a message
        msg = String()
        msg.data = json.dumps(data)

        # publish
        self.publisher.publish(msg)

    @staticmethod
    def _read_file(path: str) -> pd.DataFrame:
        # TODO: Remove the following lines
        from pydtk.models.csv import GenericCsvModel
        model = GenericCsvModel()
        model.load(path)
        data = model.data[1:, :]
        columns = model.data[0, :]

        # TODO: Use the following lines instead
        # reader = BaseFileReader()
        # _, data, columns = reader.read(MetaDataModel(data={"path": path}))

        # Convert to pandas dataframe
        df = pd.DataFrame(data, columns=columns)

        return df

    @staticmethod
    def _map_columns(df: pd.DataFrame, mapping: Optional[dict]) -> pd.DataFrame:
        mapping_dict = mapping or {}
        new_df = df.copy()
        for key, value in mapping_dict.items():
            new_df[key] = df[value]
            del new_df[value]
        return new_df

    @staticmethod
    def _filter_columns(df: pd.DataFrame, filtering: Optional[list]) -> pd.DataFrame:
        if filtering is None:
            return df
        new_df = df.copy()
        for column in new_df.columns:
            if column not in filtering:
                del new_df[column]
        return new_df


def main(file_path: str, mapping: str = None, filtering: str = None):
    """A generic file reader node.
    Args:
        file_path (str): path to the file to load
        mapping (str): JSON string of a dictionary to map columns in the file
        filtering (str): JSON string of a list to filter columns to publish

    """
    rclpy.init()

    reader = FileReader(file_path, mapping=mapping, filtering=filtering)

    rclpy.spin(reader)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reader.destroy_node()
    rclpy.shutdown()


def cli():
    """Command-line interface."""
    fire.Fire(main)


if __name__ == "__main__":
    cli()
