"""Read selected topics from a rosbag2 directory (sqlite3 or mcap) into arrays.

Needs a sourced ROS2 environment (rosbag2_py, rclpy). Reading mcap bags also
needs ros-<distro>-rosbag2-storage-mcap.
"""
import os

import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def storage_id(bag_dir):
    with open(os.path.join(bag_dir, 'metadata.yaml'), 'r') as f:
        meta = yaml.safe_load(f)
    return meta['rosbag2_bagfile_information']['storage_identifier']


def read_topics(bag_dir, topics):
    """Return {topic: [(t_stamp, msg), ...]} for the requested topics.

    t_stamp is header.stamp in seconds if the message has a non-zero header,
    otherwise the time the message was recorded.
    """
    import rosbag2_py  # imported here so the fitting code works without ROS

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id(bag_dir)),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {t: get_message(types[t]) for t in topics if t in types}
    out = {t: [] for t in topics}

    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(data, wanted[topic])
        t = t_rec * 1e-9
        header = getattr(msg, 'header', None)
        if header is not None and (header.stamp.sec or header.stamp.nanosec):
            t = header.stamp.sec + header.stamp.nanosec * 1e-9
        out[topic].append((t, msg))

    for samples in out.values():
        samples.sort(key=lambda s: s[0])
    return out


def read_records(bag_dir, topics):
    """Return {topic: [(t_recorded, t_stamp, msg), ...]} in recording order (not sorted).

    t_recorded is the time the recorder received the message, t_stamp is
    header.stamp if the message has a non-zero header, otherwise t_recorded.
    Both in seconds. Topics missing from the bag are absent from the result.
    """
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id(bag_dir)),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {t: get_message(types[t]) for t in topics if t in types}
    out = {t: [] for t in wanted}

    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(data, wanted[topic])
        t_rec = t_rec * 1e-9
        t = t_rec
        header = getattr(msg, 'header', None)
        if header is not None and (header.stamp.sec or header.stamp.nanosec):
            t = header.stamp.sec + header.stamp.nanosec * 1e-9
        out[topic].append((t_rec, t, msg))
    return out
