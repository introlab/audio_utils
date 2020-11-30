#!/usr/bin/env python3

import numpy as np

import rospy

from audio_utils.msg import AudioFrame


class RawFileWriterNode:
    def __init__(self):
        self._output_path = rospy.get_param('~output_path', '')

        self._file_descriptor = None

        self._audio_sub = rospy.Subscriber('audio_in', AudioFrame, self._audio_cb, queue_size=10)

    def _audio_cb(self, msg):
        if self._file_descriptor is None or self._file_descriptor.closed:
            return
        self._file_descriptor.write(msg.data)

    def run(self):
        with open(self._output_path, 'wb') as self._file_descriptor:
            rospy.spin()


def main():
    rospy.init_node('raw_file_writer_node')
    raw_file_writer_node = RawFileWriterNode()
    raw_file_writer_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
