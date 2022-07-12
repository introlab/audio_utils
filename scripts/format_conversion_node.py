#!/usr/bin/env python3

import numpy as np

import rospy

from audio_utils.msg import AudioFrame
from audio_utils import get_format_information, convert_audio_data_to_numpy_frames, convert_numpy_frames_to_audio_data


class FormatConversionNode:
    def __init__(self):
        self._input_format = rospy.get_param('~input_format', '')
        self._output_format = rospy.get_param('~output_format', '')

        self._input_format_information = get_format_information(self._input_format)
        self._output_format_information = get_format_information(self._output_format)


        self._audio_frame_msg = AudioFrame()

        self._audio_pub = rospy.Publisher('audio_out', AudioFrame, queue_size=10)
        self._audio_sub = rospy.Subscriber('audio_in', AudioFrame, self._audio_cb, queue_size=10)

    def _audio_cb(self, msg):
        if msg.format != self._input_format:
            rospy.logerr('Invalid input format (msg.format={}, param.input_format={})'.format(msg.format, self._input_format))
            return

        frames = convert_audio_data_to_numpy_frames(self._input_format_information, msg.channel_count, msg.data)
        data = convert_numpy_frames_to_audio_data(self._output_format_information, frames)

        self._audio_frame_msg.header = msg.header
        self._audio_frame_msg.format = self._output_format
        self._audio_frame_msg.channel_count = msg.channel_count
        self._audio_frame_msg.sampling_frequency = msg.sampling_frequency
        self._audio_frame_msg.frame_sample_count = msg.frame_sample_count
        self._audio_frame_msg.data = data

        self._audio_pub.publish(self._audio_frame_msg)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('format_conversion_node')
    format_conversion_node = FormatConversionNode()
    format_conversion_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
