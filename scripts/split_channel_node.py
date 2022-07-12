#!/usr/bin/env python3

import numpy as np

import rospy

from audio_utils.msg import AudioFrame
from audio_utils import get_format_information, convert_audio_data_to_numpy_frames, convert_numpy_frames_to_audio_data


class SplitChannelNode:
    def __init__(self):
        self._input_format = rospy.get_param('~input_format', '')
        self._output_format = rospy.get_param('~output_format', '')
        self._channel_count = rospy.get_param('~channel_count', 0)

        self._input_format_information = get_format_information(self._input_format)
        self._output_format_information = get_format_information(self._output_format)

        self._audio_pubs = [rospy.Publisher(
            'audio_out_{}'.format(i), AudioFrame, queue_size=10) for i in range(self._channel_count)]
        self._audio_sub = rospy.Subscriber('audio_in', AudioFrame, self._audio_cb, queue_size=10)

    def _audio_cb(self, msg):
        if msg.format != self._input_format or msg.channel_count != self._channel_count:
            rospy.logerr('Invalid frame (msg.format={}, msg.channel_count={})'.format(msg.format, msg.channel_count))
            return

        frames = convert_audio_data_to_numpy_frames(self._input_format_information, msg.channel_count, msg.data)

        audio_frame_msg = AudioFrame()
        audio_frame_msg.header = msg.header
        audio_frame_msg.format = self._output_format
        audio_frame_msg.channel_count = 1
        audio_frame_msg.sampling_frequency = msg.sampling_frequency
        audio_frame_msg.frame_sample_count = msg.frame_sample_count

        for i in range(len(frames)):
            data = convert_numpy_frames_to_audio_data(self._output_format_information, [frames[i]])
            audio_frame_msg.data = data
            self._audio_pubs[i].publish(audio_frame_msg)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('split_channel_node')
    split_channel_node = SplitChannelNode()
    split_channel_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
