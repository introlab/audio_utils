#!/usr/bin/env python3

import numpy as np
import scipy.signal

import rospy

from audio_utils.msg import AudioFrame
from audio_utils import get_format_information, convert_audio_data_to_numpy_frames, convert_numpy_frames_to_audio_data


class ResamplingNode:
    def __init__(self):
        self._input_format = rospy.get_param('~input_format', '')
        self._output_format = rospy.get_param('~output_format', '')
        self._channel_count = rospy.get_param('~channel_count', 1)
        self._input_sampling_frequency = rospy.get_param('~input_sampling_frequency', 0)
        self._output_sampling_frequency = rospy.get_param('~output_sampling_frequency', 0)
        self._input_frame_sample_count = rospy.get_param('~input_frame_sample_count', 0)

        self._input_format_information = get_format_information(self._input_format)
        self._output_format_information = get_format_information(self._output_format)


        self._output_frame_sample_count = int(self._input_frame_sample_count / self._input_sampling_frequency * self._output_sampling_frequency)
        self._input_step = self._input_frame_sample_count // 2
        self._output_step = self._output_frame_sample_count // 2

        self._audio_frame_msg = self._create_initialiaze_audio_frame_msg()

        self._input_window = np.sqrt(scipy.signal.hann(self._input_frame_sample_count))
        self._output_window = np.sqrt(scipy.signal.hann(self._output_frame_sample_count))
        self._input_buffer = np.zeros((self._channel_count, self._input_step * 3))
        self._output_buffer = np.zeros((self._channel_count, int(self._output_frame_sample_count * 3 / 2)))

        self._audio_pub = rospy.Publisher('audio_out', AudioFrame, queue_size=100)
        self._audio_sub = rospy.Subscriber('audio_in', AudioFrame, self._audio_cb, queue_size=100)

    def _create_initialiaze_audio_frame_msg(self):
        audio_frame_msg = AudioFrame()
        audio_frame_msg.format = self._output_format
        audio_frame_msg.channel_count = self._channel_count
        audio_frame_msg.sampling_frequency = self._output_sampling_frequency
        audio_frame_msg.frame_sample_count = self._output_frame_sample_count
        return audio_frame_msg

    def _audio_cb(self, msg):
        if msg.format != self._input_format or \
                msg.channel_count != self._channel_count or \
                msg.sampling_frequency != self._input_sampling_frequency or \
                msg.frame_sample_count != self._input_frame_sample_count:
            rospy.logerr('Invalid frame (msg.format={}, param.input_format={}, msg.channel_count={}, param.channel_count={}, msg.sampling_frequency={},' \
                'param.input_sampling_frequency={}, msg.frame_sample_count={}, param.input_frame_sample_count={})'
                .format(msg.format, self._input_format, msg.channel_count, self._channel_count, msg.frame_sample_count, self._input_frame_sample_count))
            return

        input_frames = convert_audio_data_to_numpy_frames(self._input_format_information, self._channel_count, msg.data)
        output_frames = [self._resample_frame(input_frames[i], i) for i in range(self._channel_count)]

        self._audio_frame_msg.data = convert_numpy_frames_to_audio_data(self._output_format_information, output_frames)
        self._audio_pub.publish(self._audio_frame_msg)

    def _resample_frame(self, input_frame, i):
        self._input_buffer[i, self._input_step:] = input_frame

        input_window1 = self._input_buffer[i, :self._input_frame_sample_count] * self._input_window
        input_window2 = self._input_buffer[i, self._input_step:] * self._input_window

        output_window1 = scipy.signal.resample(input_window1, self._output_frame_sample_count) * self._output_window
        output_window2 = scipy.signal.resample(input_window2, self._output_frame_sample_count) * self._output_window

        self._output_buffer[i, self._output_step:] = 0
        self._output_buffer[i, :self._output_frame_sample_count] += output_window1
        self._output_buffer[i, self._output_step:] += output_window2

        self._input_buffer[i] = np.roll(self._input_buffer[i], self._input_step)
        self._output_buffer[i] = np.roll(self._output_buffer[i], self._output_step)

        return self._output_buffer[i, self._output_step:]

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('resampling_node')
    resampling_node = ResamplingNode()
    resampling_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
