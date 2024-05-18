#!/usr/bin/env python3

import rclpy
import rclpy.node

from audio_utils.msg import AudioFrame
from audio_utils import get_format_information, convert_audio_data_to_numpy_frames, convert_numpy_frames_to_audio_data


class SplitChannelNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('split_channel_node')

        self._input_format = self.declare_parameter('input_format', '').get_parameter_value().string_value
        self._output_format = self.declare_parameter('output_format', '').get_parameter_value().string_value
        self._channel_count = self.declare_parameter('channel_count', 0).get_parameter_value().integer_value

        self._input_format_information = get_format_information(self._input_format)
        self._output_format_information = get_format_information(self._output_format)

        self._audio_pubs = [self.create_publisher(
            AudioFrame, f'audio_out_{i}', 10) for i in range(self._channel_count)]
        self._audio_sub = self.create_subscription(AudioFrame, 'audio_in', self._audio_cb, 10)

    def _audio_cb(self, msg):
        if msg.format != self._input_format or msg.channel_count != self._channel_count:
            self.get_logger().error(
                f'Invalid frame (msg.format={msg.format}, msg.channel_count={msg.channel_count})')
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
        rclpy.spin(self)


def main():
    rclpy.init()

    split_channel_node = SplitChannelNode()
    split_channel_node.run()

    split_channel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
