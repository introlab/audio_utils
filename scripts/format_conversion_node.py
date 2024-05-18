#!/usr/bin/env python3

import rclpy
import rclpy.node

from audio_utils.msg import AudioFrame
from audio_utils import get_format_information, convert_audio_data_to_numpy_frames, convert_numpy_frames_to_audio_data


class FormatConversionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('format_conversion_node')

        self._input_format = self.declare_parameter('input_format', '').get_parameter_value().string_value
        self._output_format = self.declare_parameter('output_format', '').get_parameter_value().string_value

        self._input_format_information = get_format_information(self._input_format)
        self._output_format_information = get_format_information(self._output_format)

        self._audio_frame_msg = AudioFrame()
        self._audio_pub = self.create_publisher(AudioFrame, 'audio_out', 10)
        self._audio_sub = self.create_subscription(AudioFrame, 'audio_in', self._audio_cb, 10)

    def _audio_cb(self, msg):
        if msg.format != self._input_format:
            self.get_logger().error(
                f'Invalid input format (msg.format={msg.format}, param.input_format={self._input_format})')
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
        rclpy.spin(self)


def main():
    rclpy.init()

    format_conversion_node = FormatConversionNode()
    format_conversion_node.run()

    format_conversion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
