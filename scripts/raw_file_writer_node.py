#!/usr/bin/env python3

import rclpy
import rclpy.node

from audio_utils.msg import AudioFrame


class RawFileWriterNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('raw_file_writer_node')

        self._output_path = self.declare_parameter('output_path', '').get_parameter_value().string_value

        self._file_descriptor = None
        self._audio_sub = self.create_subscription(AudioFrame, 'audio_in', self._audio_cb, 10)

    def _audio_cb(self, msg):
        if self._file_descriptor is None or self._file_descriptor.closed:
            return
        self._file_descriptor.write(msg.data)

    def run(self):
        with open(self._output_path, 'wb') as self._file_descriptor:
            rclpy.spin(self)


def main():
    rclpy.init()
    raw_file_writer_node = RawFileWriterNode()

    try:
        raw_file_writer_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        raw_file_writer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
