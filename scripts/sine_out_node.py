#!/usr/bin/env python3
import rospy
import numpy as np

from audio_utils.msg import AudioFrame


class SineOutNode:
    def __init__(self):
        self._sine_freq_hz = rospy.get_param('~sine_frequency_hz', 440)
        self._format = rospy.get_param('~format', 'signed_16')
        self._sampling_frequency = rospy.get_param('~sampling_frequency', 48000)
        self._frame_sample_count = rospy.get_param('~frame_sample_count', 480)
        self._amplitude = rospy.get_param('~amplitude', 50)
        self._audio_pub = rospy.Publisher('/audio_out', AudioFrame, queue_size=100)

    def run(self):
        rate = rospy.Rate(self._sampling_frequency / self._frame_sample_count)

        sample_count = 0

        while not rospy.is_shutdown():
            frame = AudioFrame()
            frame.format = self._format
            frame.frame_sample_count = self._frame_sample_count
            frame.sampling_frequency = self._sampling_frequency
            frame.channel_count = 1

            # Generate sine wave
            if self._format == 'signed_16':

                # Publish zeros at first to fill buffers
                if sample_count == 0:
                    frame.data = np.zeros(self._frame_sample_count).astype(np.int16).tobytes()
                    for _ in range(50):
                        self._audio_pub.publish(frame)

                samples = (self._amplitude * np.sin(2 * np.pi * np.arange(start=sample_count,
                                                                          stop=sample_count + self._frame_sample_count)
                                                    * self._sine_freq_hz / self._sampling_frequency)).astype(np.int16)

                sample_count += self._frame_sample_count
                frame.data = samples.tobytes()

            else:
                print('Format not supported: ', self._format)
                return

            # Publish
            self._audio_pub.publish(frame)

            rate.sleep()


def main():
    rospy.init_node('sine_out_node')
    sine_out_node = SineOutNode()
    sine_out_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
