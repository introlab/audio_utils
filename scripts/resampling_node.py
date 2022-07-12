#!/usr/bin/env python3

import numpy as np
import scipy.signal

import rospy

from audio_utils.msg import AudioFrame
from audio_utils import get_format_information, convert_audio_data_to_numpy_frames, convert_numpy_frames_to_audio_data
from abc import ABC, abstractmethod


class FrameInfo:
    def __init__(self, format: str, sampling_frequency: int, frame_sample_count: int, channel_count: int) -> None:
        self.format = format
        self.sampling_frequency = sampling_frequency
        self.format_information = get_format_information(format)
        self.frame_sample_count = frame_sample_count
        self.step = frame_sample_count // 2
        self.channel_count = channel_count

    @classmethod
    def from_audio_frame(cls, audio_frame: AudioFrame) -> 'FrameInfo':
        return cls(audio_frame.format, audio_frame.sampling_frequency, audio_frame.frame_sample_count, audio_frame.channel_count)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, FrameInfo) and
            self.format == other.format and
            self.sampling_frequency == other.sampling_frequency and
            self.frame_sample_count == other.frame_sample_count and
            self.channel_count == other.channel_count
        )


class FrameConverter:
    def __init__(self, format: str, sampling_frequency: int, frame_sample_count: int, channel_count: int) -> None:
        self.frame_info = FrameInfo(
            format, sampling_frequency, frame_sample_count, channel_count)
        self.window = np.sqrt(scipy.signal.windows.hann(frame_sample_count))
        self.buffer = np.zeros(
            (channel_count, int(self.frame_info.frame_sample_count * 3 / 2)))


class InputFrameConverter(FrameConverter):
    def __init__(self, format: str, sampling_frequency: int, frame_sample_count: int, channel_count: int) -> None:
        super().__init__(format, sampling_frequency, frame_sample_count, channel_count)

    @classmethod
    def from_input_info(cls, input_frame_info: FrameInfo) -> 'InputFrameConverter':
        return cls(input_frame_info.format, input_frame_info.sampling_frequency, input_frame_info.frame_sample_count, input_frame_info.channel_count)


class OutputFrameConverter(FrameConverter):
    def __init__(self, format: str, sampling_frequency: int, input_frame_info: FrameInfo) -> None:
        super().__init__(
            format,
            sampling_frequency,
            int(input_frame_info.frame_sample_count /
                input_frame_info.sampling_frequency * sampling_frequency),
            input_frame_info.channel_count
        )

    @classmethod
    def from_output_info(cls, output_frame_info: FrameInfo, input_frame_info: FrameInfo) -> 'OutputFrameConverter':
        return cls(output_frame_info.format, output_frame_info.sampling_frequency, input_frame_info)


class Resampler:
    def __init__(self, input_frame_converter: InputFrameConverter, output_frame_converter: OutputFrameConverter) -> None:
        self._input_frame_converter: InputFrameConverter = input_frame_converter
        self._output_frame_converter: OutputFrameConverter = output_frame_converter

        self._set_rospy_input_parameters(
            self._input_frame_converter.frame_info)

        self._audio_frame_msg = self._create_initialiazed_audio_frame_msg(
            self._output_frame_converter.frame_info)

    def resample(self, frame: AudioFrame) -> AudioFrame:
        input_frames = convert_audio_data_to_numpy_frames(
            self._input_frame_converter.frame_info.format_information,
            self._input_frame_converter.frame_info.channel_count,
            frame.data
        )
        output_frames = [
            self._resample_frame(input_frames[i], i) for i in range(
                self._input_frame_converter.frame_info.channel_count
            )
        ]

        self._audio_frame_msg.header = frame.header
        self._audio_frame_msg.data = convert_numpy_frames_to_audio_data(
            self._output_frame_converter.frame_info.format_information,
            output_frames
        )

        return self._audio_frame_msg

    @staticmethod
    def _create_initialiazed_audio_frame_msg(output_frame_info: FrameInfo) -> AudioFrame:
        audio_frame_msg = AudioFrame()
        audio_frame_msg.format = output_frame_info.format
        audio_frame_msg.channel_count = output_frame_info.channel_count
        audio_frame_msg.sampling_frequency = output_frame_info.sampling_frequency
        audio_frame_msg.frame_sample_count = output_frame_info.frame_sample_count
        return audio_frame_msg

    def _resample_frame(self, input_frame, i):
        self._input_frame_converter.buffer[i,
                                           self._input_frame_converter.frame_info.step:] = input_frame

        input_window1 = self._input_frame_converter.buffer[i,
                                                           :self._input_frame_converter.frame_info.frame_sample_count] * self._input_frame_converter.window
        input_window2 = self._input_frame_converter.buffer[i,
                                                           self._input_frame_converter.frame_info.step:] * self._input_frame_converter.window

        output_window1 = scipy.signal.resample(
            input_window1, self._output_frame_converter.frame_info.frame_sample_count) * self._output_frame_converter.window
        output_window2 = scipy.signal.resample(
            input_window2, self._output_frame_converter.frame_info.frame_sample_count) * self._output_frame_converter.window

        self._output_frame_converter.buffer[i,
                                            self._output_frame_converter.frame_info.step:] = 0
        self._output_frame_converter.buffer[i,
                                            :self._output_frame_converter.frame_info.frame_sample_count] += output_window1
        self._output_frame_converter.buffer[i,
                                            self._output_frame_converter.frame_info.step:] += output_window2

        self._input_frame_converter.buffer[i] = np.roll(
            self._input_frame_converter.buffer[i], self._input_frame_converter.frame_info.step)
        self._output_frame_converter.buffer[i] = np.roll(
            self._output_frame_converter.buffer[i], self._output_frame_converter.frame_info.step)

        return self._output_frame_converter.buffer[i, self._output_frame_converter.frame_info.step:]

    def is_same_input_frame_info(self, new_input_info: FrameInfo) -> bool:
        return self._input_frame_converter.frame_info == new_input_info

    @property
    def input_frame_info(self) -> FrameInfo:
        return self._input_frame_converter.frame_info

    def update_input_frame_info(self, new_input_info: FrameInfo) -> None:
        self._input_frame_converter = InputFrameConverter.from_input_info(
            new_input_info)
        self._output_frame_converter = OutputFrameConverter.from_output_info(
            self._output_frame_converter.frame_info, new_input_info)

        self._set_rospy_input_parameters(new_input_info)

        self._audio_frame_msg = self._create_initialiazed_audio_frame_msg(
            self._output_frame_converter.frame_info)

    @staticmethod
    def _set_rospy_input_parameters(new_input_info: FrameInfo) -> None:
        rospy.set_param('~input_format', new_input_info.format)
        rospy.set_param('~input_sampling_frequency',
                        new_input_info.sampling_frequency)
        rospy.set_param('~input_frame_sample_count',
                        new_input_info.frame_sample_count)


class OnCallbackStrategy(ABC):
    @abstractmethod
    def make_initial_input_frame_converter(self) -> InputFrameConverter: ...

    @abstractmethod
    def make_initial_output_frame_converter(
        self, input_frame_info: FrameInfo) -> OutputFrameConverter: ...

    @abstractmethod
    def audio_cb(self, msg: AudioFrame, resampler: Resampler,
                 publisher: rospy.Publisher) -> None: ...


class StaticResamplingOnCallbackStrategy(OnCallbackStrategy):

    def make_initial_input_frame_converter(self) -> InputFrameConverter:
        return InputFrameConverter(
            format=rospy.get_param('~input_format', ''),
            sampling_frequency=rospy.get_param('~input_sampling_frequency', 0),
            frame_sample_count=rospy.get_param('~input_frame_sample_count', 0),
            channel_count=rospy.get_param('~channel_count', 1),
        )

    def make_initial_output_frame_converter(self, input_frame_info: FrameInfo) -> OutputFrameConverter:
        return OutputFrameConverter(
            format=rospy.get_param('~output_format', ''),
            sampling_frequency=rospy.get_param(
                '~output_sampling_frequency', 0),
            input_frame_info=input_frame_info,
        )

    def audio_cb(self, msg: AudioFrame, resampler: Resampler, publisher: rospy.Publisher) -> None:
        # We don't support changing the input frame info dynamically
        if not resampler.is_same_input_frame_info(FrameInfo.from_audio_frame(msg)):
            input_param = resampler.input_frame_info
            rospy.logerr(
                f"Invalid frame (msg.format={msg.format}, input_param.format={input_param.format}, "
                f"msg.channel_count={msg.channel_count}, input_param.channel_count={input_param.channel_count}, "
                f"msg.sampling_frequency={msg.sampling_frequency}, input_param.sampling_frequency={input_param.sampling_frequency}, "
                f"msg.frame_sample_count={msg.frame_sample_count}, input_param.frame_sample_count={input_param.frame_sample_count})"
            )
            return

        publisher.publish(resampler.resample(msg))


class DynamicResamplingOnCallbackStrategy(OnCallbackStrategy):
    def make_initial_input_frame_converter(self) -> InputFrameConverter:
        # Default are valid, they can be set to choose the starting values, or they will adjust when the first frame is received
        return InputFrameConverter(
            format=rospy.get_param('~input_format', "signed_16"),
            sampling_frequency=rospy.get_param(
                '~input_sampling_frequency', 44100),
            frame_sample_count=rospy.get_param(
                '~input_frame_sample_count', 480),
            channel_count=rospy.get_param('~channel_count', 1),
        )

    def make_initial_output_frame_converter(self, input_frame_info: FrameInfo) -> OutputFrameConverter:
        return OutputFrameConverter(
            format=rospy.get_param('~output_format', ''),
            sampling_frequency=rospy.get_param(
                '~output_sampling_frequency', 0),
            input_frame_info=input_frame_info,
        )

    def audio_cb(self, msg: AudioFrame, resampler: Resampler, publisher: rospy.Publisher) -> None:
        # If input frame info has changed
        if not resampler.is_same_input_frame_info(FrameInfo.from_audio_frame(msg)):
            # We don't support changing the channel_count dynamically, it needs to stay the same
            if resampler.input_frame_info.channel_count != msg.channel_count:
                input_param = resampler.input_frame_info
                rospy.logerr(
                    f"Invalid channel_count: (msg.channel_count={msg.channel_count}, "
                    f"input_param.channel_count={input_param.channel_count}) can't change dynamically")
                return

            # We update the input frame info
            resampler.update_input_frame_info(FrameInfo.from_audio_frame(msg))

        publisher.publish(resampler.resample(msg))


class ResamplingNode:
    def __init__(self, resamplingStrategy: OnCallbackStrategy) -> None:

        self._resamplingStrategy = resamplingStrategy

        self._input_info = resamplingStrategy.make_initial_input_frame_converter()

        self._output_info = resamplingStrategy.make_initial_output_frame_converter(
            self._input_info.frame_info)

        self._resampler = Resampler(
            input_frame_converter=self._input_info, output_frame_converter=self._output_info)

        self._audio_pub = rospy.Publisher(
            'audio_out', AudioFrame, queue_size=100)
        self._audio_sub = rospy.Subscriber(
            'audio_in', AudioFrame, self._audio_cb, queue_size=100)

    def _audio_cb(self, msg: AudioFrame):
        self._resamplingStrategy.audio_cb(
            msg=msg, resampler=self._resampler, publisher=self._audio_pub)

    def run(self):
        rospy.spin()


class ResamplingNodeFactory:
    @staticmethod
    def create():
        dynamic_input_resampling = rospy.get_param(
            '~dynamic_input_resampling', False)
        if dynamic_input_resampling:
            return ResamplingNode(DynamicResamplingOnCallbackStrategy())
        else:
            return ResamplingNode(StaticResamplingOnCallbackStrategy())


def main():
    rospy.init_node('resampling_node')

    resampling_node = ResamplingNodeFactory.create()
    resampling_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
