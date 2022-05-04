# audio_utils

ROS nodes and utilities for audio streams.

Author(s): Marc-Antoine Maheux

## Setup (Ubuntu)

The following subsections explain how to use the library on Ubuntu.

### Install Dependencies

```bash
sudo apt-get install cmake build-essential gfortran texinfo libasound2-dev libpulse-dev
```

### Install Python Dependencies

```bash
sudo pip install -r requirements.txt
```

or

```bash
sudo pip3 install -r requirements.txt
```

### Setup Submodules

```bash
git submodule update --init --recursive
```

# Nodes

## `capture_node`

This node captures the sound from an ALSA or PulseAudio device and publishes it to a topic.

### Parameters

- `backend` (string): The backend to use (`alsa` or `pulse_audio`).
- `device` (string): The device to capture (ex: `hw:CARD=1,DEV=0` for ALSA or
  `alsa_input.usb-IntRoLab_16SoundsUSB_Audio_2.0-00.multichannel-input` for PulseAudio).
- `format` (string): The audio format (
  see [audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)).
- `channel_count` (int): The device channel count.
- `sampling_frequency` (int): The device sampling frequency.
- `frame_sample_count` (int): The number of samples in each frame.
- `merge` (bool): Indicate to merge the channels or not. The default value is `false`.
- `gain` (double): The gain to apply. The default value is `1.0`
- `latency_us` (int): The capture latency in microseconds. This parameter must be set only with the ALSA backend.
- `channel_map` (Array of string): The PulseAudio channel mapping. If empty or omitted, the default mapping is used.
  This parameter must be set only with the PulseAudio backend. In launch files, use this syntax :
  `<rosparam param="channel_map">[]</rosparam>`.

### Published Topics

- `audio_out` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The
  captured sound.

## `playback_node`

This node captures the sound from a topic and plays it to an ALSA device.

### Parameters

- `backend` (string): The backend to use (`alsa` or `pulse_audio`).
- `device` (string): The device to capture (ex: `hw:CARD=1,DEV=0` for ALSA or
  `alsa_input.usb-IntRoLab_16SoundsUSB_Audio_2.0-00.multichannel-input` for PulseAudio).
- `format` (string): The audio format (
  see [audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)).
- `channel_count` (int): The device channel count.
- `sampling_frequency` (int): The device sampling frequency.
- `frame_sample_count` (int): The number of samples in each frame.
- `latency_us` (int): The capture latency in microseconds. This parameter must be set only with the ALSA backend.
- `channel_map` (Array of string): The PulseAudio channel mapping. If empty or omitted, the default mapping is used.
  This parameter must be set only with the PulseAudio backend. In launch files, use this syntax :
  `<rosparam param="channel_map">[]</rosparam>`.

### Subscribed Topics

- `audio_in` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The sound
  to play.

## `beat_detector_node`

This node estimates the song tempo and detects if the beat is in the current frame.

### Subscribed Topics

- `audio_in` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The sound
  to analyze. The channel count must be 1, the sampling frequency must be 44100 Hz and the frame sample count must be
    256.

### Published Topics

- `bpm` (std_msgs/Float32): The tempo in bpm (beats per minute) for each frame.
- `beat` (std_msgs/Bool): Indicate if the beat is in the current frame.

## `format_conversion_node.py`

This node converts the format of an audio topic.

### Parameters

- `input_format` (string): The input audio format (
  see [audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)).
- `output_format` (string): The output audio format (
  see [audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)).

### Subscribed Topics

- `audio_in` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The sound
  topic to convert.

### Published Topics

- `audio_out` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The
  converted sound.

## `resampling_node.py`

This node resamples an audio topic.

### Parameters

- `input_format` (string): The input audio format (
  see [audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)).
- `output_format` (string): The output audio format (
  see [audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)).
- `channel_count` (int): The device channel count.
- `input_sampling_frequency` (int): The input sampling frequency.
- `output_sampling_frequency` (int): The output sampling frequency.
- `input_frame_sample_count` (int): The number of samples in each frame of the input.
- `dynamic_input_resampling` (bool: default is `false`): If `true`, always adjust the input sampling informations (
  format, sampling frequency and frame sample count) to the sampling informations of the reveiced frames, dynamically.
  In this mode, `input_format`, `input_sampling_frequency` and `input_frame_sample_count` are not required, but they can
  be used to save a recomputation if the starting input sampling informations are known.

### Subscribed Topics

- `audio_in` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The sound
  topic to resample.

### Published Topics

- `audio_out` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The
  resampled sound.

## `split_channel_node.py`

This node split a multichannel audio topic into several mono audio topics.

### Parameters

- `input_format` (string): The input audio format (
  see [audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)).
- `output_format` (string): The output audio format (
  see [audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)).
- `channel_count` (int): The device channel count.

### Subscribed Topics

- `audio_in` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The sound
  topic to split.

### Published Topics

- `audio_out_0` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The
  first channel sound.
- `audio_out_1` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The
  second channel sound.
- ...

## `raw_file_writer_node.py`

This node writes the raw sound data to a file.

### Parameters

- `output_path` (string): The output file path.

### Subscribed Topics

- `audio_in` ([audio_utils/AudioFrame](https://github.com/introlab/audio_utils/blob/main/msg/AudioFrame.msg)) The sound
  topic to write.

# License

* [GPL-3.0 License ](LICENSE)

# Sponsor

![IntRoLab](https://introlab.3it.usherbrooke.ca/IntRoLab.png)

[IntRoLab - Intelligent / Interactive / Integrated / Interdisciplinary Robot Lab](https://introlab.3it.usherbrooke.ca)
