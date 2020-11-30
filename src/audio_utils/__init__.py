import numpy as np

from audio_utils.msg import AudioFrame


def get_format_information(format):
    if format == 'signed_8':
        return np.int8, 0, -np.iinfo(np.int8).min
    elif format == 'signed_16':
        return np.int16, 0, -np.iinfo(np.int16).min
    elif format == 'signed_24':
        raise NotImplementedError('signed_24 format is not supported')
    elif format == 'signed_padded_24':
        return np.int16, 0, 2**23
    elif format == 'signed_32':
        return np.int32, 0, -np.iinfo(np.int32).min

    elif format == 'unsigned_8':
        offset = np.iinfo(np.uint8).max // 2
        return np.uint8, -offset, offset 
    elif format == 'unsigned_16':
        offset = np.iinfo(np.uint16).max // 2
        return np.uint16, -offset, offset
    elif format == 'unsigned_24':
        raise NotImplementedError('unsigned_24 format is not supported')
    elif format == 'unsigned_padded_24':
        offset = (2**24 - 1) // 2
        return np.uint32, -offset, offset
    elif format == 'unsigned_32':
        offset = np.iinfo(np.uint32).max // 2
        return np.uint32, -offset, offset

    elif format == 'float':
        return np.float32, 0, 1
    elif format == 'double':
        return np.float64, 0, 1
    else:
        raise ValueError('Invalid format ({})'.format(format))


def convert_audio_data_to_numpy_frames(format_information, channel_count, data):
    dtype, offset, scale = format_information
    frame = (np.frombuffer(data, dtype=dtype).astype(np.float64) + offset) / scale

    if channel_count == 1:
        return [frame]
    else:
        return _split_frame_by_channels(channel_count, frame)


def _split_frame_by_channels(channel_count, frame):
    return [frame[i::channel_count] for i in range(channel_count)]


def convert_numpy_frames_to_audio_data(format_information, frames):    
    dtype, offset, scale = format_information

    if dtype == np.float32 or dtype == np.float64:
        min_value = -1.0
        max_value = 1.0
    else:
        min_value = np.iinfo(dtype).min
        max_value = np.iinfo(dtype).max

    frame = _merge_frames(frames) * scale - offset
    frame = np.clip(frame, a_min=min_value, a_max=max_value)
    return frame.astype(dtype).tobytes()


def _merge_frames(frames):
    frame = np.zeros(frames[0].shape[0] * len(frames), dtype=np.float64)
    for i in range(len(frames)):
        frame[i::len(frames)] = frames[i]
    return frame
