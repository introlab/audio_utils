#if defined(__unix__) || defined(__linux__)

#include "PulseAudioPcmDevice.h"

#include "../Utils/PulseAudioException.h"

#include <MusicBeatDetector/Utils/Exception/InvalidValueException.h>
#include <MusicBeatDetector/Utils/Exception/NotSupportedException.h>
#include <map>

using namespace introlab;
using namespace std;

PulseAudioPcmDevice::PulseAudioPcmDevice(
    const string& device,
    PcmDevice::Stream stream,
    PcmAudioFrameFormat format,
    size_t channelCount,
    size_t frameSampleCount,
    size_t sampleFrequency,
    uint64_t latencyUs,
    const vector<string>& channelMap)
    : PcmDevice(format, channelCount, frameSampleCount)
{
    pa_sample_spec ss;
    ss.format = convert(format);
    ss.rate = static_cast<uint32_t>(sampleFrequency);
    ss.channels = static_cast<uint8_t>(channelCount);

    setChannelMap(channelCount, channelMap);

    pa_buffer_attr ba;
    ba.maxlength = -1;
    ba.tlength = (stream == PcmDevice::Stream::Playback) ? pa_usec_to_bytes(latencyUs, &ss) : -1;
    ba.prebuf = -1;
    ba.minreq = -1;
    ba.fragsize = (stream == PcmDevice::Stream::Capture) ? pa_usec_to_bytes(latencyUs, &ss) : -1;

    int error = 0;

    m_paHandle = unique_ptr<pa_simple, PaDeleter>(pa_simple_new(
        nullptr,
        "audio_utils",
        convert(stream),
        device.c_str(),
        "audio_utils",
        &ss,
        m_channelMap.get(),
        &ba,
        &error));
    if (!m_paHandle)
    {
        THROW_PULSE_AUDIO_EXCEPTION("Cannot open audio device: " + device, error, pa_strerror(error));
    }
}

PulseAudioPcmDevice::~PulseAudioPcmDevice() {}

void PulseAudioPcmDevice::read(PcmAudioFrame& frame)
{
    PcmDevice::read(frame);

    int error = 0;
    if (pa_simple_read(m_paHandle.get(), frame.data(), frame.size(), &error) < 0)
    {
        THROW_PULSE_AUDIO_EXCEPTION("Read from pulse audio failed", error, pa_strerror(error));
    }
}

void PulseAudioPcmDevice::write(const PcmAudioFrame& frame)
{
    PcmDevice::write(frame);

    int error = 0;
    if (pa_simple_write(m_paHandle.get(), frame.data(), frame.size(), &error) < 0)
    {
        THROW_PULSE_AUDIO_EXCEPTION("Write to pulse audio failed", error, pa_strerror(error));
    }
}

void PulseAudioPcmDevice::wait() {}

void PulseAudioPcmDevice::setChannelMap(size_t channelCount, const vector<string>& channelMap)
{
    if (channelMap.empty())
    {
        return;
    }
    else if (channelMap.size() != channelCount)
    {
        THROW_INVALID_VALUE_EXCEPTION("channelMap", "");
    }

    m_channelMap = make_unique<pa_channel_map>();
    pa_channel_map_init(m_channelMap.get());

    m_channelMap->channels = channelCount;
    for (size_t i = 0; i < channelCount; i++)
    {
        m_channelMap->map[i] = pa_channel_position_from_string(channelMap[i].c_str());
        if (m_channelMap->map[i] == PA_CHANNEL_POSITION_INVALID)
        {
            THROW_INVALID_VALUE_EXCEPTION("channelMap[" + to_string(i) + "]", channelMap[i]);
        }
    }

    if (!pa_channel_map_valid(m_channelMap.get()))
    {
        THROW_PULSE_AUDIO_EXCEPTION("Not valid channel map", 0, "");
    }
}

pa_stream_direction_t PulseAudioPcmDevice::convert(PcmDevice::Stream stream)
{
    switch (stream)
    {
        case PcmDevice::Stream::Playback:
            return PA_STREAM_PLAYBACK;
        case PcmDevice::Stream::Capture:
            return PA_STREAM_RECORD;
    }

    THROW_NOT_SUPPORTED_EXCEPTION("Not supported stream");
}

pa_sample_format_t PulseAudioPcmDevice::convert(PcmAudioFrameFormat format)
{
    static const map<PcmAudioFrameFormat, pa_sample_format_t> Mapping(
        {{PcmAudioFrameFormat::Signed16, PA_SAMPLE_S16LE},
         {PcmAudioFrameFormat::Signed24, PA_SAMPLE_S24LE},
         {PcmAudioFrameFormat::SignedPadded24, PA_SAMPLE_S24_32LE},
         {PcmAudioFrameFormat::Signed32, PA_SAMPLE_S32LE},

         {PcmAudioFrameFormat::Unsigned8, PA_SAMPLE_U8},

         {PcmAudioFrameFormat::Float, PA_SAMPLE_FLOAT32LE}});

    auto it = Mapping.find(format);
    if (it != Mapping.end())
    {
        return it->second;
    }

    THROW_NOT_SUPPORTED_EXCEPTION("Not supported format");
}

#endif
