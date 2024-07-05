#if defined(__unix__) || defined(__linux__)

#include "PcmDevice.h"

#include <MusicBeatDetector/Utils/Exception/InvalidValueException.h>

using namespace introlab;
using namespace std;

PcmDevice::PcmDevice(PcmAudioFrameFormat format, size_t channelCount, size_t frameSampleCount)
    : m_format(format),
      m_channelCount(channelCount),
      m_frameSampleCount(frameSampleCount)
{
}

PcmDevice::~PcmDevice() {}

void PcmDevice::read(PcmAudioFrame& frame)
{
    if (frame.format() != m_format || frame.channelCount() != m_channelCount ||
        frame.sampleCount() != m_frameSampleCount)
    {
        THROW_INVALID_VALUE_EXCEPTION("format, channelCount, sampleCount", "");
    }
}

void PcmDevice::write(const PcmAudioFrame& frame)
{
    if (frame.format() != m_format || frame.channelCount() != m_channelCount ||
        frame.sampleCount() != m_frameSampleCount)
    {
        THROW_INVALID_VALUE_EXCEPTION("format, channelCount, sampleCount", "");
    }
}

PcmDevice::Backend PcmDevice::parseBackend(const string& backend)
{
    if (backend == "alsa")
    {
        return PcmDevice::Backend::Alsa;
    }
    else if (backend == "pulse_audio")
    {
        return PcmDevice::Backend::PulseAudio;
    }

    THROW_INVALID_VALUE_EXCEPTION("backend", backend);
}

#endif
