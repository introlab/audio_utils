#if defined(__unix__) || defined(__linux__)

#include "AlsaPcmDevice.h"

#include "../Utils/AlsaException.h"

#include <MusicBeatDetector/Utils/Exception/NotSupportedException.h>
#include <map>

using namespace introlab;
using namespace std;

class PcmParamsDeleter
{
public:
    void operator()(snd_pcm_hw_params_t* handle) { snd_pcm_hw_params_free(handle); }
};

AlsaPcmDevice::AlsaPcmDevice(
    const string& device,
    PcmDevice::Stream stream,
    PcmAudioFrameFormat format,
    size_t channelCount,
    size_t frameSampleCount,
    size_t sampleFrequency,
    unsigned int latencyUs)
    : PcmDevice(format, channelCount, frameSampleCount)
{
    int err;
    snd_pcm_t* pcmHandlePointer;
    snd_pcm_hw_params_t* paramsPointer;

    if ((err = snd_pcm_open(&pcmHandlePointer, device.c_str(), convert(stream), 0)) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot open audio device: " + device, err, snd_strerror(err));
    }

    unique_ptr<snd_pcm_t, PcmDeleter> pcmHandle(pcmHandlePointer);

    if ((err = snd_pcm_hw_params_malloc(&paramsPointer)) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot allocate hardware parameter structure", err, snd_strerror(err));
    }

    unique_ptr<snd_pcm_hw_params_t, PcmParamsDeleter> params(paramsPointer);

    if ((err = snd_pcm_hw_params_any(pcmHandle.get(), params.get())) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot initialize hardware parameter structure", err, snd_strerror(err));
    }

    if ((err = snd_pcm_hw_params_set_access(pcmHandle.get(), params.get(), SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot set access type", err, snd_strerror(err));
    }

    if ((err = snd_pcm_hw_params_set_format(pcmHandle.get(), params.get(), convert(format))) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot set sample format", err, snd_strerror(err));
    }

    if ((err = snd_pcm_hw_params_set_rate(pcmHandle.get(), params.get(), static_cast<int>(sampleFrequency), 0)) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot set sample rate", err, snd_strerror(err));
    }

    if ((err = snd_pcm_hw_params_set_channels(pcmHandle.get(), params.get(), static_cast<int>(channelCount))) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot set channel count", err, snd_strerror(err));
    }

    snd_pcm_uframes_t periodSize = static_cast<snd_pcm_uframes_t>(frameSampleCount);
    if ((err = snd_pcm_hw_params_set_period_size(pcmHandle.get(), params.get(), periodSize, 0)) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot set period size", err, snd_strerror(err));
    }

    if ((err = snd_pcm_hw_params_set_buffer_time_near(pcmHandle.get(), params.get(), &latencyUs, NULL)) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot set buffer time", err, snd_strerror(err));
    }

    if ((err = snd_pcm_hw_params(pcmHandle.get(), params.get())) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot set parameters", err, snd_strerror(err));
    }

    if ((err = snd_pcm_prepare(pcmHandle.get())) < 0)
    {
        THROW_ALSA_EXCEPTION("Cannot prepare audio interface for use", err, snd_strerror(err));
    }

    m_pcmHandle = move(pcmHandle);
}

AlsaPcmDevice::~AlsaPcmDevice() {}

void AlsaPcmDevice::read(PcmAudioFrame& frame)
{
    PcmDevice::read(frame);

    snd_pcm_uframes_t periodSize = static_cast<snd_pcm_uframes_t>(m_frameSampleCount);
    int err = snd_pcm_readi(m_pcmHandle.get(), frame.data(), periodSize);

    if (err == -EPIPE)
    {
        // EPIPE means overrun
        err = snd_pcm_recover(m_pcmHandle.get(), err, 1);
        if (err >= 0)
        {
            err = snd_pcm_readi(m_pcmHandle.get(), frame.data(), periodSize);
        }
    }

    if (err != periodSize)
    {
        THROW_ALSA_EXCEPTION("Read from audio interface failed", err, snd_strerror(err));
    }
}

void AlsaPcmDevice::write(const PcmAudioFrame& frame)
{
    PcmDevice::write(frame);

    snd_pcm_uframes_t periodSize = static_cast<snd_pcm_uframes_t>(m_frameSampleCount);
    int err = snd_pcm_writei(m_pcmHandle.get(), frame.data(), periodSize);

    if (err == -EPIPE)
    {
        // EPIPE means underrun
        err = snd_pcm_recover(m_pcmHandle.get(), err, 1);
        if (err >= 0)
        {
            err = snd_pcm_writei(m_pcmHandle.get(), frame.data(), periodSize);
        }
    }

    if (err != periodSize)
    {
        THROW_ALSA_EXCEPTION("Write to audio interface failed", err, snd_strerror(err));
    }
}

void AlsaPcmDevice::wait()
{
    int err = snd_pcm_wait(m_pcmHandle.get(), -1);
    if (err != 1)
    {
        THROW_ALSA_EXCEPTION("snd_pcm_wait failed", err, snd_strerror(err));
    }
}

snd_pcm_stream_t AlsaPcmDevice::convert(PcmDevice::Stream stream)
{
    switch (stream)
    {
        case PcmDevice::Stream::Playback:
            return SND_PCM_STREAM_PLAYBACK;
        case PcmDevice::Stream::Capture:
            return SND_PCM_STREAM_CAPTURE;
    }

    THROW_NOT_SUPPORTED_EXCEPTION("Not supported stream");
}

snd_pcm_format_t AlsaPcmDevice::convert(PcmAudioFrameFormat format)
{
    static const map<PcmAudioFrameFormat, snd_pcm_format_t> Mapping(
        {{PcmAudioFrameFormat::Signed8, SND_PCM_FORMAT_S8},
         {PcmAudioFrameFormat::Signed16, SND_PCM_FORMAT_S16_LE},
         {PcmAudioFrameFormat::Signed24, SND_PCM_FORMAT_S24_LE},
         {PcmAudioFrameFormat::Signed32, SND_PCM_FORMAT_S32_LE},

         {PcmAudioFrameFormat::Unsigned8, SND_PCM_FORMAT_U8},
         {PcmAudioFrameFormat::Unsigned16, SND_PCM_FORMAT_U16_LE},
         {PcmAudioFrameFormat::Unsigned24, SND_PCM_FORMAT_U24_LE},
         {PcmAudioFrameFormat::Unsigned32, SND_PCM_FORMAT_U32_LE},

         {PcmAudioFrameFormat::Float, SND_PCM_FORMAT_FLOAT_LE},
         {PcmAudioFrameFormat::Double, SND_PCM_FORMAT_FLOAT64_LE}});

    auto it = Mapping.find(format);
    if (it != Mapping.end())
    {
        return it->second;
    }

    THROW_NOT_SUPPORTED_EXCEPTION("Not supported format");
}

#endif
