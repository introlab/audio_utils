#ifndef ALSA_PCM_DEVICE_H
#define ALSA_PCM_DEVICE_H

#if defined(__unix__) || defined(__linux__)

#include <Utils/Data/PcmAudioFrame.h>
#include <Utils/ClassMacro.h>

#include <alsa/asoundlib.h>

#include <memory>
#include <string>

namespace introlab
{
    class AlsaPcmDevice
    {
    public:
        enum class Stream
        {
            Playback = SND_PCM_STREAM_PLAYBACK,
            Capture = SND_PCM_STREAM_CAPTURE
        };

    private:
        class PcmDeleter
        {
        public:
            void operator()(snd_pcm_t* handle)
            {
                snd_pcm_close(handle);
            }
        };

        PcmAudioFrameFormat m_format;
        std::size_t m_channelCount;
        std::size_t m_frameSampleCount;
        std::unique_ptr<snd_pcm_t, PcmDeleter> m_pcmHandle;

    public:
        AlsaPcmDevice(const std::string& device,
            Stream stream,
            PcmAudioFrameFormat format,
            std::size_t channelCount,
            std::size_t frameSampleCount,
            std::size_t sampleFrequency,
            unsigned int latencyUs);
        ~AlsaPcmDevice();

        DECLARE_NOT_COPYABLE(AlsaPcmDevice);
        DECLARE_NOT_MOVABLE(AlsaPcmDevice);

        bool read(PcmAudioFrame& frame);
        void write(const PcmAudioFrame& frame);
        void wait();

    private:
        static snd_pcm_format_t convert(PcmAudioFrameFormat format);
    };
}

#else

#error "Invalid include file"

#endif

#endif
