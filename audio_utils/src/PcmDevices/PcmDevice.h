#ifndef PCM_DEVICES_PCM_DEVICE_H
#define PCM_DEVICES_PCM_DEVICE_H

#if defined(__unix__) || defined(__linux__)

#include <MusicBeatDetector/Utils/ClassMacro.h>
#include <MusicBeatDetector/Utils/Data/PcmAudioFrame.h>

namespace introlab
{
    class PcmDevice
    {
    public:
        enum class Stream
        {
            Playback,
            Capture
        };

        enum class Backend
        {
            Alsa,
            PulseAudio
        };

    protected:
        PcmAudioFrameFormat m_format;
        std::size_t m_channelCount;
        std::size_t m_frameSampleCount;

    public:
        PcmDevice(PcmAudioFrameFormat format, std::size_t channelCount, std::size_t frameSampleCount);
        virtual ~PcmDevice();

        DECLARE_NOT_COPYABLE(PcmDevice);
        DECLARE_NOT_MOVABLE(PcmDevice);

        virtual void read(PcmAudioFrame& frame);
        virtual void write(const PcmAudioFrame& frame);
        virtual void wait() = 0;

        static Backend parseBackend(const std::string& backend);
    };
}

#else

#error "Invalid include file"

#endif

#endif
