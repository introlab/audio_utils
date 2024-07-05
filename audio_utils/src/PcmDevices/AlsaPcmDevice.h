#ifndef PCM_DEVICES_ALSA_PCM_DEVICE_H
#define PCM_DEVICES_ALSA_PCM_DEVICE_H

#if defined(__unix__) || defined(__linux__)

#include "PcmDevice.h"

#include <MusicBeatDetector/Utils/ClassMacro.h>
#include <MusicBeatDetector/Utils/Data/PcmAudioFrame.h>

#include <alsa/asoundlib.h>

#include <memory>
#include <string>

namespace introlab
{
    class AlsaPcmDevice : public PcmDevice
    {
        class PcmDeleter
        {
        public:
            void operator()(snd_pcm_t* handle) { snd_pcm_close(handle); }
        };

        std::unique_ptr<snd_pcm_t, PcmDeleter> m_pcmHandle;

    public:
        AlsaPcmDevice(
            const std::string& device,
            PcmDevice::Stream stream,
            PcmAudioFrameFormat format,
            std::size_t channelCount,
            std::size_t frameSampleCount,
            std::size_t sampleFrequency,
            unsigned int latencyUs);
        ~AlsaPcmDevice() override;

        DECLARE_NOT_COPYABLE(AlsaPcmDevice);
        DECLARE_NOT_MOVABLE(AlsaPcmDevice);

        void read(PcmAudioFrame& frame) override;
        void write(const PcmAudioFrame& frame) override;
        void wait() override;

    private:
        static snd_pcm_stream_t convert(PcmDevice::Stream stream);
        static snd_pcm_format_t convert(PcmAudioFrameFormat format);
    };
}

#else

#error "Invalid include file"

#endif

#endif
