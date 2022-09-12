#ifndef PCM_DEVICES_PULSE_AUDIO_PCM_DEVICE_H
#define PCM_DEVICES_PULSE_AUDIO_PCM_DEVICE_H

#if defined(__unix__) || defined(__linux__)

#include "PcmDevice.h"

#include <MusicBeatDetector/Utils/ClassMacro.h>
#include <MusicBeatDetector/Utils/Data/PcmAudioFrame.h>

#include <pulse/simple.h>
#include <pulse/error.h>

#include <memory>
#include <string>
#include <vector>

namespace introlab
{
    class PulseAudioPcmDevice : public PcmDevice
    {
        class PaDeleter
        {
        public:
            void operator()(pa_simple* handle) { pa_simple_free(handle); }
        };

        std::unique_ptr<pa_simple, PaDeleter> m_paHandle;
        std::unique_ptr<pa_channel_map> m_channelMap;

    public:
        PulseAudioPcmDevice(
            const std::string& device,
            PcmDevice::Stream stream,
            PcmAudioFrameFormat format,
            std::size_t channelCount,
            std::size_t frameSampleCount,
            std::size_t sampleFrequency,
            uint64_t latencyUs,
            const std::vector<std::string>& channelMap);
        ~PulseAudioPcmDevice() override;

        DECLARE_NOT_COPYABLE(PulseAudioPcmDevice);
        DECLARE_NOT_MOVABLE(PulseAudioPcmDevice);

        void read(PcmAudioFrame& frame) override;
        void write(const PcmAudioFrame& frame) override;
        void wait() override;

    private:
        void setChannelMap(std::size_t channelCount, const std::vector<std::string>& channelMap);

        static pa_stream_direction_t convert(PcmDevice::Stream stream);
        static pa_sample_format_t convert(PcmAudioFrameFormat format);
    };
}

#else

#error "Invalid include file"

#endif

#endif
