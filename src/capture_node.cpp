#include "PcmDevices/AlsaPcmDevice.h"
#include "PcmDevices/PulseAudioPcmDevice.h"

#include <MusicBeatDetector/Utils/Exception/InvalidValueException.h>

#include <audio_utils/AudioFrame.h>

#include <ros/ros.h>

using namespace introlab;
using namespace std;

struct CaptureNodeConfiguration
{
    PcmDevice::Backend backend;
    string backendString;
    string device;
    string formatString;
    PcmAudioFrameFormat format;
    int channelCount;
    int samplingFrequency;
    int frameSampleCount;
    int latencyUs;

    vector<string> channelMap;

    bool merge;
    float gain;

    CaptureNodeConfiguration()
        : backend(PcmDevice::Backend::Alsa),
          format(PcmAudioFrameFormat::Signed8),
          channelCount(0),
          samplingFrequency(0),
          frameSampleCount(0),
          latencyUs(0),
          merge(false),
          gain(1.f)
    {
    }
};

class AudioFrameTimestampCalculator
{
    uint64_t m_samplingFrequency;
    uint64_t m_frameSampleCount;

    ros::Duration m_tolerance;

    ros::Time m_startTime;
    uint64_t m_sampleCount;

public:
    AudioFrameTimestampCalculator(int samplingFrequency, int frameSampleCount)
        : m_samplingFrequency(samplingFrequency),
          m_frameSampleCount(frameSampleCount),
          m_tolerance(0.5),
          m_startTime(ros::Time::now()),
          m_sampleCount(0)
    {
    }

    ros::Time next()
    {
        m_sampleCount += m_frameSampleCount;
        ros::Time timestamp = m_startTime + sampleCountToDuration(m_samplingFrequency, m_sampleCount);
        resetIfOutOfTolerance(timestamp);

        return timestamp;
    }

private:
    void resetIfOutOfTolerance(ros::Time& timestamp)
    {
        ros::Time now = ros::Time::now();
        ros::Duration difference = now - timestamp;

        if (difference > m_tolerance || difference < -m_tolerance)
        {
            timestamp = now;
            m_startTime = now;
            m_sampleCount = 0;
            ROS_WARN("The audio frame timestamp calculator has been reset.");
        }
    }

    static ros::Duration sampleCountToDuration(uint64_t samplingFrequency, uint64_t sampleCount)
    {
        constexpr uint64_t NsecsPerSec = 1'000'000'000;
        uint32_t sec = sampleCount / samplingFrequency;
        uint32_t nsec = (sampleCount % samplingFrequency) * NsecsPerSec / samplingFrequency;

        return ros::Duration(static_cast<int32_t>(sec), static_cast<int32_t>(nsec));
    }
};

void mergeChannels(
    const PcmAudioFrame& pcmInput,
    PcmAudioFrame& pcmOutput,
    PackedAudioFrame<float>& input,
    PackedAudioFrame<float>& output,
    float gain)
{
    pcmInput.copyTo(input);
    if (output.channelCount() != 1 || output.sampleCount() != input.sampleCount())
    {
        output = PackedAudioFrame<float>(1, input.sampleCount());
    }

    for (size_t sample = 0; sample < input.sampleCount(); sample++)
    {
        output[sample] = 0;
        for (size_t channel = 0; channel < input.channelCount(); channel++)
        {
            output[sample] += input[channel * input.sampleCount() + sample];
        }

        output[sample] /= input.channelCount();
        output[sample] *= gain;
    }

    pcmOutput = output;
}

void applyGain(PcmAudioFrame& pcmFrame, PackedAudioFrame<float>& frame, float gain)
{
    if (gain == 1.f)
    {
        return;
    }

    pcmFrame.copyTo(frame);
    for (size_t i = 0; i < frame.size(); i++)
    {
        frame[i] *= gain;
    }
    pcmFrame = frame;
}

unique_ptr<PcmDevice> createCaptureDevice(const CaptureNodeConfiguration& configuration)
{
    switch (configuration.backend)
    {
        case PcmDevice::Backend::Alsa:
            return make_unique<AlsaPcmDevice>(
                configuration.device,
                PcmDevice::Stream::Capture,
                configuration.format,
                configuration.channelCount,
                configuration.frameSampleCount,
                configuration.samplingFrequency,
                configuration.latencyUs);
        case PcmDevice::Backend::PulseAudio:
            return make_unique<PulseAudioPcmDevice>(
                configuration.device,
                PcmDevice::Stream::Capture,
                configuration.format,
                configuration.channelCount,
                configuration.frameSampleCount,
                configuration.samplingFrequency,
                configuration.latencyUs,
                configuration.channelMap);
        default:
            THROW_INVALID_VALUE_EXCEPTION("backend", "");
    }
}

void run(unique_ptr<PcmDevice> captureDevice, const CaptureNodeConfiguration& configuration, ros::Publisher& audioPub)
{
    PcmAudioFrame manyChannelPcmFrame(configuration.format, configuration.channelCount, configuration.frameSampleCount);
    PcmAudioFrame oneChannelPcmFrame(configuration.format, 1, configuration.frameSampleCount);
    PackedAudioFrame<float> manyChannelFrame(configuration.channelCount, configuration.frameSampleCount);
    PackedAudioFrame<float> oneChannelFrame(1, configuration.frameSampleCount);

    audio_utils::AudioFrame audioFrameMsg;
    audioFrameMsg.format = configuration.formatString;
    audioFrameMsg.channel_count = configuration.merge ? 1 : configuration.channelCount;
    audioFrameMsg.sampling_frequency = configuration.samplingFrequency;
    audioFrameMsg.frame_sample_count = configuration.frameSampleCount;
    audioFrameMsg.data.resize(configuration.merge ? oneChannelPcmFrame.size() : manyChannelPcmFrame.size());

    AudioFrameTimestampCalculator timestampCalculator(configuration.samplingFrequency, configuration.frameSampleCount);

    while (ros::ok())
    {
        captureDevice->read(manyChannelPcmFrame);

        if (configuration.merge)
        {
            mergeChannels(
                manyChannelPcmFrame,
                oneChannelPcmFrame,
                manyChannelFrame,
                oneChannelFrame,
                configuration.gain);
            memcpy(audioFrameMsg.data.data(), oneChannelPcmFrame.data(), audioFrameMsg.data.size());
        }
        else
        {
            applyGain(manyChannelPcmFrame, manyChannelFrame, configuration.gain);
            memcpy(audioFrameMsg.data.data(), manyChannelPcmFrame.data(), audioFrameMsg.data.size());
        }

        audioFrameMsg.header.stamp = timestampCalculator.next();
        audioPub.publish(audioFrameMsg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "capture_node");

    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");

    ros::Publisher audioPub = nodeHandle.advertise<audio_utils::AudioFrame>("audio_out", 100);

    CaptureNodeConfiguration configuration;

    if (!privateNodeHandle.getParam("backend", configuration.backendString))
    {
        ROS_ERROR("The parameter backend must be alsa or pulse_audio.");
        return -1;
    }

    try
    {
        configuration.backend = PcmDevice::parseBackend(configuration.backendString);

        if (!privateNodeHandle.getParam("device", configuration.device))
        {
            ROS_ERROR("The parameter device is required.");
            return -1;
        }
        if (!privateNodeHandle.getParam("format", configuration.formatString))
        {
            ROS_ERROR("The parameter format is required.");
            return -1;
        }
        configuration.format = parseFormat(configuration.formatString);

        if (!privateNodeHandle.getParam("channel_count", configuration.channelCount))
        {
            ROS_ERROR("The parameter channel_count is required.");
            return -1;
        }
        if (!privateNodeHandle.getParam("sampling_frequency", configuration.samplingFrequency))
        {
            ROS_ERROR("The parameter sampling_frequency is required.");
            return -1;
        }
        if (!privateNodeHandle.getParam("frame_sample_count", configuration.frameSampleCount))
        {
            ROS_ERROR("The parameter frame_sample_count is required.");
            return -1;
        }
        if (!privateNodeHandle.getParam("latency_us", configuration.latencyUs))
        {
            ROS_ERROR("The parameter latency_us is required.");
            return -1;
        }

        bool channelMapFound = privateNodeHandle.getParam("channel_map", configuration.channelMap);
        if (channelMapFound && configuration.backend != PcmDevice::Backend::PulseAudio)
        {
            ROS_WARN("The parameter channel_map is only supported with the PulseAudio backend");
        }

        configuration.merge = privateNodeHandle.param("merge", false);
        configuration.gain = privateNodeHandle.param("gain", 1.f);

        run(createCaptureDevice(configuration), configuration, audioPub);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
        return -1;
    }

    return 0;
}
