#include "PcmDevices/AlsaPcmDevice.h"
#include "PcmDevices/PulseAudioPcmDevice.h"

#include <MusicBeatDetector/Utils/Exception/InvalidValueException.h>

#include <audio_utils/msg/audio_frame.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace introlab;

struct CaptureNodeConfiguration
{
    PcmDevice::Backend backend;
    std::string backendString;
    std::string device;
    std::string formatString;
    PcmAudioFrameFormat format;
    int channelCount;
    int samplingFrequency;
    int frameSampleCount;
    int latencyUs;

    std::vector<std::string> channelMap;

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
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Clock m_clock;

    uint64_t m_samplingFrequency;
    uint64_t m_frameSampleCount;

    rclcpp::Duration m_tolerance;
    rclcpp::Duration m_minusTolerance;

    rclcpp::Time m_startTime;
    uint64_t m_sampleCount;

public:
    AudioFrameTimestampCalculator(std::shared_ptr<rclcpp::Node> node, int samplingFrequency, int frameSampleCount)
        : m_node(std::move(node)),
          m_samplingFrequency(samplingFrequency),
          m_frameSampleCount(frameSampleCount),
          m_tolerance(std::chrono::milliseconds(500)),
          m_minusTolerance(std::chrono::milliseconds(-500)),
          m_startTime(m_clock.now()),
          m_sampleCount(0)
    {
    }

    rclcpp::Time next()
    {
        m_sampleCount += m_frameSampleCount;
        rclcpp::Time timestamp = m_startTime + sampleCountToDuration(m_samplingFrequency, m_sampleCount);
        resetIfOutOfTolerance(timestamp);

        return timestamp;
    }

private:
    void resetIfOutOfTolerance(rclcpp::Time& timestamp)
    {
        rclcpp::Time now = m_clock.now();
        rclcpp::Duration difference = now - timestamp;

        if (difference > m_tolerance || difference < m_minusTolerance)
        {
            timestamp = now;
            m_startTime = now;
            m_sampleCount = 0;
            RCLCPP_WARN(m_node->get_logger(), "The audio frame timestamp calculator has been reset.");
        }
    }

    static rclcpp::Duration sampleCountToDuration(uint64_t samplingFrequency, uint64_t sampleCount)
    {
        constexpr uint64_t NsecsPerSec = 1'000'000'000;
        uint32_t sec = sampleCount / samplingFrequency;
        uint32_t nsec = (sampleCount % samplingFrequency) * NsecsPerSec / samplingFrequency;

        return rclcpp::Duration(static_cast<int32_t>(sec), static_cast<int32_t>(nsec));
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

std::unique_ptr<PcmDevice> createCaptureDevice(const CaptureNodeConfiguration& configuration)
{
    switch (configuration.backend)
    {
        case PcmDevice::Backend::Alsa:
            return std::make_unique<AlsaPcmDevice>(
                configuration.device,
                PcmDevice::Stream::Capture,
                configuration.format,
                configuration.channelCount,
                configuration.frameSampleCount,
                configuration.samplingFrequency,
                configuration.latencyUs);
        case PcmDevice::Backend::PulseAudio:
            return std::make_unique<PulseAudioPcmDevice>(
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

void run(
    std::shared_ptr<rclcpp::Node>& node,
    std::unique_ptr<PcmDevice> captureDevice,
    const CaptureNodeConfiguration& configuration,
    rclcpp::Publisher<audio_utils::msg::AudioFrame>::SharedPtr& audioPub)
{
    PcmAudioFrame manyChannelPcmFrame(configuration.format, configuration.channelCount, configuration.frameSampleCount);
    PcmAudioFrame oneChannelPcmFrame(configuration.format, 1, configuration.frameSampleCount);
    PackedAudioFrame<float> manyChannelFrame(configuration.channelCount, configuration.frameSampleCount);
    PackedAudioFrame<float> oneChannelFrame(1, configuration.frameSampleCount);

    audio_utils::msg::AudioFrame audioFrameMsg;
    audioFrameMsg.format = configuration.formatString;
    audioFrameMsg.channel_count = configuration.merge ? 1 : configuration.channelCount;
    audioFrameMsg.sampling_frequency = configuration.samplingFrequency;
    audioFrameMsg.frame_sample_count = configuration.frameSampleCount;
    audioFrameMsg.data.resize(configuration.merge ? oneChannelPcmFrame.size() : manyChannelPcmFrame.size());

    AudioFrameTimestampCalculator timestampCalculator(
        node,
        configuration.samplingFrequency,
        configuration.frameSampleCount);

    while (rclcpp::ok())
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
        audioPub->publish(audioFrameMsg);

        rclcpp::spin_some(node);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("capture_node");
    auto audioPub = node->create_publisher<audio_utils::msg::AudioFrame>("audio_out", 100);

    CaptureNodeConfiguration configuration;
    configuration.backendString = node->declare_parameter("backend", "alsa");

    try
    {
        configuration.backend = PcmDevice::parseBackend(configuration.backendString);

        configuration.device = node->declare_parameter("device", "default");
        configuration.formatString = node->declare_parameter("format", "signed_16");
        configuration.format = parseFormat(configuration.formatString);
        configuration.channelCount = node->declare_parameter("channel_count", 1);
        configuration.samplingFrequency = node->declare_parameter("sampling_frequency", 16000);
        configuration.frameSampleCount = node->declare_parameter("frame_sample_count", 1024);
        configuration.latencyUs = node->declare_parameter("latency_us", 64000);
        configuration.channelMap = node->declare_parameter("channel_map", std::vector<std::string>{});
        if (!configuration.channelMap.empty() && configuration.backend != PcmDevice::Backend::PulseAudio)
        {
            RCLCPP_WARN(node->get_logger(), "The parameter channel_map is only supported with the PulseAudio backend");
        }

        configuration.merge = node->declare_parameter("merge", false);
        configuration.gain = node->declare_parameter("gain", 1.f);

        run(node, createCaptureDevice(configuration), configuration, audioPub);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "%s", e.what());
        return -1;
    }

    rclcpp::shutdown();

    return 0;
}
