#include "PcmDevices/AlsaPcmDevice.h"
#include "PcmDevices/PulseAudioPcmDevice.h"
#include "Utils/Semaphore.h"

#include <MusicBeatDetector/Utils/Exception/InvalidValueException.h>

#include <rclcpp/rclcpp.hpp>

#include <audio_utils/msg/audio_frame.hpp>

#include <atomic>
#include <chrono>
#include <memory>

constexpr const char* NODE_NAME = "playback_node";

using namespace introlab;

class PlaybackNode : public rclcpp::Node
{
    PcmDevice::Backend m_backend;
    std::string m_device;
    std::string m_formatString;
    PcmAudioFrameFormat m_format;
    size_t m_channelCount;
    size_t m_samplingFrequency;
    size_t m_frameSampleCount;
    size_t m_latencyUs;
    std::vector<std::string> m_channelMap;

    std::unique_ptr<PcmDevice> m_playbackDevice;
    std::unique_ptr<PcmAudioFrame> m_emptyFrame;

    audio_utils::msg::AudioFrame::SharedPtr m_pendingFrame;
    Semaphore m_pendingFrameWriteSemaphore;
    Semaphore m_pendingFrameReadSemaphore;
    std::atomic<std::chrono::time_point<std::chrono::system_clock>> m_lastAudioFrameTime;
    std::chrono::nanoseconds m_frameDuration;

    rclcpp::Subscription<audio_utils::msg::AudioFrame>::SharedPtr m_audioSub;

public:
    explicit PlaybackNode()
        : rclcpp::Node(NODE_NAME),
          m_pendingFrameWriteSemaphore(1),
          m_pendingFrameReadSemaphore(0),
          m_lastAudioFrameTime(std::chrono::system_clock::now())
    {
        m_backend = PcmDevice::parseBackend(declare_parameter("backend", "alsa"));
        m_device = declare_parameter("device", "default");
        m_formatString = declare_parameter("format", "signed_16");
        m_format = parseFormat(m_formatString);
        m_channelCount = declare_parameter("channel_count", 1);
        m_samplingFrequency = declare_parameter("sampling_frequency", 16000);
        m_frameSampleCount = declare_parameter("frame_sample_count", 1024);
        m_latencyUs = declare_parameter("latency_us", 64000);
        m_channelMap = declare_parameter("channel_map", std::vector<std::string>{});
        if (!m_channelMap.empty() && m_backend != PcmDevice::Backend::PulseAudio)
        {
            RCLCPP_WARN(get_logger(), "The parameter channel_map is only supported with the PulseAudio backend");
        }

        m_frameDuration = std::chrono::milliseconds(1000 * m_frameSampleCount / m_samplingFrequency);

        m_playbackDevice = createPlaybackDevice();
        m_emptyFrame = std::make_unique<PcmAudioFrame>(m_format, m_channelCount, m_frameSampleCount);
        m_emptyFrame->clear();

        m_audioSub = create_subscription<audio_utils::msg::AudioFrame>(
            "audio_in",
            100,
            [this](const audio_utils::msg::AudioFrame::SharedPtr msg) { audioCallback(msg); });
    }

    void run()
    {
        std::thread spinThread([this]() { rclcpp::spin(shared_from_this()); });

        while (rclcpp::ok())
        {
            writeStep();
        }

        m_pendingFrameWriteSemaphore.release();
        spinThread.join();
    }

private:
    void audioCallback(const audio_utils::msg::AudioFrame::SharedPtr msg)
    {
        if (msg->format != m_formatString || msg->channel_count != m_channelCount ||
            msg->sampling_frequency != m_samplingFrequency || msg->frame_sample_count != m_frameSampleCount ||
            msg->data.size() != size(m_format, msg->channel_count, msg->frame_sample_count))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Not supported audio frame (msg->format=%s, msg->channel_count=%d,"
                "sampling_frequency=%d, frame_sample_count=%d, data_size=%ld)",
                msg->format.c_str(),
                msg->channel_count,
                msg->sampling_frequency,
                msg->frame_sample_count,
                msg->data.size());
            return;
        }

        m_pendingFrameWriteSemaphore.acquire();
        m_pendingFrame = msg;
        m_lastAudioFrameTime.store(std::chrono::system_clock::now());
        m_pendingFrameReadSemaphore.release();
    }

    void writeStep()
    {
        m_playbackDevice->wait();

        if (m_pendingFrameReadSemaphore.tryAcquireFor(m_frameDuration / 2))
        {
            PcmAudioFrame frame(m_format, m_channelCount, m_frameSampleCount, m_pendingFrame->data.data());
            m_playbackDevice->write(frame);
            m_pendingFrameWriteSemaphore.release();
        }
        else if ((std::chrono::system_clock::now() - m_lastAudioFrameTime.load()) > 2 * m_frameDuration)
        {
            m_playbackDevice->write(*m_emptyFrame);
        }
    }

    std::unique_ptr<PcmDevice> createPlaybackDevice()
    {
        switch (m_backend)
        {
            case PcmDevice::Backend::Alsa:
                return std::make_unique<AlsaPcmDevice>(
                    m_device,
                    PcmDevice::Stream::Playback,
                    m_format,
                    m_channelCount,
                    m_frameSampleCount,
                    m_samplingFrequency,
                    m_latencyUs);
            case PcmDevice::Backend::PulseAudio:
                return std::make_unique<PulseAudioPcmDevice>(
                    m_device,
                    PcmDevice::Stream::Playback,
                    m_format,
                    m_channelCount,
                    m_frameSampleCount,
                    m_samplingFrequency,
                    m_latencyUs,
                    m_channelMap);
            default:
                THROW_INVALID_VALUE_EXCEPTION("backend", "");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<PlaybackNode>();
        node->run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "%s", e.what());
        return -1;
    }

    rclcpp::shutdown();

    return 0;
}
