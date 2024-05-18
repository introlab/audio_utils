#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <MusicBeatDetector/MusicBeatDetector.h>

#include <audio_utils/msg/audio_frame.hpp>

#include <memory>

using namespace introlab;

constexpr size_t SupportedChannelCount = 1;

class BeatDetectorNode : public rclcpp::Node
{
    rclcpp::Subscription<audio_utils::msg::AudioFrame>::SharedPtr m_audioSub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_bpmPub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_beatPub;

    std_msgs::msg::Float32 m_bpmMsg;
    std_msgs::msg::Bool m_beatMsg;

    size_t m_samplingFrequency;
    size_t m_frameSampleCount;

    std::unique_ptr<MusicBeatDetector> m_musicBeatDetector;

public:
    BeatDetectorNode() : rclcpp::Node("beat_detector_node")
    {
        m_samplingFrequency = declare_parameter("sampling_frequency", 44100);
        m_frameSampleCount = declare_parameter("frame_sample_count", 128);
        size_t ossFftWindowSize = declare_parameter("oss_fft_window_size", 1024);
        size_t fluxHammingSize = declare_parameter("flux_hamming_size", 15);
        size_t ossBpmWindowSize = declare_parameter("oss_bpm_window_size", 1024);
        float minBpm = declare_parameter("min_bpm", 50.f);
        float maxBpm = declare_parameter("min_bpm", 180.f);
        size_t bpmCandidateCount = declare_parameter("min_bpm", 10);

        m_musicBeatDetector = std::make_unique<MusicBeatDetector>(
            m_samplingFrequency,
            m_frameSampleCount,
            ossFftWindowSize,
            fluxHammingSize,
            ossBpmWindowSize,
            minBpm,
            maxBpm,
            bpmCandidateCount);

        m_audioSub = create_subscription<audio_utils::msg::AudioFrame>(
            "audio_in",
            10,
            [this] (const audio_utils::msg::AudioFrame::SharedPtr msg) { audioCallback(msg); });

        m_bpmPub = create_publisher<std_msgs::msg::Float32>("bpm", 1000);
        m_beatPub = create_publisher<std_msgs::msg::Bool>("beat", 1000);
    }

private:
    void audioCallback(const audio_utils::msg::AudioFrame::SharedPtr msg)
    {
        PcmAudioFrameFormat format = parseFormat(msg->format);
        if (msg->channel_count != SupportedChannelCount || msg->sampling_frequency != m_samplingFrequency ||
            (msg->frame_sample_count % m_frameSampleCount) != 0 ||
            msg->data.size() != size(format, msg->channel_count, msg->frame_sample_count))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Not supported audio frame (msg->channel_count=%d, "
                "sampling_frequency=%d, frame_sample_count=%d, data_size=%ld)",
                msg->channel_count,
                msg->sampling_frequency,
                msg->frame_sample_count,
                msg->data.size());
            return;
        }

        PcmAudioFrame frame(format, msg->channel_count, msg->frame_sample_count, msg->data.data());

        m_beatMsg.data = false;
        for (size_t i = 0; i < msg->frame_sample_count; i += m_frameSampleCount)
        {
            Beat beat = m_musicBeatDetector->detect(frame.slice(i, m_frameSampleCount));
            m_bpmMsg.data = beat.bpm;
            m_beatMsg.data = m_beatMsg.data || beat.isBeat;
        }

        m_bpmPub->publish(m_bpmMsg);
        m_beatPub->publish(m_beatMsg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BeatDetectorNode>());
    rclcpp::shutdown();

    return 0;
}
