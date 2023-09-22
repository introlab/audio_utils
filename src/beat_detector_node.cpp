#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <MusicBeatDetector/MusicBeatDetector.h>

#include <audio_utils/AudioFrame.h>

#include <memory>

using namespace introlab;
using namespace std;

constexpr size_t SupportedChannelCount = 1;

class BeatDetectorNode
{
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_privateNodeHandle;

    ros::Subscriber m_audioSub;

    ros::Publisher m_bpmPub;
    ros::Publisher m_beatPub;

    std_msgs::Float32 m_bpmMsg;
    std_msgs::Bool m_beatMsg;

    size_t m_samplingFrequency;
    size_t m_frameSampleCount;

    unique_ptr<MusicBeatDetector> m_musicBeatDetector;

public:
    BeatDetectorNode() : m_privateNodeHandle("~")
    {
        int samplingFrequency;
        int frameSampleCount;
        int ossFftWindowSize;
        int fluxHammingSize;
        int ossBpmWindowSize;
        float minBpm;
        float maxBpm;
        int bpmCandidateCount;

        m_privateNodeHandle.param<int>("sampling_frequency", samplingFrequency, 44100);
        m_privateNodeHandle.param<int>("frame_sample_count", frameSampleCount, 128);
        m_privateNodeHandle.param<int>("oss_fft_window_size", ossFftWindowSize, 1024);
        m_privateNodeHandle.param<int>("flux_hamming_size", fluxHammingSize, 15);
        m_privateNodeHandle.param<int>("oss_bpm_window_size", ossBpmWindowSize, 1024);
        m_privateNodeHandle.param<float>("min_bpm", minBpm, 50);
        m_privateNodeHandle.param<float>("max_bpm", maxBpm, 180);
        m_privateNodeHandle.param<int>("bpm_candidate_count", bpmCandidateCount, 10);

        m_samplingFrequency = static_cast<size_t>(samplingFrequency);
        m_frameSampleCount = static_cast<size_t>(frameSampleCount);

        m_musicBeatDetector = make_unique<MusicBeatDetector>(
            m_samplingFrequency,
            m_frameSampleCount,
            static_cast<size_t>(ossFftWindowSize),
            static_cast<size_t>(fluxHammingSize),
            static_cast<size_t>(ossBpmWindowSize),
            minBpm,
            maxBpm,
            static_cast<size_t>(bpmCandidateCount));

        m_audioSub = m_nodeHandle.subscribe("audio_in", 10, &BeatDetectorNode::audioCallback, this);

        m_bpmPub = m_nodeHandle.advertise<std_msgs::Float32>("bpm", 1000);
        m_beatPub = m_nodeHandle.advertise<std_msgs::Bool>("beat", 1000);
    }

    void run() { ros::spin(); }

private:
    void audioCallback(const audio_utils::AudioFramePtr& msg)
    {
        PcmAudioFrameFormat format = parseFormat(msg->format);
        if (msg->channel_count != SupportedChannelCount || msg->sampling_frequency != m_samplingFrequency ||
            (msg->frame_sample_count % m_frameSampleCount) != 0 ||
            msg->data.size() != size(format, msg->channel_count, msg->frame_sample_count))
        {
            ROS_ERROR(
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

        m_bpmPub.publish(m_bpmMsg);
        m_beatPub.publish(m_beatMsg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "beat_detector_node");

    BeatDetectorNode node;
    node.run();

    return 0;
}
