#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <MusicBeatDetector/MusicBeatDetector.h>

#include <audio_utils/AudioFrame.h>

using namespace introlab;
using namespace std;

constexpr size_t SupportedChannelCount = 1;
constexpr size_t SupportedSamplingFrequency = 44100;
constexpr size_t SupportedFrameSampleCount = 256;

class BeatDetectorNode
{
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_privateNodeHandle;

    ros::Subscriber m_audioSub;

    ros::Publisher m_bpmPub;
    ros::Publisher m_beatPub;

    std_msgs::Float32 m_bpmMsg;
    std_msgs::Bool m_beatMsg;

    MusicBeatDetector m_musicBeatDetector;

public:
    BeatDetectorNode() :
        m_privateNodeHandle("~"),
        m_musicBeatDetector(SupportedSamplingFrequency, SupportedFrameSampleCount)
    {
        m_audioSub = m_nodeHandle.subscribe("audio_in", 10, &BeatDetectorNode::audioCallback, this);

        m_bpmPub = m_nodeHandle.advertise<std_msgs::Float32>("bpm", 1000);
        m_beatPub = m_nodeHandle.advertise<std_msgs::Bool>("beat", 1000);
    }

    void audioCallback(const audio_utils::AudioFramePtr& msg)
    {
        if (msg->channel_count != SupportedChannelCount ||
            msg->sampling_frequency != SupportedSamplingFrequency ||
            msg->frame_sample_count != SupportedFrameSampleCount)
        {
            ROS_ERROR("Not supported audio frame (msg->channel_count=%d, sampling_frequency=%d, frame_sample_count=%d)",
                msg->channel_count, msg->sampling_frequency, msg->frame_sample_count);
            return;
        }

        PcmAudioFrame frame(parseFormat(msg->format), msg->channel_count, msg->frame_sample_count, msg->data.data());

        Beat beat = m_musicBeatDetector.detect(frame);
        m_bpmMsg.data = beat.bpm;
        m_beatMsg.data = beat.isBeat;

        m_bpmPub.publish(m_bpmMsg);
        m_beatPub.publish(m_beatMsg);
    }

    void run()
    {
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "beat_detector_node");

    BeatDetectorNode node;
    node.run();
 
    return 0;
}
