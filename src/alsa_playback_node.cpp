#include "AlsaPcmDevice.h"

#include <audio_utils/AudioFrame.h>

#include <ros/ros.h>

#include <memory>

using namespace introlab;
using namespace std;

class AlsaPlaybackNode
{
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_privateNodeHandle;

    string m_device;
    string m_formatString;
    size_t m_channelCount;
    size_t m_samplingFrequency;
    size_t m_frameSampleCount;

    PcmAudioFrameFormat m_format;

    unique_ptr<AlsaPcmDevice> m_captureDevice;


    ros::Subscriber m_audioSub;

public:
    AlsaPlaybackNode() :
        m_privateNodeHandle("~")
    {
        m_privateNodeHandle.getParam("device", m_device);
        m_privateNodeHandle.getParam("format", m_formatString);
        m_format = parseFormat(m_formatString);

        int tmp;
        m_privateNodeHandle.getParam("channel_count", tmp);
        m_channelCount = tmp;
        m_privateNodeHandle.getParam("sampling_frequency", tmp);
        m_samplingFrequency = tmp;
        m_privateNodeHandle.getParam("frame_sample_count", tmp);
        m_frameSampleCount = tmp;

        m_captureDevice = make_unique<AlsaPcmDevice>(m_device, AlsaPcmDevice::Stream::Playback, 
            m_format, m_channelCount, m_frameSampleCount, m_samplingFrequency);

        m_audioSub = m_nodeHandle.subscribe("audio_in", 10, &AlsaPlaybackNode::audioCallback, this);
    }

    void audioCallback(const audio_utils::AudioFramePtr& msg)
    {
        if (msg->format != m_formatString ||
            msg->channel_count != m_channelCount ||
            msg->sampling_frequency != m_samplingFrequency ||
            msg->frame_sample_count != m_frameSampleCount)
        {
            ROS_ERROR("Not supported audio frame (msg->format=%s, msg->channel_count=%d,"
                "sampling_frequency=%d, frame_sample_count=%d)",
                msg->format.c_str(), msg->channel_count, msg->sampling_frequency, msg->frame_sample_count);
            return;
        }

        PcmAudioFrame frame(m_format, m_channelCount, m_frameSampleCount, msg->data.data());
        m_captureDevice->write(frame);
    }

    void run()
    {
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alsa_playback_node");

    AlsaPlaybackNode node;
    node.run();
 
    return 0;
}
