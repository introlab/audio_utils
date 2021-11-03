#include "AlsaPcmDevice.h"
#include "Semaphore.h"

#include <audio_utils/AudioFrame.h>

#include <ros/ros.h>

#include <memory>
#include <chrono>
#include <atomic>

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

    unique_ptr<AlsaPcmDevice> m_playbackDevice;
    unique_ptr<PcmAudioFrame> m_emptyFrame;

    audio_utils::AudioFramePtr m_pendingFrame;
    Semaphore m_pendingFrameWriteSemaphore;
    Semaphore m_pendingFrameReadSemaphore;
    atomic<chrono::time_point<chrono::system_clock>> m_lastAudioFrameTime;
    chrono::nanoseconds m_frameDuration;

    ros::Subscriber m_audioSub;

public:
    AlsaPlaybackNode() :
        m_privateNodeHandle("~"),
        m_pendingFrameWriteSemaphore(1),
        m_pendingFrameReadSemaphore(0),
        m_lastAudioFrameTime(chrono::system_clock::now())
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
        m_privateNodeHandle.getParam("latency_us", tmp);
        unsigned int latencyUs = tmp;

        m_frameDuration = chrono::milliseconds(1000 * m_frameSampleCount / m_samplingFrequency);

        m_playbackDevice = make_unique<AlsaPcmDevice>(m_device, AlsaPcmDevice::Stream::Playback,
            m_format, m_channelCount, m_frameSampleCount, m_samplingFrequency, latencyUs);
        m_emptyFrame = make_unique<PcmAudioFrame>(m_format, m_channelCount, m_frameSampleCount);
        m_emptyFrame->clear();

        m_audioSub = m_nodeHandle.subscribe("audio_in", 100, &AlsaPlaybackNode::audioCallback, this);
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

        m_pendingFrameWriteSemaphore.acquire();
        m_pendingFrame = msg;
        m_lastAudioFrameTime.store(chrono::system_clock::now());
        m_pendingFrameReadSemaphore.release();
    }

    void run()
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();

        while (ros::ok())
        {
            writeStep();
        }

        m_pendingFrameWriteSemaphore.release();
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
        else if ((chrono::system_clock::now() - m_lastAudioFrameTime.load()) > 2 * m_frameDuration)
        {
            m_playbackDevice->write(*m_emptyFrame);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alsa_playback_node");

    AlsaPlaybackNode node;
    node.run();

    return 0;
}
