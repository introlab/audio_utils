#include "AlsaPcmDevice.h"

#include <audio_utils/AudioFrame.h>

#include <ros/ros.h>

#include <memory>
#include <deque>
#include <mutex>
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
    size_t m_frameQueueSize;

    unique_ptr<AlsaPcmDevice> m_playbackDevice;
    unique_ptr<PcmAudioFrame> m_emptyFrame;

    deque<audio_utils::AudioFramePtr> m_queue;
    bool m_started;
    mutex m_mutex;

    atomic<chrono::time_point<chrono::system_clock>> m_lastAudioFrameTime;
    chrono::nanoseconds m_frameDuration;

    ros::Subscriber m_audioSub;

public:
    AlsaPlaybackNode() :
        m_privateNodeHandle("~"),
        m_started(false),
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

        m_privateNodeHandle.getParam("frame_queue_size", tmp);
        m_frameQueueSize = tmp;

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

        lock_guard<mutex> lock(m_mutex);
        m_queue.push_back(msg);
        m_lastAudioFrameTime.store(chrono::system_clock::now());
    }

    void run()
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();

        while (ros::ok())
        {
            writeStep();
        }
    }

    void writeStep()
    {
        m_playbackDevice->wait();
        auto duration = chrono::system_clock::now().time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

        size_t queueSize;
        {
            lock_guard<mutex> lock(m_mutex);
            queueSize = m_queue.size();
        }

        if ((chrono::system_clock::now() - m_lastAudioFrameTime.load()) > 2 * m_frameDuration || queueSize == 0)
        {
            m_started = false;
        }
        if (!m_started && queueSize >= m_frameQueueSize)
        {
            m_started = true;
        }

        if (m_started)
        {
            writeNextFrame();
        }
        else
        {
            writeEmptyFrame();
        }
    }

    void writeNextFrame()
    {
        audio_utils::AudioFramePtr msg;
        {
            lock_guard<mutex> lock(m_mutex);
            msg = m_queue.front();
            m_queue.pop_front();
        }
        PcmAudioFrame frame(m_format, m_channelCount, m_frameSampleCount, msg->data.data());
        m_playbackDevice->write(frame);
    }

    void writeEmptyFrame()
    {
        m_playbackDevice->write(*m_emptyFrame);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alsa_playback_node");

    AlsaPlaybackNode node;
    node.run();

    return 0;
}
