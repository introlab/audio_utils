#include "PcmDevices/AlsaPcmDevice.h"
#include "PcmDevices/PulseAudioPcmDevice.h"
#include "Utils/Semaphore.h"

#include <MusicBeatDetector/Utils/Exception/InvalidValueException.h>

#include <audio_utils/AudioFrame.h>

#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <memory>

using namespace introlab;
using namespace std;

struct PlaybackNodeConfiguration
{
    PcmDevice::Backend backend;
    string backendString;
    string device;
    string formatString;
    PcmAudioFrameFormat format = PcmAudioFrameFormat::Signed16;
    int channelCount;
    int samplingFrequency;
    int frameSampleCount;
    int latencyUs;

    vector<string> channelMap;

    PlaybackNodeConfiguration()
        : backend(PcmDevice::Backend::Alsa),
          format(PcmAudioFrameFormat::Signed8),
          channelCount(0),
          samplingFrequency(0),
          frameSampleCount(0),
          latencyUs(0)
    {
    }
};

class PlaybackNode
{
    ros::NodeHandle m_nodeHandle;
    PlaybackNodeConfiguration m_configuration;

    unique_ptr<PcmDevice> m_playbackDevice;
    unique_ptr<PcmAudioFrame> m_emptyFrame;

    audio_utils::AudioFramePtr m_pendingFrame;
    Semaphore m_pendingFrameWriteSemaphore;
    Semaphore m_pendingFrameReadSemaphore;
    atomic<chrono::time_point<chrono::system_clock>> m_lastAudioFrameTime;
    chrono::nanoseconds m_frameDuration;

    ros::Subscriber m_audioSub;

public:
    explicit PlaybackNode(PlaybackNodeConfiguration configuration)
        : m_configuration(move(configuration)),
          m_pendingFrameWriteSemaphore(1),
          m_pendingFrameReadSemaphore(0),
          m_lastAudioFrameTime(chrono::system_clock::now())
    {
        m_frameDuration =
            chrono::milliseconds(1000 * m_configuration.frameSampleCount / m_configuration.samplingFrequency);

        m_playbackDevice = createPlaybackDevice();
        m_emptyFrame = make_unique<PcmAudioFrame>(
            m_configuration.format,
            m_configuration.channelCount,
            m_configuration.frameSampleCount);
        m_emptyFrame->clear();

        m_audioSub = m_nodeHandle.subscribe("audio_in", 100, &PlaybackNode::audioCallback, this);
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

private:
    void audioCallback(const audio_utils::AudioFramePtr& msg)
    {
        if (msg->format != m_configuration.formatString || msg->channel_count != m_configuration.channelCount ||
            msg->sampling_frequency != m_configuration.samplingFrequency ||
            msg->frame_sample_count != m_configuration.frameSampleCount ||
            msg->data.size() != size(m_configuration.format, msg->channel_count, msg->frame_sample_count))
        {
            ROS_ERROR(
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
        m_lastAudioFrameTime.store(chrono::system_clock::now());
        m_pendingFrameReadSemaphore.release();
    }

    void writeStep()
    {
        m_playbackDevice->wait();

        if (m_pendingFrameReadSemaphore.tryAcquireFor(m_frameDuration / 2))
        {
            PcmAudioFrame frame(
                m_configuration.format,
                m_configuration.channelCount,
                m_configuration.frameSampleCount,
                m_pendingFrame->data.data());
            m_playbackDevice->write(frame);
            m_pendingFrameWriteSemaphore.release();
        }
        else if ((chrono::system_clock::now() - m_lastAudioFrameTime.load()) > 2 * m_frameDuration)
        {
            m_playbackDevice->write(*m_emptyFrame);
        }
    }

    unique_ptr<PcmDevice> createPlaybackDevice()
    {
        switch (m_configuration.backend)
        {
            case PcmDevice::Backend::Alsa:
                return make_unique<AlsaPcmDevice>(
                    m_configuration.device,
                    PcmDevice::Stream::Playback,
                    m_configuration.format,
                    m_configuration.channelCount,
                    m_configuration.frameSampleCount,
                    m_configuration.samplingFrequency,
                    m_configuration.latencyUs);
            case PcmDevice::Backend::PulseAudio:
                return make_unique<PulseAudioPcmDevice>(
                    m_configuration.device,
                    PcmDevice::Stream::Playback,
                    m_configuration.format,
                    m_configuration.channelCount,
                    m_configuration.frameSampleCount,
                    m_configuration.samplingFrequency,
                    m_configuration.latencyUs,
                    m_configuration.channelMap);
            default:
                THROW_INVALID_VALUE_EXCEPTION("backend", "");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "playback_node");

    ros::NodeHandle privateNodeHandle("~");

    PlaybackNodeConfiguration configuration;

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

        PlaybackNode node(configuration);
        node.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
        return -1;
    }

    return 0;
}
