#include "PcmDevices/AlsaPcmDevice.h"
#include "PcmDevices/PulseAudioPcmDevice.h"

#include <Utils/Exception/InvalidValueException.h>

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
    PcmAudioFrameFormat format = PcmAudioFrameFormat::Signed16;
    int channelCount;
    int samplingFrequency;
    int frameSampleCount;
    int latencyUs;

    vector<string> channelMap;

    bool merge;
    float gain;

    CaptureNodeConfiguration()
        : channelCount(0),
          samplingFrequency(0),
          frameSampleCount(0),
          latencyUs(0),
          merge(false),
          gain(1.f)
    {
    }
};

void mergeChannels(
    const PcmAudioFrame& pcmInput,
    PcmAudioFrame& pcmOutput,
    AudioFrame<float>& input,
    AudioFrame<float>& output,
    float gain)
{
    pcmInput.copyTo(input);
    if (output.channelCount() != 1 || output.sampleCount() != input.sampleCount())
    {
        output = AudioFrame<float>(1, input.sampleCount());
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

void applyGain(PcmAudioFrame& pcmFrame, AudioFrame<float>& frame, float gain)
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
                configuration.channelMap);
        default:
            THROW_INVALID_VALUE_EXCEPTION("backend", "");
    }
}

void run(unique_ptr<PcmDevice> captureDevice, const CaptureNodeConfiguration& configuration, ros::Publisher& audioPub)
{
    PcmAudioFrame manyChannelPcmFrame(configuration.format, configuration.channelCount, configuration.frameSampleCount);
    PcmAudioFrame oneChannelPcmFrame(configuration.format, 1, configuration.frameSampleCount);
    AudioFrame<float> manyChannelFrame(configuration.channelCount, configuration.frameSampleCount);
    AudioFrame<float> oneChannelFrame(1, configuration.frameSampleCount);

    audio_utils::AudioFrame audioFrameMsg;
    audioFrameMsg.format = configuration.formatString;
    audioFrameMsg.channel_count = configuration.merge ? 1 : configuration.channelCount;
    audioFrameMsg.sampling_frequency = configuration.samplingFrequency;
    audioFrameMsg.frame_sample_count = configuration.frameSampleCount;

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
            audioFrameMsg.data =
                vector<uint8_t>(oneChannelPcmFrame.data(), oneChannelPcmFrame.data() + oneChannelPcmFrame.size());
        }
        else
        {
            applyGain(manyChannelPcmFrame, manyChannelFrame, configuration.gain);
            audioFrameMsg.data =
                vector<uint8_t>(manyChannelPcmFrame.data(), manyChannelPcmFrame.data() + manyChannelPcmFrame.size());
        }

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

        bool latencyUsFound = privateNodeHandle.getParam("latency_us", configuration.latencyUs);
        if (latencyUsFound && configuration.backend != PcmDevice::Backend::Alsa)
        {
            ROS_ERROR("The parameter latency_us is only supported with the Alsa backend");
            return -1;
        }
        else if (!latencyUsFound && configuration.backend == PcmDevice::Backend::Alsa)
        {
            ROS_ERROR("The parameter latency_us must be set with the Alsa backend");
            return -1;
        }

        bool channelMapFound = privateNodeHandle.getParam("channel_map", configuration.channelMap);
        if (channelMapFound && configuration.backend != PcmDevice::Backend::PulseAudio)
        {
            ROS_ERROR("The parameter channel_map is only supported with the PulseAudio backend");
            return -1;
        }

        configuration.merge = privateNodeHandle.param("merge", false);
        configuration.gain = privateNodeHandle.param("gain", 1.f);

        run(createCaptureDevice(configuration), configuration, audioPub);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR(e.what());
        return -1;
    }

    return 0;
}
