#include "AlsaPcmDevice.h"

#include <audio_utils/AudioFrame.h>

#include <ros/ros.h>


using namespace introlab;
using namespace std;

void mergeChannels(const PcmAudioFrame& pcmInput, PcmAudioFrame& pcmOutput, float gain)
{
    AudioFrame<float> input = pcmInput;
    AudioFrame<float> output(1, input.sampleCount());

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

    pcmOutput = PcmAudioFrame(output, pcmInput.format());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alsa_capture_node");

    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");

    ros::Publisher audioPub = nodeHandle.advertise<audio_utils::AudioFrame>("audio_out", 100);

    string device;
    string formatString;
    PcmAudioFrameFormat format;
    int channelCount;
    int samplingFrequency;
    int frameSampleCount;
    int latencyUs;

    bool merge;
    float merge_gain;

    privateNodeHandle.getParam("device", device);
    privateNodeHandle.getParam("format", formatString);
    format = parseFormat(formatString);

    privateNodeHandle.getParam("channel_count", channelCount);
    privateNodeHandle.getParam("sampling_frequency", samplingFrequency);
    privateNodeHandle.getParam("frame_sample_count", frameSampleCount);
    privateNodeHandle.getParam("latency_us", latencyUs);


    privateNodeHandle.getParam("merge", merge);
    privateNodeHandle.getParam("merge_gain", merge_gain);

    audio_utils::AudioFrame audioFrameMsg;
    audioFrameMsg.format = formatString;
    audioFrameMsg.channel_count = merge ? 1 : channelCount;
    audioFrameMsg.sampling_frequency = samplingFrequency;
    audioFrameMsg.frame_sample_count = frameSampleCount;

    try
    {
        AlsaPcmDevice captureDevice(device, AlsaPcmDevice::Stream::Capture, format, channelCount, frameSampleCount, samplingFrequency, latencyUs);
        PcmAudioFrame manyChannelFrame(format, channelCount, frameSampleCount);
        PcmAudioFrame oneChannelFrame(format, 1, frameSampleCount);


        while(ros::ok())
        {
            captureDevice.read(manyChannelFrame);

            if (merge)
            {
                mergeChannels(manyChannelFrame, oneChannelFrame, merge_gain);
                audioFrameMsg.data = vector<uint8_t>(oneChannelFrame.data(), oneChannelFrame.data() + oneChannelFrame.size());
            }
            else
            {
                audioFrameMsg.data = vector<uint8_t>(manyChannelFrame.data(), manyChannelFrame.data() + manyChannelFrame.size());
            }

            audioPub.publish(audioFrameMsg);
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR(e.what());
    }

    return 0;
}
