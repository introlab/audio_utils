#include <MusicBeatDetector/Utils/ClassMacro.h>
#include <MusicBeatDetector/Utils/Data/PcmAudioFrame.h>
#include <MusicBeatDetector/Utils/Data/PackedAudioFrame.h>
#include <MusicBeatDetector/Utils/Exception/NotSupportedException.h>

#include <audio_utils/VoiceActivity.h>
#include <audio_utils/AudioFrame.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <onnxruntime_cxx_api.h>

#include <memory>

using namespace introlab;
using namespace std;

constexpr size_t SupportedChannelCount = 1;
constexpr size_t SupportedSamplingFrequency = 16000;
constexpr size_t SupportedFrameSampleCount = 512;

enum class VadStateType
{
    SILENCE,
    VOICE,
    SILENCE_PENDING
};

// Inspired by https://github.com/snakers4/silero-vad/blob/master/examples/cpp/silero-vad-onnx.cpp
class Vad
{
    static constexpr size_t SIZE_HC = 128;

    float m_silenceToVoiceThreshold;
    float m_voiceToSilenceThreshold;
    size_t m_minSilenceFrameCount;

    PackedAudioFrame<float> m_packedAudioFrame;

    Ort::Env m_env;
    Ort::SessionOptions m_sessionOptions;
    unique_ptr<Ort::Session> m_session;
    Ort::MemoryInfo m_cpuMemoryInfo;

    vector<const char*> m_ortInputsNames;
    vector<Ort::Value> m_ortInputs;

    vector<int64_t> m_inputShape;

    vector<int64_t> m_sr;
    vector<int64_t> m_srShape;

    vector<float> m_h;
    vector<float> m_c;
    vector<int64_t> m_hcShape;

    vector<const char*> m_ortOutputsNames;

    // State
    VadStateType m_stateType;
    size_t m_frameIndex;
    size_t m_silenceFrameIndex;

public:
    Vad(float silenceToVoiceThreshold, float voiceToSilenceThreshold, size_t minSilenceFrameCount)
        : m_silenceToVoiceThreshold(silenceToVoiceThreshold),
          m_voiceToSilenceThreshold(voiceToSilenceThreshold),
          m_minSilenceFrameCount(minSilenceFrameCount),
          m_packedAudioFrame(1, SupportedFrameSampleCount),
          m_cpuMemoryInfo(nullptr),
          m_ortInputsNames({"input", "sr", "h", "c"}),
          m_inputShape({1, SupportedFrameSampleCount}),
          m_sr({SupportedSamplingFrequency}),
          m_srShape({1}),
          m_h(SIZE_HC),
          m_c(SIZE_HC),
          m_hcShape({2, 1, 64}),
          m_ortOutputsNames({"output", "hn", "cn"})
    {
        m_sessionOptions.SetInterOpNumThreads(1);
        m_sessionOptions.SetIntraOpNumThreads(1);
        m_sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        m_sessionOptions.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        m_sessionOptions.SetLogSeverityLevel(ORT_LOGGING_LEVEL_ERROR);

        string modelPath = ros::package::getPath("audio_utils") + "/models/silero_vad.onnx";
        m_session = make_unique<Ort::Session>(m_env, modelPath.c_str(), m_sessionOptions);
        m_cpuMemoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        reset();
    }

    DECLARE_NOT_COPYABLE(Vad);
    DECLARE_NOT_MOVABLE(Vad);

    void reset()
    {
        memset(m_h.data(), 0, m_h.size() * sizeof(float));
        memset(m_c.data(), 0, m_c.size() * sizeof(float));

        m_stateType = VadStateType::SILENCE;
        m_frameIndex = 0;
        m_silenceFrameIndex = 0;
    }

    bool detect(const PcmAudioFrame& pcmAudioframe)
    {
        if (pcmAudioframe.channelCount() != 1 || pcmAudioframe.sampleCount() != SupportedFrameSampleCount)
        {
            THROW_NOT_SUPPORTED_EXCEPTION("Invalid frame");
        }

        m_packedAudioFrame = pcmAudioframe;
        m_ortInputs.clear();
        m_ortInputs.emplace_back(Ort::Value::CreateTensor<float>(
            m_cpuMemoryInfo,
            m_packedAudioFrame.data(),
            m_packedAudioFrame.size(),
            m_inputShape.data(),
            m_inputShape.size()));
        m_ortInputs.emplace_back(Ort::Value::CreateTensor<int64_t>(
            m_cpuMemoryInfo,
            m_sr.data(),
            m_sr.size(),
            m_srShape.data(),
            m_srShape.size()));
        m_ortInputs.emplace_back(Ort::Value::CreateTensor<float>(
            m_cpuMemoryInfo,
            m_h.data(),
            m_h.size(),
            m_hcShape.data(),
            m_hcShape.size()));
        m_ortInputs.emplace_back(Ort::Value::CreateTensor<float>(
            m_cpuMemoryInfo,
            m_c.data(),
            m_c.size(),
            m_hcShape.data(),
            m_hcShape.size()));

        vector<Ort::Value> ortOutputs = m_session->Run(
            Ort::RunOptions{nullptr},
            m_ortInputsNames.data(),
            m_ortInputs.data(),
            m_ortInputs.size(),
            m_ortOutputsNames.data(),
            m_ortOutputsNames.size());
        float voiceProbability = ortOutputs[0].GetTensorMutableData<float>()[0];
        float* hn = ortOutputs[1].GetTensorMutableData<float>();
        memcpy(m_h.data(), hn, m_h.size() * sizeof(float));
        float* cn = ortOutputs[2].GetTensorMutableData<float>();
        memcpy(m_c.data(), cn, m_c.size() * sizeof(float));

        return updateState(voiceProbability);
    }

    bool updateState(float voiceProbability)
    {
        m_frameIndex++;

        if (m_stateType == VadStateType::SILENCE && voiceProbability > m_silenceToVoiceThreshold)
        {
            m_stateType = VadStateType::VOICE;
        }
        else if (m_stateType == VadStateType::VOICE && voiceProbability < m_voiceToSilenceThreshold)
        {
            m_stateType = VadStateType::SILENCE_PENDING;
            m_silenceFrameIndex = m_frameIndex;
        }
        else if (m_stateType == VadStateType::SILENCE_PENDING && voiceProbability > m_voiceToSilenceThreshold)
        {
            m_stateType = VadStateType::VOICE;
        }
        else if (
            m_stateType == VadStateType::SILENCE_PENDING &&
            (m_frameIndex - m_silenceFrameIndex) > m_minSilenceFrameCount)
        {
            m_stateType = VadStateType::SILENCE;
        }

        switch (m_stateType)
        {
            case VadStateType::SILENCE:
                return false;
            case VadStateType::VOICE:
            case VadStateType::SILENCE_PENDING:
                return true;
            default:
                return false;
        }
    }
};

class VadNode
{
    ros::NodeHandle m_nodeHandle;

    ros::Subscriber m_audioSub;
    ros::Publisher m_voiceActivityPub;

    uint32_t m_lastSeq;
    Vad m_vad;
    audio_utils::VoiceActivity m_voiceActivityMsg;

public:
    VadNode(float silenceToVoiceThreshold, float voiceToSilenceThreshold, size_t minSilenceFrameCount)
        : m_lastSeq(0),
          m_vad(silenceToVoiceThreshold, voiceToSilenceThreshold, minSilenceFrameCount)
    {
        m_audioSub = m_nodeHandle.subscribe("audio_in", 100, &VadNode::audioCallback, this);
        m_voiceActivityPub = m_nodeHandle.advertise<audio_utils::VoiceActivity>("voice_activity", 100);
    }

    void run() { ros::spin(); }

private:
    void audioCallback(const audio_utils::AudioFramePtr& msg)
    {
        PcmAudioFrameFormat format = parseFormat(msg->format);
        if (msg->channel_count != SupportedChannelCount || msg->sampling_frequency != SupportedSamplingFrequency ||
            (msg->frame_sample_count % SupportedFrameSampleCount) != 0 ||
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

        if (msg->header.seq != m_lastSeq + 1)
        {
            m_vad.reset();
        }
        m_lastSeq = msg->header.seq;

        PcmAudioFrame frame(format, msg->channel_count, msg->frame_sample_count, msg->data.data());

        m_voiceActivityMsg.header = msg->header;
        m_voiceActivityMsg.is_voice = false;
        for (size_t i = 0; i < msg->frame_sample_count; i += SupportedFrameSampleCount)
        {
            m_voiceActivityMsg.is_voice =
                m_voiceActivityMsg.is_voice || m_vad.detect(frame.slice(i, SupportedFrameSampleCount));
        }

        m_voiceActivityPub.publish(m_voiceActivityMsg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vad_node");

    ros::NodeHandle privateNodeHandle("~");

    double silenceToVoiceThreshold;
    if (!privateNodeHandle.getParam("silence_to_voice_threshold", silenceToVoiceThreshold))
    {
        ROS_ERROR("The parameter silence_to_voice_threshold is required.");
        return -1;
    }

    double voiceToSilenceThreshold;
    if (!privateNodeHandle.getParam("voice_to_silence_threshold", voiceToSilenceThreshold))
    {
        ROS_ERROR("The parameter voice_to_silence_threshold is required.");
        return -1;
    }
    if (voiceToSilenceThreshold >= silenceToVoiceThreshold)
    {
        ROS_ERROR("voice_to_silence_threshold must be lower than voice_to_silence_threshold.");
        return -1;
    }

    double minSilenceDurationMs;
    if (!privateNodeHandle.getParam("min_silence_duration_ms", minSilenceDurationMs))
    {
        ROS_ERROR("The parameter min_silence_duration_ms is required.");
        return -1;
    }
    size_t minSilenceFrameCount =
        static_cast<size_t>(minSilenceDurationMs * SupportedSamplingFrequency / 1000 / SupportedFrameSampleCount);

    try
    {
        VadNode node(
            static_cast<float>(silenceToVoiceThreshold),
            static_cast<float>(voiceToSilenceThreshold),
            minSilenceFrameCount);
        node.run();
    }
    catch (const exception& e)
    {
        ROS_ERROR("%s", e.what());
        return -1;
    }

    return 0;
}
