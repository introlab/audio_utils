#include <MusicBeatDetector/Utils/ClassMacro.h>
#include <MusicBeatDetector/Utils/Data/PcmAudioFrame.h>
#include <MusicBeatDetector/Utils/Data/PackedAudioFrame.h>
#include <MusicBeatDetector/Utils/Exception/NotSupportedException.h>

#include <audio_utils_msgs/msg/voice_activity.hpp>
#include <audio_utils_msgs/msg/audio_frame.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <onnxruntime_cxx_api.h>

#include <memory>

using namespace introlab;
using namespace std;

constexpr size_t SupportedChannelCount = 1;
constexpr size_t SupportedSamplingFrequency = 16000;
constexpr size_t SupportedFrameSampleCount = 512;

constexpr const char* NODE_NAME = "vad_node";

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

        string modelPath = ament_index_cpp::get_package_share_directory("audio_utils") + "/models/silero_vad.onnx";
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

class VadNode : public rclcpp::Node
{
    rclcpp::Subscription<audio_utils_msgs::msg::AudioFrame>::SharedPtr m_audioSub;
    rclcpp::Publisher<audio_utils_msgs::msg::VoiceActivity>::SharedPtr m_voiceActivityPub;

    Vad m_vad;
    audio_utils_msgs::msg::VoiceActivity m_voiceActivityMsg;

public:
    VadNode()
        : rclcpp::Node(NODE_NAME),
          m_vad(
              declare_parameter("silence_to_voice_threshold", 0.5f),
              declare_parameter("voice_to_silence_threshold", 0.4f),
              declare_parameter("min_silence_duration_ms", 500) * SupportedSamplingFrequency / 1000 /
                  SupportedFrameSampleCount)
    {
        m_audioSub = create_subscription<audio_utils_msgs::msg::AudioFrame>(
            "audio_in",
            100,
            [this](const audio_utils_msgs::msg::AudioFrame::SharedPtr msg) { audioCallback(msg); });
        m_voiceActivityPub = create_publisher<audio_utils_msgs::msg::VoiceActivity>("voice_activity", 100);
    }

    void run() { rclcpp::spin(shared_from_this()); }

private:
    void audioCallback(const audio_utils_msgs::msg::AudioFrame::SharedPtr msg)
    {
        PcmAudioFrameFormat format = parseFormat(msg->format);
        if (msg->channel_count != SupportedChannelCount || msg->sampling_frequency != SupportedSamplingFrequency ||
            (msg->frame_sample_count % SupportedFrameSampleCount) != 0 ||
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

        m_voiceActivityMsg.header = msg->header;
        m_voiceActivityMsg.is_voice = false;
        for (size_t i = 0; i < msg->frame_sample_count; i += SupportedFrameSampleCount)
        {
            m_voiceActivityMsg.is_voice =
                m_voiceActivityMsg.is_voice || m_vad.detect(frame.slice(i, SupportedFrameSampleCount));
        }

        m_voiceActivityPub->publish(m_voiceActivityMsg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<VadNode>();
        node->run();
    }
    catch (const exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "%s", e.what());
        return -1;
    }
    rclcpp::shutdown();
    return 0;
}
