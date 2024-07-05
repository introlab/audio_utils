#ifndef UTILS_PULSE_AUDIO_EXCEPTION_H
#define UTILS_PULSE_AUDIO_EXCEPTION_H

#if defined(__unix__) || defined(__linux__)

#include <stdexcept>

#define THROW_PULSE_AUDIO_EXCEPTION(message, errorCode, errorDescription)                                              \
    throw introlab::PulseAudioException(                                                                               \
        __FILENAME__,                                                                                                  \
        __LOGGED_FUNCTION__,                                                                                           \
        __LINE__,                                                                                                      \
        (message),                                                                                                     \
        (errorCode),                                                                                                   \
        (errorDescription))

namespace introlab
{
    class PulseAudioException : public std::runtime_error
    {
    public:
        PulseAudioException(
            const std::string& filename,
            const std::string& function,
            int line,
            const std::string& message,
            int errorCode,
            const std::string& errorDescription);

        ~PulseAudioException() override;
    };
}

#else

#error "Invalid include file"

#endif

#endif
