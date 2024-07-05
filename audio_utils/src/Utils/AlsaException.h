#ifndef UTILS_ALSA_EXCEPTION_H
#define UTILS_ALSA_EXCEPTION_H

#if defined(__unix__) || defined(__linux__)

#include <stdexcept>

#define THROW_ALSA_EXCEPTION(message, errorCode, errorDescription)                                                     \
    throw introlab::AlsaException(                                                                                     \
        __FILENAME__,                                                                                                  \
        __LOGGED_FUNCTION__,                                                                                           \
        __LINE__,                                                                                                      \
        (message),                                                                                                     \
        (errorCode),                                                                                                   \
        (errorDescription))

namespace introlab
{
    class AlsaException : public std::runtime_error
    {
    public:
        AlsaException(
            const std::string& filename,
            const std::string& function,
            int line,
            const std::string& message,
            int errorCode,
            const std::string& errorDescription);

        ~AlsaException() override;
    };
}

#else

#error "Invalid include file"

#endif

#endif
