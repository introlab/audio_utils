#if defined(__unix__) || defined(__linux__)

#include "PulseAudioException.h"

using namespace introlab;
using namespace std;

PulseAudioException::PulseAudioException(
    const string& filename,
    const string& function,
    int line,
    const string& message,
    int errorCode,
    const string& errorDescription)
    : runtime_error(
          "[" + filename + ", " + function + ", " + to_string(line) + "] : PulseAudioException: " + message + " (" +
          to_string(errorCode) + ": " + errorDescription + ")")
{
}

PulseAudioException::~PulseAudioException() {}

#endif
