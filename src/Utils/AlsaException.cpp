#if defined(__unix__) || defined(__linux__)

#include "AlsaException.h"

using namespace introlab;
using namespace std;

AlsaException::AlsaException(
    const string& filename,
    const string& function,
    int line,
    const string& message,
    int errorCode,
    const string& errorDescription)
    : runtime_error(
          "[" + filename + ", " + function + ", " + to_string(line) + "] : AlsaException: " + message + " (" +
          to_string(errorCode) + ": " + errorDescription + ")")
{
}

AlsaException::~AlsaException() {}

#endif
