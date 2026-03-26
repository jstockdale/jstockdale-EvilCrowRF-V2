#include "LogGate.h"

bool LogGate::suppressed_ = false;
vprintf_like_t LogGate::originalVprintf_ = nullptr;
