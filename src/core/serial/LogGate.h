/**
 * @file LogGate.h
 * @brief Conditionally suppress ESP_LOG output on the serial UART.
 *
 * When SDR text mode is active, ESP_LOG output on Serial would corrupt
 * the text command/response protocol.  LogGate uses esp_log_set_vprintf()
 * to redirect log output to a no-op function while the gate is closed,
 * and restores the original handler when re-opened.
 *
 * Usage:
 *     LogGate::suppress();   // call from SdrModule::enable()
 *     LogGate::restore();    // call from SdrModule::disable()
 */

#ifndef LOG_GATE_H
#define LOG_GATE_H

#include "esp_log.h"
#include <cstdio>
#include <cstdarg>

class LogGate {
public:
    /// Suppress all ESP_LOG output on the primary serial UART.
    static void suppress() {
        if (!suppressed_) {
            originalVprintf_ = esp_log_set_vprintf(nullVprintf);
            suppressed_ = true;
        }
    }

    /// Restore normal ESP_LOG output.
    static void restore() {
        if (suppressed_ && originalVprintf_) {
            esp_log_set_vprintf(originalVprintf_);
            suppressed_ = false;
        }
    }

    /// @return true if log output is currently suppressed.
    static bool isSuppressed() { return suppressed_; }

private:
    static bool suppressed_;
    static vprintf_like_t originalVprintf_;

    /// No-op vprintf that discards all output.
    static int nullVprintf(const char* fmt, va_list args) {
        (void)fmt;
        (void)args;
        return 0;
    }
};

#endif // LOG_GATE_H
