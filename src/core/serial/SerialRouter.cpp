/**
 * @file SerialRouter.cpp
 * @brief Serial command router implementation.
 *
 * State machine flow:
 *
 *   IDLE ──┬── byte == 0xAA ──────────────→ BINARY_FRAME
 *          ├── byte is printable ASCII ───→ TEXT_LINE
 *          └── anything else ─────────────→ RAW_CMD
 *
 *   BINARY_FRAME ─── frame complete ─────→ dispatch → IDLE
 *                 ─── timeout (500ms) ───→ discard → IDLE
 *
 *   TEXT_LINE ─── newline received ───────→ dispatch → IDLE
 *             ─── buffer full ───────────→ discard → IDLE
 *
 *   RAW_CMD ─── handler consumed >0 ─────→ shift buffer → IDLE (or stay if more data)
 *           ─── timeout (200ms) ─────────→ discard → IDLE
 */

#include "SerialRouter.h"
#include "esp_log.h"

static const char* TAG = "SerialRouter";

// ── Lifecycle ────────────────────────────────────────────────────────

void SerialRouter::init() {
    // Drain any garbage accumulated in the UART RX FIFO during boot.
    // Between Serial.begin() and this task starting (~1.3s), noise from
    // the USB-serial chip reset can leave stale bytes that would put us
    // into BINARY_FRAME mode on the first poll.
    vTaskDelay(pdMS_TO_TICKS(50));
    int drained = 0;
    while (Serial.available()) {
        Serial.read();
        drained++;
    }
    if (drained > 0) {
        ESP_LOGI(TAG, "Boot flush: drained %d stale bytes", drained);
    }
    resetBuffer();
    ESP_LOGI(TAG, "Initialized (frame timeout=%dms, raw timeout=%dms)",
             FRAME_TIMEOUT_MS, RAW_CMD_TIMEOUT_MS);
}

void SerialRouter::poll() {
    // ── Check for timeouts in non-IDLE states ────────────────────
    if (state_ != State::IDLE) {
        uint32_t timeout = (state_ == State::BINARY_FRAME)
                           ? FRAME_TIMEOUT_MS
                           : RAW_CMD_TIMEOUT_MS;
        if (isTimedOut(timeout)) {
            resetBuffer();
            return;
        }
    }

    // ── Read available bytes ─────────────────────────────────────
    if (!Serial.available()) {
        return;
    }

    uint8_t byte = Serial.read();

    // Buffer overflow protection
    if (bufIdx_ >= BUF_SIZE) {
        resetBuffer();
        return;
    }

    buffer_[bufIdx_++] = byte;

    // Debug: log state transitions on first byte of a new message

    // ── State machine ────────────────────────────────────────────
    switch (state_) {
        case State::IDLE:
            classifyByte(byte);
            break;

        case State::BINARY_FRAME:
            tryDispatchBinaryFrame();
            break;

        case State::TEXT_LINE:
            tryDispatchTextLine();
            break;

        case State::RAW_CMD:
            tryDispatchRawCmd();
            break;
    }
}

// ── State management ─────────────────────────────────────────────────

void SerialRouter::enterState(State s) {
    state_ = s;
    stateEnterMs_ = millis();
}

void SerialRouter::resetBuffer() {
    bufIdx_ = 0;
    enterState(State::IDLE);
}

bool SerialRouter::isTimedOut(uint32_t timeoutMs) const {
    return (millis() - stateEnterMs_) > timeoutMs;
}

// ── First-byte classification ────────────────────────────────────────

void SerialRouter::classifyByte(uint8_t byte) {
    if (byte == FRAME_MAGIC) {
        enterState(State::BINARY_FRAME);
    } else if (byte >= 0x20 && byte < 0x7F) {
        // Printable ASCII → text command mode
        enterState(State::TEXT_LINE);
    } else if (byte == '\n' || byte == '\r') {
        // Bare newline — ignore (parser flush, keepalive, etc.)
        resetBuffer();
    } else {
        // Non-printable, non-magic → raw byte command
        enterState(State::RAW_CMD);
    }
}

// ── Binary frame dispatch ────────────────────────────────────────────
//
// Frame format:
//   [0xAA] [cmd] [module] [flags] [reserved] [dataLen_lo] [dataLen_hi] [data...] [checksum]
//   Header = 7 bytes, total = 7 + dataLen + 1

void SerialRouter::tryDispatchBinaryFrame() {
    if (bufIdx_ < FRAME_MIN_SIZE) {
        return;  // Need more data
    }

    // Validate magic (should always be true since we enter this state on 0xAA)
    if (buffer_[0] != FRAME_MAGIC) {
        ESP_LOGW(TAG, "Binary frame: bad magic 0x%02X, resetting", buffer_[0]);
        resetBuffer();
        return;
    }

    // Extract data length (little-endian, bytes 5-6)
    uint16_t dataLen = buffer_[5] | (buffer_[6] << 8);
    size_t expectedLen = FRAME_HEADER_SIZE + dataLen + 1;  // +1 for checksum

    // Sanity check: reject absurd lengths
    if (expectedLen > BUF_SIZE) {
        ESP_LOGW(TAG, "Binary frame: dataLen=%u exceeds buffer, discarding", dataLen);
        resetBuffer();
        return;
    }

    if (bufIdx_ < expectedLen) {
        return;  // Need more data
    }

    // Complete frame — dispatch
    if (binaryHandler_) {
        binaryHandler_(buffer_, expectedLen);
    }

    // Shift any remaining bytes (pipelining)
    size_t remaining = bufIdx_ - expectedLen;
    if (remaining > 0) {
        memmove(buffer_, buffer_ + expectedLen, remaining);
    }
    bufIdx_ = remaining;
    enterState(State::IDLE);

    // If there's leftover data, classify the first byte immediately
    if (bufIdx_ > 0) {
        classifyByte(buffer_[0]);
    }
}

// ── Text line dispatch ───────────────────────────────────────────────

void SerialRouter::tryDispatchTextLine() {
    // Scan for newline in the accumulated data
    for (size_t i = 0; i < bufIdx_; i++) {
        if (buffer_[i] == '\n' || buffer_[i] == '\r') {
            // Found end of line — extract command
            size_t cmdLen = i;

            // Skip empty lines
            if (cmdLen == 0) {
                // Consume the newline(s) and continue
                size_t consumed = i + 1;
                while (consumed < bufIdx_ &&
                       (buffer_[consumed] == '\n' || buffer_[consumed] == '\r')) {
                    consumed++;
                }
                size_t remaining = bufIdx_ - consumed;
                if (remaining > 0) {
                    memmove(buffer_, buffer_ + consumed, remaining);
                }
                bufIdx_ = remaining;
                enterState(State::IDLE);
                if (bufIdx_ > 0) {
                    classifyByte(buffer_[0]);
                }
                return;
            }

            // Build trimmed command string
            String cmd;
            cmd.reserve(cmdLen);
            for (size_t c = 0; c < cmdLen; c++) {
                cmd += (char)buffer_[c];
            }

            // Consume newline + any trailing \r\n
            size_t consumed = i + 1;
            while (consumed < bufIdx_ &&
                   (buffer_[consumed] == '\n' || buffer_[consumed] == '\r')) {
                consumed++;
            }

            // Dispatch
            bool handled = false;
            if (textHandler_) {
                handled = textHandler_(cmd);
            }
            if (!handled) {
                Serial.println("HACKRF_ERROR");
                Serial.println("Unknown command: " + cmd);
            }

            // Shift remaining
            size_t remaining = bufIdx_ - consumed;
            if (remaining > 0) {
                memmove(buffer_, buffer_ + consumed, remaining);
            }
            bufIdx_ = remaining;
            enterState(State::IDLE);
            if (bufIdx_ > 0) {
                classifyByte(buffer_[0]);
            }
            return;
        }
    }
    // No newline yet — keep accumulating
}

// ── Raw byte command dispatch ────────────────────────────────────────

void SerialRouter::tryDispatchRawCmd() {
    if (!rawHandler_) {
        // No handler registered — discard
        resetBuffer();
        return;
    }

    size_t consumed = 0;
    rawHandler_(buffer_, bufIdx_, consumed);

    if (consumed == 0) {
        // Handler needs more data — keep accumulating
        return;
    }

    // Shift remaining
    size_t remaining = bufIdx_ - consumed;
    if (remaining > 0) {
        memmove(buffer_, buffer_ + consumed, remaining);
    }
    bufIdx_ = remaining;
    enterState(State::IDLE);

    // Classify next byte if present
    if (bufIdx_ > 0) {
        classifyByte(buffer_[0]);
    }
}
