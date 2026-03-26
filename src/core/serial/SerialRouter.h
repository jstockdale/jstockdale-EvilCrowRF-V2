/**
 * @file SerialRouter.h
 * @brief Serial command router for EvilCrow-RF-V2.
 *
 * Replaces the monolithic inline serial parser in main.cpp with a clean
 * state machine that discriminates between three traffic types on the
 * UART and routes each to the appropriate handler:
 *
 *   1. Binary framed packets  (magic 0xAA)  → BleAdapter::processBinaryData
 *   2. Text line commands      (ASCII + \n)  → SdrModule::processSerialCommand
 *   3. Raw byte commands       (0x01, 0x04)  → bruter / ping handler
 *
 * Key improvements over the original:
 *   - Timeout recovery: if a binary frame isn't completed within
 *     FRAME_TIMEOUT_MS the buffer is flushed and the router returns to
 *     IDLE.  A single stale 0xAA can no longer brick the interface.
 *   - Boot flush: drains any garbage bytes in the UART RX FIFO before
 *     entering the main loop (noise accumulated between Serial.begin
 *     and task start).
 *   - Buffer overflow protection: oversized accumulations are discarded
 *     rather than silently wrapping.
 *   - Clean separation: no business logic in the router — only dispatch.
 */

#ifndef SERIAL_ROUTER_H
#define SERIAL_ROUTER_H

#include <Arduino.h>
#include <functional>
#include <cstdint>

class SerialRouter {
public:
    /// Handler for complete binary framed packets (0xAA header + payload + checksum).
    /// @param data   Raw packet bytes (including magic byte).
    /// @param len    Total packet length.
    using BinaryFrameHandler = std::function<void(uint8_t* data, size_t len)>;

    /// Handler for text line commands (terminated by \n or \r\n).
    /// @param cmd    Trimmed command string (no trailing whitespace).
    /// @return true if the command was recognized, false otherwise.
    using TextCommandHandler = std::function<bool(const String& cmd)>;

    /// Handler for raw single/multi-byte commands (non-ASCII, non-0xAA).
    /// Called with the current buffer contents.  The handler must set
    /// `consumed` to the number of bytes it processed.  If it needs more
    /// data it should set consumed=0 and the router will keep accumulating.
    /// @param buf       Current buffer (first byte is the command byte).
    /// @param len       Number of bytes currently in the buffer.
    /// @param consumed  [out] Set by handler: bytes consumed (0 = need more).
    using RawByteHandler = std::function<void(const uint8_t* buf, size_t len, size_t& consumed)>;

    // ── Configuration ────────────────────────────────────────────

    void setBinaryFrameHandler(BinaryFrameHandler handler) { binaryHandler_ = handler; }
    void setTextCommandHandler(TextCommandHandler handler)  { textHandler_ = handler; }
    void setRawByteHandler(RawByteHandler handler)          { rawHandler_ = handler; }

    // ── Lifecycle ────────────────────────────────────────────────

    /// Call once at task start.  Drains stale UART RX bytes.
    void init();

    /// Call repeatedly from the serial task loop.
    /// Reads available bytes, runs the state machine, and dispatches
    /// complete messages to the registered handlers.
    void poll();

private:
    enum class State : uint8_t {
        IDLE,           ///< Waiting for first byte of a new message.
        RAW_CMD,        ///< Accumulating a raw byte command (0x01, 0x04 …).
        TEXT_LINE,      ///< Accumulating printable ASCII until newline.
        BINARY_FRAME,   ///< Accumulating a binary framed packet (0xAA …).
    };

    // ── Constants ────────────────────────────────────────────────
    static constexpr size_t   BUF_SIZE             = 512;
    static constexpr uint8_t  FRAME_MAGIC          = 0xAA;
    static constexpr uint32_t FRAME_TIMEOUT_MS     = 500;
    static constexpr uint32_t RAW_CMD_TIMEOUT_MS   = 200;
    static constexpr size_t   FRAME_HEADER_SIZE    = 7;   // magic + cmd + module + flags + dataLen(2)
    static constexpr size_t   FRAME_MIN_SIZE       = 8;   // header + checksum

    // ── State ────────────────────────────────────────────────────
    State    state_       = State::IDLE;
    uint8_t  buffer_[BUF_SIZE];
    size_t   bufIdx_      = 0;
    uint32_t stateEnterMs_ = 0;  // millis() when we entered the current state

    // ── Handlers ─────────────────────────────────────────────────
    BinaryFrameHandler binaryHandler_;
    TextCommandHandler textHandler_;
    RawByteHandler     rawHandler_;

    // ── Internal methods ─────────────────────────────────────────
    void enterState(State s);
    void resetBuffer();
    bool isTimedOut(uint32_t timeoutMs) const;

    /// Classify first byte → determine initial state.
    void classifyByte(uint8_t byte);

    /// Try to dispatch a complete binary frame.
    void tryDispatchBinaryFrame();

    /// Try to dispatch a complete text line.
    void tryDispatchTextLine();

    /// Try to dispatch a raw byte command.
    void tryDispatchRawCmd();
};

#endif // SERIAL_ROUTER_H
