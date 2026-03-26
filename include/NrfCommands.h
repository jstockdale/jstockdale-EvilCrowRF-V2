/**
 * @file NrfCommands.h
 * @brief BLE command handlers for all NRF24 features.
 *
 * Registers command IDs 0x20-0x2F, 0x41-0x45 for MouseJack, Spectrum, and Jammer.
 * Declarations only — implementations in NrfCommands.cpp.
 */

#ifndef NRF_COMMANDS_H
#define NRF_COMMANDS_H

#include <Arduino.h>
#include "core/ble/CommandHandler.h"
#include "core/ble/ClientsManager.h"
#include "BinaryMessages.h"
#include "modules/nrf/NrfModule.h"
#include "modules/nrf/MouseJack.h"
#include "modules/nrf/NrfSpectrum.h"
#include "modules/nrf/NrfJammer.h"
#include "ConfigManager.h"
#include "esp_log.h"

class NrfCommands {
public:
    static void registerCommands(CommandHandler& handler);

private:
    static bool handleInit(const uint8_t* data, size_t len);
    static bool handleScanStart(const uint8_t* data, size_t len);
    static bool handleScanStop(const uint8_t* data, size_t len);
    static bool handleScanStatus(const uint8_t* data, size_t len);
    static bool handleAttackHid(const uint8_t* data, size_t len);
    static bool handleAttackString(const uint8_t* data, size_t len);
    static bool handleAttackDucky(const uint8_t* data, size_t len);
    static bool handleAttackStop(const uint8_t* data, size_t len);
    static bool handleSpectrumStart(const uint8_t* data, size_t len);
    static bool handleSpectrumStop(const uint8_t* data, size_t len);
    static bool handleJamStart(const uint8_t* data, size_t len);
    static bool handleJamStop(const uint8_t* data, size_t len);
    static bool handleJamSetMode(const uint8_t* data, size_t len);
    static bool handleJamSetChannel(const uint8_t* data, size_t len);
    static bool handleClearTargets(const uint8_t* data, size_t len);
    static bool handleStopAll(const uint8_t* data, size_t len);
    static bool handleNrfSettings(const uint8_t* data, size_t len);
    static bool handleJamSetDwell(const uint8_t* data, size_t len);
    static bool handleJamModeConfig(const uint8_t* data, size_t len);
    static bool handleJamModeInfo(const uint8_t* data, size_t len);
    static bool handleJamResetConfig(const uint8_t* data, size_t len);
};

#endif // NRF_COMMANDS_H
