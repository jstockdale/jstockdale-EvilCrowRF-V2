/**
 * @file main.cpp
 * @brief EvilCrow-RF-V2 firmware entry point.
 *
 * Responsible for hardware initialization, FreeRTOS task creation,
 * and the main Arduino loop.  Serial command routing is delegated to
 * SerialRouter; business logic lives in the command modules.
 */

#include <Arduino.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include "core/ble/CommandHandler.h"
#include "FileCommands.h"
#include "TransmitterCommands.h"
#include "RecorderCommands.h"
#include "StateCommands.h"
#include "BruterCommands.h"
#include "NrfCommands.h"
#include "OtaCommands.h"
#include "ButtonCommands.h"
#include "SdrCommands.h"
#include "AllProtocols.h"
#if PROTOPIRATE_MODULE_ENABLED
#include "ProtoPirateCommands.h"
#include "modules/protopirate/ProtoPirateModule.h"
#endif
#include "modules/bruter/bruter_main.h"
#include "core/ble/ClientsManager.h"
#include "ConfigManager.h"
#include "core/device_controls/DeviceControls.h"
#include "FS.h"
#include <LittleFS.h>
#include "SD.h"
#include "SPI.h"
#include "core/ble/BleAdapter.h"
#include "config.h"
#include "esp_log.h"
#include "modules/CC1101_driver/CC1101_Module.h"
#include "BinaryMessages.h"
#include "modules/CC1101_driver/CC1101_Worker.h"
#include "modules/nrf/NrfModule.h"
#include "modules/nrf/MouseJack.h"
#include "modules/nrf/NrfJammer.h"
#include "driver/gpio.h"
#include "core/serial/SerialRouter.h"

#if BATTERY_MODULE_ENABLED
#include "modules/battery/BatteryModule.h"
#endif
#if SDR_MODULE_ENABLED
#include "modules/sdr/SdrModule.h"
#endif

static const char* TAG = "Setup";

// ── Global objects ───────────────────────────────────────────────────

bool bleAdapterStarted = false;
BleAdapter bleAdapter;
uint32_t deviceTime = 0;           // Unix timestamp, updated by timeSyncTask
SPIClass sdspi(VSPI);
ClientsManager& clients = ClientsManager::getInstance();

struct DeviceConfig {
    bool powerBlink;
    bool sdCardMounted;
} deviceConfig;

// ── Forward declarations ─────────────────────────────────────────────

static void setupCc1101Pins();
static void setupFilesystems();
static void setupModules();
static void setupTasks();
void signalRecordedHandler(bool saved, const std::string& filename);
void cc1101WorkerSignalDetectedHandler(const CC1101DetectedSignal& signal);
void taskProcessor(void* pvParameters);
void serialCommandTask(void* pvParameters);
void timeSyncTask(void* pvParameters);

// ── Serial command handlers (used by SerialRouter) ───────────────────

/// Handle raw byte commands: 0x01 (ping), 0x04 (bruter control).
static void handleRawByteCommand(const uint8_t* buf, size_t len, size_t& consumed);

/// Handle text line commands (SDR interface, help, etc.)
static bool handleTextCommand(const String& cmd);

/// Handle complete binary framed packets (0xAA header).
static void handleBinaryFrame(uint8_t* data, size_t len);

// ── Utility ──────────────────────────────────────────────────────────

void logHeapStats(const char* context) {
    size_t freeHeap = ESP.getFreeHeap();
    size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    size_t minFreeHeap = ESP.getMinFreeHeap();

    float fragmentation = 0.0f;
    if (freeHeap > 0) {
        fragmentation = 100.0f * (1.0f - (float)largestBlock / (float)freeHeap);
    }

    ESP_LOGI("Heap", "[%s] Free: %d, Largest: %d, MinFree: %d, Frag: %.1f%%",
             context, freeHeap, largestBlock, minFreeHeap, fragmentation);

    if (fragmentation > 30.0f) {
        ESP_LOGW("Heap", "High fragmentation: %.1f%%", fragmentation);
    }
}

// ── ConfigManager runtime application ────────────────────────────────

void ConfigManager::applyToRuntime() {
    BruterModule& bruter = getBruterModule();
    bruter.setInterFrameDelay(settings.bruterDelay);
    if (settings.bruterRepeats >= 1 && settings.bruterRepeats <= BRUTER_MAX_REPETITIONS) {
        bruter.setGlobalRepeats(settings.bruterRepeats);
    }
    extern ModuleCc1101 moduleCC1101State[CC1101_NUM_MODULES];
    moduleCC1101State[0].setPA(settings.radioPowerMod1);
    moduleCC1101State[1].setPA(settings.radioPowerMod2);
    ESP_LOGI("ConfigManager", "Runtime settings applied: delay=%dms reps=%d mod1_pwr=%ddBm mod2_pwr=%ddBm",
             settings.bruterDelay, settings.bruterRepeats,
             settings.radioPowerMod1, settings.radioPowerMod2);
}

// ═══════════════════════════════════════════════════════════════════════
//  Serial command handlers
// ═══════════════════════════════════════════════════════════════════════

void handleRawByteCommand(const uint8_t* buf, size_t len, size_t& consumed) {
    if (len < 1) return;
    uint8_t command = buf[0];

    // 0x01 — Ping
    if (command == 0x01) {
        Serial.write((uint8_t)MSG_COMMAND_SUCCESS);
        consumed = 1;
        return;
    }

    // 0x04 — Bruter control (variable-length sub-protocol)
    if (command == 0x04) {
        if (len < 2) return;  // Need sub-command byte
        uint8_t choice = buf[1];

        // Sub-commands that need extra bytes
        if (choice == 0xFE) {
            if (len < 4) return;  // Need 2 delay bytes
            consumed = 4;
            uint16_t delayMs = buf[2] | (buf[3] << 8);
            BruterModule& bruter = getBruterModule();
            bruter.setInterFrameDelay(delayMs);
            ESP_LOGI("Serial", "Inter-frame delay set to %d ms", delayMs);
            Serial.write((uint8_t)MSG_COMMAND_SUCCESS);
            return;
        }
        if (choice == 0xFC) {
            if (len < 3) return;  // Need repeats byte
            consumed = 3;
            uint8_t repeats = buf[2];
            if (repeats >= 1 && repeats <= BRUTER_MAX_REPETITIONS) {
                BruterModule& bruter = getBruterModule();
                bruter.setGlobalRepeats(repeats);
                ESP_LOGI("Serial", "Global repeats set to %d", repeats);
                Serial.write((uint8_t)MSG_COMMAND_SUCCESS);
            } else {
                Serial.write((uint8_t)MSG_COMMAND_ERROR);
                Serial.write((uint8_t)3);
            }
            return;
        }

        // All other sub-commands are 2 bytes
        consumed = 2;
        BruterModule& bruter = getBruterModule();

        if (choice == 0) {
            bruter.cancelAttack();
            Serial.write((uint8_t)MSG_COMMAND_SUCCESS);
        } else if (choice == 0xFB) {
            if (bruter.isAttackRunning()) {
                bruter.pauseAttack();
                Serial.write((uint8_t)MSG_COMMAND_SUCCESS);
            } else {
                Serial.write((uint8_t)MSG_COMMAND_ERROR);
                Serial.write((uint8_t)5);
            }
        } else if (choice == 0xFA) {
            if (bruter.isAttackRunning() || BruterModule::attackTaskHandle != nullptr) {
                Serial.write((uint8_t)MSG_COMMAND_ERROR);
                Serial.write((uint8_t)4);
            } else {
                Serial.write((uint8_t)MSG_COMMAND_SUCCESS);
                if (!bruter.resumeAttackAsync()) {
                    ESP_LOGE("Serial", "Resume failed — no saved state");
                }
            }
        } else if (choice == 0xF9) {
            bruter.checkAndNotifySavedState();
            Serial.write((uint8_t)MSG_COMMAND_SUCCESS);
        } else if (choice >= 1 && choice <= 40) {
            if (bruter.isAttackRunning() || BruterModule::attackTaskHandle != nullptr) {
                Serial.write((uint8_t)MSG_COMMAND_ERROR);
                Serial.write((uint8_t)4);
            } else {
                Serial.write((uint8_t)MSG_COMMAND_SUCCESS);
                if (!bruter.startAttackAsync(choice)) {
                    ESP_LOGE("Serial", "Failed to create bruter task");
                }
            }
        } else {
            Serial.write((uint8_t)MSG_COMMAND_ERROR);
            Serial.write((uint8_t)2);
        }
        return;
    }

    // Unknown raw command
    consumed = 1;
    Serial.write((uint8_t)MSG_COMMAND_ERROR);
    Serial.write((uint8_t)1);
}

bool handleTextCommand(const String& cmd) {
#if SDR_MODULE_ENABLED
    return SdrModule::processSerialCommand(cmd);
#else
    (void)cmd;
    return false;
#endif
}

void handleBinaryFrame(uint8_t* data, size_t len) {
    bleAdapter.setSerialCommand(true);
    bleAdapter.processBinaryData(data, len);
}

// ═══════════════════════════════════════════════════════════════════════
//  FreeRTOS tasks
// ═══════════════════════════════════════════════════════════════════════

void serialCommandTask(void* pvParameters) {
    static SerialRouter router;
    router.setBinaryFrameHandler(handleBinaryFrame);
    router.setTextCommandHandler(handleTextCommand);
    router.setRawByteHandler(handleRawByteCommand);
    router.init();

    while (true) {
        router.poll();
        if (!Serial.available()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void timeSyncTask(void* pvParameters) {
    const TickType_t delay = pdMS_TO_TICKS(1000);
    while (true) {
        vTaskDelay(delay);
        if (deviceTime > 0) {
            deviceTime++;
        }
    }
}

void taskProcessor(void* pvParameters) {
    if (ControllerAdapter::xTaskQueue == nullptr) {
        ESP_LOGE(TAG, "Task queue not found");
        vTaskDelete(nullptr);
    }
    QueueItem* item;
    while (true) {
        if (xQueueReceive(ControllerAdapter::xTaskQueue, &item, portMAX_DELAY)) {
            switch (item->type) {
                case Device::TaskType::Transmission: {
                    Device::TaskTransmission& task = item->transmissionTask;
                    ESP_LOGI(TAG, "Transmission on module %d", task.module);

                    if (task.filename) {
                        int repeat = task.repeat ? *task.repeat : 1;
                        if (CC1101Worker::transmit(task.module, *task.filename, repeat, task.pathType)) {
                            BinarySignalSent msg;
                            msg.module = task.module;
                            msg.filenameLength = (uint8_t)std::min((size_t)255, task.filename->length());
                            static uint8_t buffer[260];
                            memcpy(buffer, &msg, sizeof(msg));
                            memcpy(buffer + sizeof(msg), task.filename->c_str(), msg.filenameLength);
                            clients.notifyAllBinary(NotificationType::SignalSent, buffer, sizeof(msg) + msg.filenameLength);
                        } else {
                            BinarySignalSendError msg;
                            msg.module = task.module;
                            msg.errorCode = 1;
                            msg.filenameLength = (uint8_t)std::min((size_t)255, task.filename->length());
                            static uint8_t buffer[260];
                            memcpy(buffer, &msg, sizeof(msg));
                            memcpy(buffer + sizeof(msg), task.filename->c_str(), msg.filenameLength);
                            clients.notifyAllBinary(NotificationType::SignalSendingError, buffer, sizeof(msg) + msg.filenameLength);
                        }
                    }
                } break;

                case Device::TaskType::Record: {
                    Device::TaskRecord& task = item->recordTask;
                    if (task.module) {
                        int module = *task.module;
                        float frequency = task.config.frequency;
                        int modulation = MODULATION_ASK_OOK;
                        float deviation = 2.380371;
                        float bandwidth = 650;
                        float dataRate = 3.79372;
                        std::string preset = "Ook650";
                        std::string errorMessage;

                        if (task.config.preset) {
                            preset = *task.config.preset;
                            ESP_LOGI(TAG, "Preset: '%s'", preset.c_str());

                            if      (preset == "Ook270")     { bandwidth = 270.833333; }
                            else if (preset == "Ook650")     { /* defaults */ }
                            else if (preset == "2FSKDev238") { modulation = MODULATION_2_FSK; bandwidth = 270.833333; dataRate = 4.79794; }
                            else if (preset == "2FSKDev476") { modulation = MODULATION_2_FSK; deviation = 47.60742; bandwidth = 270.833333; dataRate = 4.79794; }
                            else { errorMessage = "Unsupported preset: " + preset; }
                        } else {
                            modulation = task.config.modulation ? *task.config.modulation : MODULATION_ASK_OOK;
                            bandwidth  = task.config.rxBandwidth ? *task.config.rxBandwidth : 650;
                            deviation  = task.config.deviation ? *task.config.deviation : 47.60742;
                            dataRate   = task.config.dataRate ? *task.config.dataRate : 4.79794;
                            preset = "Custom";
                        }

                        if (errorMessage.empty()) {
                            if (!CC1101Worker::startRecord(module, frequency, modulation, deviation, bandwidth, dataRate, preset)) {
                                static uint8_t err[2] = { MSG_ERROR, 11 };
                                clients.notifyAllBinary(NotificationType::SignalRecordError, err, 2);
                            }
                        } else {
                            ESP_LOGE(TAG, "%s", errorMessage.c_str());
                            static uint8_t errBuf[260];
                            errBuf[0] = MSG_ERROR;
                            errBuf[1] = 12;
                            uint8_t msgLen = (uint8_t)std::min((size_t)255, errorMessage.length());
                            memcpy(errBuf + 2, errorMessage.c_str(), msgLen);
                            clients.notifyAllBinary(NotificationType::SignalRecordError, errBuf, 2 + msgLen);
                        }
                    }
                } break;

                case Device::TaskType::DetectSignal: {
                    Device::TaskDetectSignal& task = item->detectSignalTask;
                    if (task.module && task.minRssi) {
                        bool bg = task.background ? *task.background : false;
                        CC1101Worker::startDetect(*task.module, *task.minRssi, bg);
                    }
                } break;

                case Device::TaskType::GetState: {
                    const byte numRegs = 0x2E;
                    BinaryStatus status;
                    status.messageType = MSG_STATUS;
                    status.module0Mode = static_cast<uint8_t>(CC1101Worker::getState(0));
                    status.module1Mode = static_cast<uint8_t>(CC1101Worker::getState(1));
                    status.numRegisters = numRegs;
                    status.freeHeap = ESP.getFreeHeap();
                    status.cpuTempDeciC = static_cast<int16_t>(temperatureRead() * 10.0f)
                        + ConfigManager::settings.cpuTempOffsetDeciC;
                    status.core0Mhz = static_cast<uint16_t>(ESP.getCpuFreqMHz());
                    status.core1Mhz = static_cast<uint16_t>(ESP.getCpuFreqMHz());
                    moduleCC1101State[0].readAllConfigRegisters(status.module0Registers, numRegs);
                    moduleCC1101State[1].readAllConfigRegisters(status.module1Registers, numRegs);
                    clients.notifyAllBinary(NotificationType::State,
                        reinterpret_cast<const uint8_t*>(&status), sizeof(status));
                } break;

                case Device::TaskType::Jam: {
                    Device::TaskJam& task = item->jamTask;
                    const std::vector<uint8_t>* pat = task.customPattern ? task.customPattern.get() : nullptr;
                    CC1101Worker::startJam(task.module, task.frequency, task.power,
                                           task.patternType, pat,
                                           task.maxDurationMs, task.cooldownMs);
                } break;

                case Device::TaskType::Idle: {
                    Device::TaskIdle& task = item->idleTask;
                    CC1101Worker::goIdle(task.module);
                } break;

                default:
                    break;
            }
            delete item;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Callback handlers (for CC1101Worker)
// ═══════════════════════════════════════════════════════════════════════

void signalRecordedHandler(bool saved, const std::string& filename) {
    if (saved) {
        BinarySignalRecorded msg;
        msg.module = 0;
        msg.filenameLength = (uint8_t)std::min((size_t)255, filename.length());
        static uint8_t buffer[260];
        memcpy(buffer, &msg, sizeof(msg));
        memcpy(buffer + sizeof(msg), filename.c_str(), msg.filenameLength);
        clients.notifyAllBinary(NotificationType::SignalRecorded, buffer, sizeof(msg) + msg.filenameLength);
    } else {
        static uint8_t errBuf[260];
        errBuf[0] = MSG_ERROR;
        errBuf[1] = 10;
        std::string errMsg = "Failed to save: " + filename;
        uint8_t len = (uint8_t)std::min((size_t)255, errMsg.length());
        memcpy(errBuf + 2, errMsg.c_str(), len);
        clients.notifyAllBinary(NotificationType::FileSystem, errBuf, 2 + len);
    }
}

void cc1101WorkerSignalDetectedHandler(const CC1101DetectedSignal& signal) {
    ESP_LOGI("Main", "Signal: rssi=%d, freq=%.2f, module=%d",
             signal.rssi, signal.frequency, signal.module);
    BinarySignalDetected msg;
    msg.module = signal.module;
    msg.frequency = (uint32_t)(signal.frequency * 1000000);
    msg.rssi = signal.rssi;
    msg.samples = 0;
    clients.notifyAllBinary(NotificationType::SignalDetected,
        reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

// ═══════════════════════════════════════════════════════════════════════
//  Hardware setup helpers
// ═══════════════════════════════════════════════════════════════════════

static void setupCc1101Pins() {
    pinMode(CC1101_SCK, OUTPUT);
    pinMode(CC1101_MOSI, OUTPUT);
    pinMode(CC1101_MISO, INPUT);
    pinMode(CC1101_SS0, OUTPUT);
    pinMode(CC1101_SS1, OUTPUT);
}

static void setupFilesystems() {
    if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) {
        ESP_LOGW(TAG, "LittleFS mount failed, formatting...");
        if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
            ESP_LOGE(TAG, "LittleFS format failed!");
            return;
        }
    }
    ESP_LOGI(TAG, "LittleFS mounted");

    ConfigManager::loadSettings();
    int serialBaud = ConfigManager::settings.serialBaudRate;
    Serial.begin(serialBaud);

    esp_log_level_set("*", ESP_LOG_INFO);
    Serial.printf("Serial started at %d bps\n", serialBaud);

    // SD card (non-blocking, optional)
    sdspi.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_SS);
    if (!SD.begin(SD_SS, sdspi)) {
        ESP_LOGW(TAG, "SD card not mounted — LittleFS-only mode");
        deviceConfig.sdCardMounted = false;
    } else {
        deviceConfig.sdCardMounted = true;
        ESP_LOGI(TAG, "SD card initialized");
        static const char* dirs[] = { "/DATA", "/DATA/RECORDS", "/DATA/SIGNALS", "/DATA/PRESETS", "/DATA/TEMP" };
        for (auto& d : dirs) {
            if (!SD.exists(d)) SD.mkdir(d);
        }
    }
}

static void setupModules() {
    setupCc1101Pins();

    // GPIO ISR service
    static bool isrReady = false;
    if (!isrReady) {
        esp_err_t r = gpio_install_isr_service(0);
        isrReady = (r == ESP_OK || r == ESP_ERR_INVALID_STATE);
    }

    DeviceControls::setup();
    DeviceControls::onLoadPowerManagement();
    DeviceControls::onLoadServiceMode();

    if (ConfigManager::isServiceMode()) return;

    // CC1101 modules
    for (int i = 0; i < CC1101_NUM_MODULES; i++) {
        moduleCC1101State[i].init();
    }
    deviceConfig.powerBlink = true;

    CC1101Worker::init(cc1101WorkerSignalDetectedHandler, signalRecordedHandler);
    CC1101Worker::start();
    ESP_LOGI(TAG, "CC1101Worker started");

    // Command handler
    ControllerAdapter::initializeQueue();
    ClientsManager::getInstance().initializeQueue(NOTIFICATIONS_QUEUE);

    StateCommands::registerCommands(commandHandler);
    FileCommands::registerCommands(commandHandler);
    TransmitterCommands::registerCommands(commandHandler);
    RecorderCommands::registerCommands(commandHandler);
    BruterCommands::registerCommands(commandHandler);
#if PROTOPIRATE_MODULE_ENABLED
    ProtoPirateCommands::registerCommands(commandHandler);
#endif
    NrfCommands::registerCommands(commandHandler);
    OtaCommands::registerCommands(commandHandler);
    ButtonCommands::registerCommands(commandHandler);
#if SDR_MODULE_ENABLED
    SdrCommands::registerCommands(commandHandler);
#endif
    ESP_LOGI(TAG, "CommandHandler: %zu commands", commandHandler.getCommandCount());

    // Bruter
    bruter_init();
    getBruterModule().checkAndNotifySavedState();

#if PROTOPIRATE_MODULE_ENABLED
    ProtoPirateModule::getInstance().init();
#endif

    ConfigManager::applyToRuntime();

#if NRF_MODULE_ENABLED
    NrfJammer::loadConfigs();
    if (NrfModule::init()) {
        MouseJack::init();
        ESP_LOGI(TAG, "nRF24L01 + MouseJack initialized");
    } else {
        ESP_LOGW(TAG, "nRF24L01 not detected");
    }
#endif

#if BATTERY_MODULE_ENABLED
    BatteryModule::init();
#endif

#if SDR_MODULE_ENABLED
    SdrModule::init();
#endif
}

static void setupTasks() {
    // Core 1 (app core)
    xTaskCreatePinnedToCore(taskProcessor,     "TaskProc",   6144, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(serialCommandTask, "SerialCmd",  4096, NULL, 1, NULL, 1);

    // Core 0 (BLE core)
    xTaskCreatePinnedToCore(ClientsManager::processMessageQueue, "Notify", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(timeSyncTask,      "TimeSync",   1024, NULL, 1, NULL, 0);

    // BLE adapter
    bleAdapter.begin();
    bleAdapter.setCommandHandler(&commandHandler);
    clients.addAdapter(&bleAdapter);
    bleAdapterStarted = true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Arduino entry points
// ═══════════════════════════════════════════════════════════════════════

void setup() {
    setupFilesystems();
    setupModules();
    setupTasks();

    ESP_LOGI(TAG, "===== INITIAL HEAP STATE =====");
    logHeapStats("Setup complete");
}

void loop() {
    ButtonCommands::checkButtons();

#if SDR_MODULE_ENABLED
    if (SdrModule::isActive() && SdrModule::isStreaming()) {
        SdrModule::pollRawRx();
    }
#endif

    if (deviceConfig.powerBlink) {
        BruterModule& bruter = getBruterModule();
        if (NrfJammer::isRunning()) {
            DeviceControls::nrfJamActiveBlink();
        } else if (bruter.isAttackRunning()) {
            DeviceControls::bruterActiveBlink();
        } else {
            DeviceControls::poweronBlink();
        }
    }
}
