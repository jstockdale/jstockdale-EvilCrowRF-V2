#ifndef FileCommands_h
#define FileCommands_h

#include "StringBuffer.h"
#include "core/ble/CommandHandler.h"
#include "core/ble/ClientsManager.h"
#include "BinaryMessages.h"
#include "core/ble/BleAdapter.h"
#include "SD.h"
#include <LittleFS.h>
#include "Arduino.h"
#include <cstring>
#include <vector>
#include "ff.h"

extern ClientsManager& clients;

/**
 * File commands using static buffers instead of dynamic strings.
 * Declarations only — implementations in FileCommands.cpp.
 */
class FileCommands {
public:
    static void registerCommands(CommandHandler& handler);

private:
    static JsonBuffer jsonBuffer;
    static PathBuffer pathBuffer;
    static LogBuffer logBuffer;

    // Helpers
    static fs::FS& getFS(uint8_t pathType);
    static bool bufferedFileCopy(File& src, File& dst);
    static void buildBasePath(uint8_t pathType, PathBuffer& buffer);
    static void buildFullPath(uint8_t pathType, const char* relativePath, size_t pathLen, PathBuffer& buffer);
    static void extractFilename(const char* fullPath, PathBuffer& filename);
    static void buildDirectoryTreeBinaryRecursive(const char* path, uint8_t* buffer, size_t& offset, uint16_t& count, size_t maxBufferSize = 1024);

    // Notification helpers
    static void sendBinaryFileListError(uint8_t errorCode);
    static void sendBinaryFileActionResult(uint8_t action, bool success, uint8_t errorCode, const char* path = nullptr);
    static void sendBinaryDirectoryTreeError(uint8_t errorCode);

    // Filesystem helpers
    static bool removeDirectoryRecursive(fs::FS& fs, const char* path);
    static void collectDirectoryPaths(const char* basePath, std::vector<String>& paths);

    // Command handlers (0x05, 0x09, 0x0A, 0x0B, 0x0C, 0x0E, 0x0F, 0x10, 0x14, 0x18)
    static bool handleGetFilesList(const uint8_t* data, size_t len);
    static bool handleLoadFileData(const uint8_t* data, size_t len);
    static bool handleRemoveFile(const uint8_t* data, size_t len);
    static bool handleRenameFile(const uint8_t* data, size_t len);
    static bool handleCreateDirectory(const uint8_t* data, size_t len);
    static bool handleCopyFile(const uint8_t* data, size_t len);
    static bool handleMoveFile(const uint8_t* data, size_t len);
    static bool handleSaveToSignalsWithName(const uint8_t* data, size_t len);
    static bool handleGetDirectoryTree(const uint8_t* data, size_t len);
    static bool handleFormatSDCard(const uint8_t* data, size_t len);
    static bool handleUploadFile(const uint8_t* data, size_t len);
};

#endif // FileCommands_h
