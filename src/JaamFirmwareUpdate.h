#pragma once

#include <Arduino.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <functional>
#include "JaamDisplay.h"
#include "JaamApi.h"

struct JaamFirmware {
    int major = 0;
    int minor = 0;
    int patch = 0;
    int beta = 0;
    int lux = 0;   // [rsa_lux] номер неофіційної збірки (суфікс "-luxN"), 0 для офіційних
};

// Канали оновлення (налаштування FW_UPDATE_CHANNEL)
static constexpr int FW_CHANNEL_PROD = 0;
static constexpr int FW_CHANNEL_BETA = 1;
static constexpr int FW_CHANNEL_RSA_LUX = 2;   // [rsa_lux] неофіційний канал: маніфест і прошивки з GitHub Rsa76d

class JaamFirmwareUpdate {
public:
    void setDisplay(JaamDisplay* display);
    void setApi(JaamApi* api);
    void setMapUpdateCallback(std::function<void(float)> cb);
    void setRebootCallback(std::function<void()> cb);
    void setServicePinCallback(std::function<void()> cb);

    void init(const char* version);
    void initCallbacks();

    // Parses websocket TYPE_FIRMWARE_UPDATE_BETA/PROD_BATCH payload (without header byte).
    // Оновлює спільний стан і сповіщає API лише якщо isBeta збігається з активним каналом (_activeIsBeta).
    void processBatch(const uint8_t* data, size_t bodyLen, bool isBeta);

    bool requestUpdate(const char* id, int channel);
    void applyActiveChannel(int channel);

    // [rsa_lux] Перевірка маніфесту неофіційного каналу (викликається періодично з основного циклу)
    void processCustomChannel(bool wifiConnected);
    const char* getCustomVersion() const;
    bool isUpdateRequested() const;
    void clearUpdateRequest();
    void download();
    bool isValidFirmwareId(const char* id, int channel) const;

    bool isUpdateAvailable() const;
    const char* getCurrentVersion() const;
    const char* getNewVersion() const;
    const char* getUpdateId() const;
    const JaamFirmware* getFirmwaresBeta() const;
    const JaamFirmware* getFirmwaresProd() const;
    const JaamFirmware& getFirmware() const;

private:
    char _currentFwVersion[25] = {};
    char _newFwVersion[25] = {};
    char _fwUpdateId[25] = {};
    bool _fwUpdateAvailable = false;
    volatile bool _needUpdate = false;
    static constexpr size_t MAX_FW = 10;
    int _activeChannel = FW_CHANNEL_PROD;

    // [rsa_lux] стан неофіційного каналу
    char _customVersion[25] = {};
    char _customUrl[200] = {};
    uint32_t _customLastCheck = 0;
    bool _customCheckPending = true;
    bool fetchCustomManifest();
    void applyCustomState();
    JaamFirmware _firmwares_beta[MAX_FW] = {};
    JaamFirmware _firmwares_prod[MAX_FW] = {};
    JaamFirmware _firmware = {};
    WiFiClientSecure _updateClient;

    JaamDisplay* _display = nullptr;
    JaamApi* _api = nullptr;
    std::function<void(float)> _mapUpdateCb;
    std::function<void()> _rebootCb;
    std::function<void()> _servicePinCb;

    void handleUpdateStatus(t_httpUpdate_return ret, bool isSpiffsUpdate);

    static bool isNewerFirmware(const JaamFirmware& candidate, const JaamFirmware& current);
    static JaamFirmware parseFirmwareVersion(const char* version);
    static void fillFwVersion(char* result, JaamFirmware fw);
};
