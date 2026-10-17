#include <string>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "JaamFirmwareUpdate.h"
#include "JaamLogs.h"

// [rsa_lux] Неофіційний канал оновлення. Маніфест і прошивки лежать на GitHub автора доопрацьованої версії.
// Дозволені лише адреси з цим префіксом: навіть якщо маніфест підмінять, завантаження з інших сайтів не піде.
static const char* const RSA_LUX_URL_PREFIX = "https://raw.githubusercontent.com/Rsa76d/";
static const char* const RSA_LUX_MANIFEST_URL = "https://raw.githubusercontent.com/Rsa76d/jaam_fusion/lux-firmware/manifest.json";
static const uint32_t RSA_LUX_CHECK_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;  // перевірка раз на 6 годин
static const uint32_t RSA_LUX_RETRY_INTERVAL_MS = 10UL * 60UL * 1000UL;        // після помилки повтор через 10 хв
static const uint32_t RSA_LUX_FIRST_CHECK_DELAY_MS = 20000;                    // не раніше 20 с після запуску

// Ключ у маніфесті залежно від чипа
#if ARDUINO_ESP32S3_DEV
static const char* const RSA_LUX_CHIP_KEY = "s3";
#elif ARDUINO_ESP32C3_DEV
static const char* const RSA_LUX_CHIP_KEY = "c3";
#else
static const char* const RSA_LUX_CHIP_KEY = "esp32";
#endif

static bool isAllowedCustomUrl(const char* url) {
    return url != nullptr && strncmp(url, RSA_LUX_URL_PREFIX, strlen(RSA_LUX_URL_PREFIX)) == 0;
}

void JaamFirmwareUpdate::setDisplay(JaamDisplay* display) {
    _display = display;
}

void JaamFirmwareUpdate::setApi(JaamApi* api) {
    _api = api;
}

void JaamFirmwareUpdate::setMapUpdateCallback(std::function<void(float)> cb) {
    _mapUpdateCb = cb;
}

void JaamFirmwareUpdate::setRebootCallback(std::function<void()> cb) {
    _rebootCb = cb;
}

void JaamFirmwareUpdate::setServicePinCallback(std::function<void()> cb) {
    _servicePinCb = cb;
}

void JaamFirmwareUpdate::init(const char* version) {
    _firmware = parseFirmwareVersion(version);
    LOG.printf("[INIT] major: %d, minor: %d, patch: %d, beta: %d\n",
               _firmware.major, _firmware.minor, _firmware.patch, _firmware.beta);
    fillFwVersion(_currentFwVersion, _firmware);
    LOG.printf("[INIT] Current firmware version: %s\n", _currentFwVersion);
    snprintf(_fwUpdateId, sizeof(_fwUpdateId), "%s", version);
    LOG.printf("[INIT] Firmware Update ID initialized: %s\n", _fwUpdateId);
}

void JaamFirmwareUpdate::initCallbacks() {
    httpUpdate.onStart([this]() {
        if (_api) _api->setFirmwareProgress(0);
        if (_display) _display->showServiceMessage("Починаємо!", "Оновлення:");
        delay(1000);
    });
    httpUpdate.onEnd([this]() {
        if (_api) _api->setFirmwareProgress(100);
        if (_display) _display->showServiceMessage("Перезавантаження..", "Оновлення:");
        delay(1000);
    });
    httpUpdate.onProgress([this](int progress, int total) {
        if (total == 0) return;
        uint8_t percent = static_cast<uint8_t>((progress * 100) / total);
        if (percent > 100) percent = 100;
        LOG.printf("[UPDATE] Progress: %u%%\n", percent);
        char progressText[5];
        snprintf(progressText, sizeof(progressText), "%u%%", percent);
        if (_api) _api->setFirmwareProgress(percent);
        if (_display) _display->showServiceMessage(progressText, "Оновлення:");
        if (_mapUpdateCb) _mapUpdateCb(percent / 100.0f);
    });
    httpUpdate.onError([this](int error) {
        if (_api) _api->setFirmwareProgress(-1);
        if (!_display) return;
        switch (error) {
            case HTTP_UE_TOO_LESS_SPACE:
                _display->showServiceMessage("Замало місця", "Помилка оновлення:", 5000);
                break;
            case HTTP_UE_SERVER_NOT_REPORT_SIZE:
                _display->showServiceMessage("Невідомий розмір файлу", "Помилка оновлення:", 5000);
                break;
            case HTTP_UE_BIN_FOR_WRONG_FLASH:
                _display->showServiceMessage("Неправильний тип пам'яті", "Помилка оновлення:", 5000);
                break;
            default:
                _display->showServiceMessage("Щось пішло не так", "Помилка оновлення:", 5000);
                break;
        }
    });
}

void JaamFirmwareUpdate::processBatch(const uint8_t* data, size_t bodyLen, bool isBeta) {
    bool isActiveChannel = (_activeChannel == (isBeta ? FW_CHANNEL_BETA : FW_CHANNEL_PROD));
    static constexpr size_t RECORD_FW = 5;
    size_t count = bodyLen / RECORD_FW;
    if (count > MAX_FW) {
        LOG.printf("[FIRMWARE] Batch has %u records, clamping to %u\n", (unsigned)count, (unsigned)MAX_FW);
        count = MAX_FW;
    }
    const uint8_t* ptr = data;

    JaamFirmware* target = isBeta ? _firmwares_beta : _firmwares_prod;
    memset(target, 0, MAX_FW * sizeof(JaamFirmware));

    LOG.printf("[WEBSOCKET] TYPE_FIRMWARE_UPDATE_%s_BATCH data processing\n", isBeta ? "BETA" : "PROD");

    for (size_t i = 0; i < count; ++i) {
        target[i].major = ptr[0];
        target[i].minor = ptr[1];
        target[i].patch = ptr[2];
        // Little-Endian: low byte first, high byte second
        target[i].beta = ptr[3] | (ptr[4] << 8);
        LOG.printf("Parsed FW: %d.%d.%d b%d\n",
                   target[i].major, target[i].minor,
                   target[i].patch, target[i].beta);
        ptr += RECORD_FW;
    }

    // Знаходимо найновішу версію серед отриманих
    JaamFirmware latestInBatch{};
    for (size_t i = 0; i < count; ++i) {
        if ((target[i].major | target[i].minor | target[i].patch | target[i].beta) == 0) continue;
        if (isNewerFirmware(target[i], latestInBatch)) {
            latestInBatch = target[i];
        }
    }

    // Оновлюємо спільний стан лише для активного каналу
    if (isActiveChannel) {
        fillFwVersion(_newFwVersion, latestInBatch);
        snprintf(_fwUpdateId, sizeof(_fwUpdateId), "%s", _newFwVersion);
        _fwUpdateAvailable = isNewerFirmware(latestInBatch, _firmware);

        if (_fwUpdateAvailable) {
            LOG.printf("[FIRMWARE] Update available: %s -> %s\n", _currentFwVersion, _newFwVersion);
        } else {
            LOG.printf("[FIRMWARE] No updates available. Current version: %s, Latest version: %s\n",
                       _currentFwVersion, _newFwVersion);
        }

        if (_api) _api->setNewFirmwareInfo(_newFwVersion);
        if (_servicePinCb) _servicePinCb();
    }
}

void JaamFirmwareUpdate::applyActiveChannel(int channel) {
    if (channel < FW_CHANNEL_PROD || channel > FW_CHANNEL_RSA_LUX) channel = FW_CHANNEL_PROD;
    const bool channelChanged = (channel != _activeChannel);
    _activeChannel = channel;

    // [rsa_lux] Неофіційний канал: стан береться з маніфесту, а не зі списків сервера
    if (_activeChannel == FW_CHANNEL_RSA_LUX) {
        if (channelChanged) {
            _customCheckPending = true;     // перевірити маніфест якнайшвидше
            _fwUpdateAvailable = false;     // не показувати оновлення з іншого каналу
        }
        if (_customVersion[0] != '\0') applyCustomState();
        return;
    }

    const bool isBeta = (_activeChannel == FW_CHANNEL_BETA);
    const JaamFirmware* arr = isBeta ? _firmwares_beta : _firmwares_prod;

    JaamFirmware latestInChannel{};
    for (size_t i = 0; i < MAX_FW; ++i) {
        if ((arr[i].major | arr[i].minor | arr[i].patch | arr[i].beta) == 0) continue;
        if (isNewerFirmware(arr[i], latestInChannel)) {
            latestInChannel = arr[i];
        }
    }

    // Якщо список каналу ще не отримано — не перезаписуємо стан і не сповіщаємо API
    if ((latestInChannel.major | latestInChannel.minor | latestInChannel.patch | latestInChannel.beta) == 0) {
        LOG.printf("[FIRMWARE] Channel switched to %s, but list is empty — skipping state update\n",
                   isBeta ? "BETA" : "PROD");
        return;
    }

    fillFwVersion(_newFwVersion, latestInChannel);
    snprintf(_fwUpdateId, sizeof(_fwUpdateId), "%s", _newFwVersion);
    _fwUpdateAvailable = isNewerFirmware(latestInChannel, _firmware);

    LOG.printf("[FIRMWARE] Channel switched to %s, latest: %s, update available: %d\n",
               isBeta ? "BETA" : "PROD", _newFwVersion, _fwUpdateAvailable);

    if (_api) _api->setNewFirmwareInfo(_newFwVersion);
    if (_servicePinCb) _servicePinCb();
}

bool JaamFirmwareUpdate::requestUpdate(const char* id, int channel) {
    if (!isValidFirmwareId(id, channel)) {
        if (_display) _display->showServiceMessage("Невідома версія", "Помилка оновлення:", 5000);
        LOG.printf("[FIRMWARE] Invalid firmware ID: %s\n", id ? id : "(null)");
        return false;
    }
    
    snprintf(_fwUpdateId, sizeof(_fwUpdateId), "%s", id);
    _needUpdate = true;
    LOG.printf("[FIRMWARE] Update requested for: %s\n", id);
    return true;
}

bool JaamFirmwareUpdate::isUpdateRequested() const {
    return _needUpdate;
}

void JaamFirmwareUpdate::clearUpdateRequest() {
    _needUpdate = false;
}

void JaamFirmwareUpdate::download() {
    disableLoopWDT();

    LOG.println("Starting firmware update...");
    char firmwareUrlChar[220];

    LOG.println("Building firmware url...");
#if ARDUINO_ESP32S3_DEV
    snprintf(firmwareUrlChar, sizeof(firmwareUrlChar), "https://update.jaam.net.ua/s3/%s", _fwUpdateId);
#elif ARDUINO_ESP32C3_DEV
    snprintf(firmwareUrlChar, sizeof(firmwareUrlChar), "https://update.jaam.net.ua/c3/%s", _fwUpdateId);
#else
    snprintf(firmwareUrlChar, sizeof(firmwareUrlChar), "https://update.jaam.net.ua/%s", _fwUpdateId);
#endif

    // [rsa_lux] Неофіційний канал: адреса береться з перевіреного маніфесту
    if (_activeChannel == FW_CHANNEL_RSA_LUX) {
        if (!isAllowedCustomUrl(_customUrl)) {
            LOG.printf("[FIRMWARE] rsa_lux: url is not allowed: %s\n", _customUrl);
            if (_display) _display->showServiceMessage("Невірна адреса", "Помилка оновлення:", 5000);
            enableLoopWDT();
            return;
        }
        snprintf(firmwareUrlChar, sizeof(firmwareUrlChar), "%s", _customUrl);
    }

    LOG.printf("Firmware url: %s\n", firmwareUrlChar);
    _updateClient.setInsecure();
    httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    t_httpUpdate_return fwRet = httpUpdate.update(_updateClient, firmwareUrlChar, _currentFwVersion);
    handleUpdateStatus(fwRet, false);

    enableLoopWDT();
}

bool JaamFirmwareUpdate::isUpdateAvailable() const {
    return _fwUpdateAvailable;
}

const char* JaamFirmwareUpdate::getCurrentVersion() const {
    return _currentFwVersion;
}

const char* JaamFirmwareUpdate::getNewVersion() const {
    return _newFwVersion;
}

const char* JaamFirmwareUpdate::getUpdateId() const {
    return _fwUpdateId;
}

const JaamFirmware* JaamFirmwareUpdate::getFirmwaresBeta() const {
    return _firmwares_beta;
}

const JaamFirmware* JaamFirmwareUpdate::getFirmwaresProd() const {
    return _firmwares_prod;
}

const JaamFirmware& JaamFirmwareUpdate::getFirmware() const {
    return _firmware;
}

void JaamFirmwareUpdate::handleUpdateStatus(t_httpUpdate_return ret, bool isSpiffsUpdate) {
    LOG.printf("%s update status:\n", isSpiffsUpdate ? "Spiffs" : "Firmware");
    switch (ret) {
        case HTTP_UPDATE_FAILED:
            LOG.printf("Error Occurred. Error (%d): %s\n",
                       httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            LOG.println("HTTP_UPDATE_NO_UPDATES");
            break;
        case HTTP_UPDATE_OK:
            if (isSpiffsUpdate) {
                LOG.println("Spiffs update successfully completed. Starting firmware update...");
            } else {
                LOG.println("Firmware update successfully completed. Rebooting...");
                if (_rebootCb) _rebootCb();
            }
            break;
    }
}

// Повертає true якщо 'candidate' новіший за 'current'.
// Стабільний (beta==0) вважається новішим за будь-який beta з тим самим major.minor.patch.
bool JaamFirmwareUpdate::isNewerFirmware(const JaamFirmware& candidate, const JaamFirmware& current) {
    if (candidate.major != current.major) return candidate.major > current.major;
    if (candidate.minor != current.minor) return candidate.minor > current.minor;
    if (candidate.patch != current.patch) return candidate.patch > current.patch;
    // Однакові major.minor.patch: stable > beta
    if (candidate.beta == 0 && current.beta > 0) return true;
    if (candidate.beta > 0 && current.beta == 0) return false;
    // Обидва beta або обидва stable: більший номер beta → новіший
    if (candidate.beta != current.beta) return candidate.beta > current.beta;
    // [rsa_lux] Однакова офіційна основа: новішою вважається збірка з більшим номером lux
    return candidate.lux > current.lux;
}

JaamFirmware JaamFirmwareUpdate::parseFirmwareVersion(const char* version) {
    JaamFirmware fw{};
    if (version == nullptr) return fw;
    char* versionCopy = strdup(version);
    if (versionCopy == nullptr) return fw;
    char* token = strtok(versionCopy, ".-");
    int part = 0;
    while (token) {
        if (isdigit(token[0])) {
            if (part == 0) fw.major = atoi(token);
            else if (part == 1) fw.minor = atoi(token);
            else if (part == 2) fw.patch = atoi(token);
            part++;
        } else if (token[0] == 'b' && strcmp(token, "bin") != 0) {
            fw.beta = atoi(token + 1);
        } else if (strncmp(token, "lux", 3) == 0 && isdigit(token[3])) {
            fw.lux = atoi(token + 3);   // [rsa_lux] суфікс "-luxN"
        }
        token = strtok(NULL, ".-");
    }
    free(versionCopy);
    return fw;
}

void JaamFirmwareUpdate::fillFwVersion(char* result, JaamFirmware fw) {
    std::string version = std::to_string(fw.major) + "." + std::to_string(fw.minor);

    if (fw.patch > 0) {
        version += "." + std::to_string(fw.patch);
    }

    if (fw.beta > 0) {
        version += "-b" + std::to_string(fw.beta);
    }

    #if ARDUINO_ESP32S3_DEV
    version += "-s3";
    #elif ARDUINO_ESP32C3_DEV
    version += "-c3";
    #endif

    #if LITE
    version += "-lite";
    #endif

    #if TEST_MODE
    version += "-test";
    #endif

    strcpy(result, version.c_str());
}

bool JaamFirmwareUpdate::isValidFirmwareId(const char* id, int channel) const {
    if (id == nullptr || strlen(id) == 0) {
        return false;
    }

    // [rsa_lux] Неофіційний канал: дозволена лише версія з перевіреного маніфесту
    if (channel == FW_CHANNEL_RSA_LUX) {
        return _customVersion[0] != '\0' && strcmp(id, _customVersion) == 0 && isAllowedCustomUrl(_customUrl);
    }
    const bool isBeta = (channel == FW_CHANNEL_BETA);

    JaamFirmware candidate = parseFirmwareVersion(id);

    if (candidate.major == 0 && candidate.minor == 0) {
        return false;
    }

    // Перевіряємо лише список активного каналу
    const JaamFirmware* arr = isBeta ? _firmwares_beta : _firmwares_prod;
    for (size_t i = 0; i < MAX_FW; ++i) {
        const JaamFirmware& fw = arr[i];
        if ((fw.major | fw.minor | fw.patch | fw.beta) == 0) continue;
        if (fw.major == candidate.major &&
            fw.minor == candidate.minor &&
            fw.patch == candidate.patch &&
            fw.beta == candidate.beta) {
            return true;
        }
    }

    return false;
}

// ───────────────────────── [rsa_lux] неофіційний канал оновлення ─────────────────────────

const char* JaamFirmwareUpdate::getCustomVersion() const {
    return _customVersion;
}

// Оновлює спільний стан «є/немає оновлення» за версією з маніфесту
void JaamFirmwareUpdate::applyCustomState() {
    JaamFirmware fw = parseFirmwareVersion(_customVersion);
    snprintf(_newFwVersion, sizeof(_newFwVersion), "%s", _customVersion);
    snprintf(_fwUpdateId, sizeof(_fwUpdateId), "%s", _customVersion);
    _fwUpdateAvailable = isNewerFirmware(fw, _firmware);

    LOG.printf("[FIRMWARE] rsa_lux: latest %s, current %s, update available: %d\n",
               _customVersion, _currentFwVersion, _fwUpdateAvailable);

    if (_api) _api->setNewFirmwareInfo(_newFwVersion);
    if (_servicePinCb) _servicePinCb();
}

// Завантажує маніфест з GitHub. Формат: {"version":"5.1.2-b3-lux9","esp32":"https://...bin","c3":"https://...bin"}
bool JaamFirmwareUpdate::fetchCustomManifest() {
    LOG.printf("[FIRMWARE] rsa_lux: checking manifest\n");
    _updateClient.stop();
    _updateClient.setInsecure();

    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(8000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (!http.begin(_updateClient, RSA_LUX_MANIFEST_URL)) {
        LOG.printf("[FIRMWARE] rsa_lux: http.begin failed\n");
        return false;
    }

    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        LOG.printf("[FIRMWARE] rsa_lux: manifest HTTP code %d\n", code);
        http.end();
        return false;
    }
    int len = http.getSize();
    if (len > 1500) {
        LOG.printf("[FIRMWARE] rsa_lux: manifest too large (%d)\n", len);
        http.end();
        return false;
    }
    String body = http.getString();
    http.end();
    _updateClient.stop();

    JsonDocument doc;
    if (deserializeJson(doc, body)) {
        LOG.printf("[FIRMWARE] rsa_lux: manifest is not valid JSON\n");
        return false;
    }

    const char* version = doc["version"] | "";
    const char* url = doc[RSA_LUX_CHIP_KEY] | "";
    JaamFirmware fw = parseFirmwareVersion(version);
    if ((fw.major == 0 && fw.minor == 0) || strlen(version) >= sizeof(_customVersion)) {
        LOG.printf("[FIRMWARE] rsa_lux: bad version in manifest\n");
        return false;
    }
    if (!isAllowedCustomUrl(url) || strlen(url) >= sizeof(_customUrl)) {
        LOG.printf("[FIRMWARE] rsa_lux: no allowed url for chip %s\n", RSA_LUX_CHIP_KEY);
        return false;
    }

    snprintf(_customVersion, sizeof(_customVersion), "%s", version);
    snprintf(_customUrl, sizeof(_customUrl), "%s", url);
    return true;
}

// Викликається з основного циклу: перевіряє маніфест при запуску, при виборі каналу і раз на 6 годин
void JaamFirmwareUpdate::processCustomChannel(bool wifiConnected) {
    if (_activeChannel != FW_CHANNEL_RSA_LUX || !wifiConnected || _needUpdate) return;

    const uint32_t now = millis();
    if (now < RSA_LUX_FIRST_CHECK_DELAY_MS) return;
    const bool due = _customCheckPending || (uint32_t)(now - _customLastCheck) >= RSA_LUX_CHECK_INTERVAL_MS;
    if (!due) return;

    _customCheckPending = false;
    _customLastCheck = now;
    if (fetchCustomManifest()) {
        applyCustomState();
    } else {
        // повторити через 10 хвилин
        _customLastCheck = now - (RSA_LUX_CHECK_INTERVAL_MS - RSA_LUX_RETRY_INTERVAL_MS);
    }
}
