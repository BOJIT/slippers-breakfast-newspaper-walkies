/**
 * @file main.cpp
 * @author James Bennion-Pedley
 * @brief Brief summary here
 * @date 27/10/2025
 *
 * @copyright Copyright (c) 2025
 *
 */

/*--------------------------------- Includes ---------------------------------*/

#include <atomic>
#include <Arduino.h>

#include <WiFi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <nvs.h>

#include <VCS.h>

/*---------------------------- Macros & Constants ----------------------------*/

static constexpr size_t INPUT_LINE_BUFFER_LEN = 256;
static constexpr const char *NVS_STORAGE_KEY  = "esp_comms";

/*----------------------------------- Types ----------------------------------*/

// Uint64 representation of a MAC address
typedef union _uU64
{
    uint64_t val;
    uint8_t bytes[8];
} tU64;

/*----------------------------------- State ----------------------------------*/

static constexpr size_t m_channel_count = 4;

static const uint8_t m_switch[m_channel_count] = {23, 21, 19, 5};

static const uint8_t m_bulb[m_channel_count] = {32, 33, 25, 26};

static std::atomic<uint8_t> m_msg_mask;
static std::atomic<uint32_t> m_msg_timeout;

/*------------------------------ Private Functions ---------------------------*/

static const bool mac_to_str(const uint8_t mac[6], char *str, size_t max_len)
{
    snprintf(str, max_len, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return true;
}

static bool str_to_mac(const char *str, uint8_t mac[6])
{
    return (sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6);
}

static bool nvs_mac_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    return err == ESP_OK;
}

static bool nvs_mac_get(uint8_t mac[6])
{
    tU64 mac_access;
    mac_access.val = 0;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_STORAGE_KEY, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return false;

    // Get MAC Address
    err = nvs_get_u64(handle, "mac", &mac_access.val);
    memcpy(mac, mac_access.bytes, 6);

    nvs_close(handle);
    return err == ESP_OK;
}

static bool nvs_mac_set(const uint8_t mac[6])
{
    tU64 mac_access;
    mac_access.val = 0;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_STORAGE_KEY, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return false;

    // Set MAC Address
    memcpy(mac_access.bytes, mac, 6);
    err = nvs_set_u64(handle, "mac", mac_access.val);

    nvs_close(handle);
    return err == ESP_OK;
}

static bool nvs_mac_clear(void)
{
    uint8_t mac[6];
    memset(mac, 0, 6);
    return nvs_mac_set(mac);
}

/*-------------------------------- Entry Point -------------------------------*/

void setup()
{
    // Debug / Versioning
    Serial.begin(115200);

    Serial.println("------------------------------------");
    Serial.println("slippers-breakfast-newspaper-walkies");
    Serial.printf("Commit: %s\n", vcs.short_hash);
    Serial.printf("Tag: %s\n", vcs.tag_describe);
    Serial.printf("Mac Address: %s\n", WiFi.macAddress().c_str());

    // NVS Storage of Paired MAC
    nvs_mac_init();

    uint8_t mac[6]   = {0};
    char mac_str[20] = {0};
    if (nvs_mac_get(mac))
    {
        mac_to_str(mac, mac_str, sizeof(mac_str));
        Serial.printf("Paired MAC Address: %s\n", mac_str);
    }
    else
        Serial.println("No Paired MAC Address!");

    // Atomic Initialisation
    m_msg_mask    = 0;
    m_msg_timeout = millis();
}

void loop()
{
    // Parse Serial Input for any info
    static int m_index = 0;
    static char m_input_buf[INPUT_LINE_BUFFER_LEN];
    if (m_index >= sizeof(m_input_buf))
        m_index = 0;

    while (Serial.available() > 0)
    {
        m_input_buf[m_index] = Serial.read();

        // If at the end of a line, parse the string. Otherwise, increment
        if (m_input_buf[m_index] == '\n')
        {
            m_input_buf[m_index] = '\0';

            uint8_t mac[6] = {0};
            if (str_to_mac(m_input_buf, mac) && nvs_mac_set(mac))
                Serial.printf("Stored MAC to NVS: %s\n", m_input_buf);
            else
                Serial.println("Could not Parse MAC address!\n");

            m_index = 0;
        }
        else
            m_index++;
    }
}

/*----------------------------------------------------------------------------*/
