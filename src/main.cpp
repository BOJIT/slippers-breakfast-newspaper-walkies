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
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <nvs.h>

#include <VCS.h>

/*---------------------------- Macros & Constants ----------------------------*/

static constexpr uint8_t WIFI_CHANNEL         = 4;
static constexpr size_t INPUT_LINE_BUFFER_LEN = 256;
static constexpr const char *NVS_STORAGE_KEY  = "esp_comms";

static constexpr uint32_t BLINK_INTERVAL = 500;
static constexpr uint32_t OFF_TIMEOUT    = 2000;

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
static std::atomic<uint32_t> m_msg_tick;

static bool m_valid_initialise         = false;
static esp_now_peer_info_t m_peer_info = {0};

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

static void comms_tx(uint8_t msg)
{
    esp_now_send(m_peer_info.peer_addr, (uint8_t *)(&msg), 1);
}

static void comms_rx(const uint8_t *mac, const uint8_t *data, int len)
{
    if (len != 1)
        return;

    m_msg_tick = millis(); // Record last message time
    m_msg_mask = data[0];
}

static bool comms_init(uint8_t mac[6])
{
    // Set device as a Wi-Fi Station
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Initialise and register RX callback
    if (esp_now_init() != ESP_OK)
        return false;

    esp_now_register_recv_cb(comms_rx);
    // esp_now_register_send_cb(comms_tx);

    memcpy(m_peer_info.peer_addr, mac, 6);
    m_peer_info.channel = WIFI_CHANNEL;
    m_peer_info.encrypt = false;
    esp_now_add_peer(&m_peer_info);

    return true;
}

/*-------------------------------- Entry Point -------------------------------*/

void setup()
{
    // Atomic Initialisation
    m_msg_mask = 0;
    m_msg_tick = millis();

    // Debug / Versioning
    Serial.begin(115200);

    Serial.println("------------------------------------");
    Serial.println("slippers-breakfast-newspaper-walkies");
    Serial.printf("Commit: %s\n", vcs.short_hash);
    Serial.printf("Tag: %s\n", vcs.tag_describe);
    Serial.printf("Mac Address: %s\n", WiFi.macAddress().c_str());

    // IO Init
    for (size_t i = 0; i < m_channel_count; i++)
    {
        pinMode(m_switch[i], INPUT_PULLUP);
        pinMode(m_bulb[i], OUTPUT);
        digitalWrite(m_bulb[i], LOW);
    }

    // NVS Storage of Paired MAC
    nvs_mac_init();

    uint8_t mac[6]   = {0};
    char mac_str[20] = {0};
    if (nvs_mac_get(mac))
    {
        mac_to_str(mac, mac_str, sizeof(mac_str));
        Serial.printf("Paired MAC Address: %s\n", mac_str);
        m_valid_initialise = comms_init(mac);
    }
    else
        Serial.println("No Paired MAC Address!");
}

void loop()
{
    uint32_t tick = millis();

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

    // Handle button messages
    if (!m_valid_initialise)
        return;

    // Read button inputs
    uint8_t input_mask = 0;
    for (size_t i = 0; i < m_channel_count; i++)
    {
        input_mask |= !digitalRead(m_switch[i]) << i;
    }

    // Transmit state message
    static uint32_t m_last_tx = millis();
    static bool m_last_state  = false;
    if (input_mask && (tick - m_last_tx > BLINK_INTERVAL))
    {
        if (m_last_state)
            comms_tx(input_mask);
        else
            comms_tx(0);

        m_last_state = !m_last_state;
        m_last_tx    = tick;
    }

    // If we haven't received anything, ensure the mask is cleared
    if (tick - m_msg_tick > OFF_TIMEOUT)
        m_msg_mask = 0;

    // Set outputs
    uint8_t output_mask = m_msg_mask | input_mask;
    for (size_t i = 0; i < m_channel_count; i++)
        digitalWrite(m_bulb[i], (output_mask & (1 << i)) ? HIGH : LOW);

    vTaskDelay(50); // This loop can run quite slowly
}

/*----------------------------------------------------------------------------*/
