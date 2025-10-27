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

#include <Arduino.h>

#include <WiFi.h>
#include <esp_now.h>

#include <VCS.h>

/*---------------------------- Macros & Constants ----------------------------*/

/*----------------------------------- State ----------------------------------*/

static constexpr size_t m_channel_count = 4;

static const uint8_t m_bulb[m_channel_count] = {32, 33, 25, 26};

/*------------------------------ Private Functions ---------------------------*/

/*-------------------------------- Entry Point -------------------------------*/

void setup()
{
    Serial.begin(115200);

    Serial.println("slippers-breakfast-newspaper-walkies");
    Serial.printf("Commit: %s\n", vcs.short_hash);
    Serial.printf("Tag: %s\n", vcs.tag_describe);
    Serial.printf("Mac Address: %s\n", WiFi.macAddress().c_str());
}

void loop()
{
    // put your main code here, to run repeatedly:

    vTaskDelay(100);
}

/*----------------------------------------------------------------------------*/
