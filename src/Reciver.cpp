#include "Reciver.h"
#include <WiFi.h>
#include <esp_now.h>

// Buffer to store received data
static char receivedBuffer[64];
static bool newData = false;

static void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    int copyLen = min(len, (int)sizeof(receivedBuffer) - 1);
    memcpy(receivedBuffer, data, copyLen);
    receivedBuffer[copyLen] = '\0';   
    newData = true;
}

void Receiver_Init()
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("Receiver Ready");
}


String Receive()
{
    if (!newData)
        return "";        

    newData = false;
    return String(receivedBuffer);
}
