#ifndef SENDER_H
#define SENDER_H
#include <stdint.h>
#include <esp_now.h>
void Sender_Init();
void Send(const char* message);

#endif