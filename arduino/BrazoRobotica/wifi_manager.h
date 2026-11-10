#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

// =====================================================================
//  Wi-Fi SoftAP + DNS (custom hostname) + HTTP/WebSocket server
//
//  - SoftAP at 192.168.4.1. A tiny DNS server answers the configured
//    hostname (default "robot.arm.esp32") so the UI is reachable at
//    http://robot.arm.esp32/ . mDNS (<hostname-with-dashes>.local) and the
//    raw IP also work.
//  - WebSocket messages are received in the AsyncTCP task and queued; the
//    main loop pulls them with wifiNextMessage() so all hardware access
//    stays in one task. EMERGENCY STOP is the exception: it is applied
//    immediately inside the WebSocket callback.
// =====================================================================

#include <Arduino.h>

struct WsMessage {
  uint32_t clientId;
  char*    text;      // heap allocated, release with free()
};

bool   wifiBegin();
void   wifiLoop();
bool   wifiNextMessage(WsMessage& msg);

void   wsSend(uint32_t clientId, const char* text);
void   wsBroadcast(const char* text);
size_t wsClientCount();

String wifiUrl();     // e.g. "http://robot.arm.esp32/"

#endif // WIFI_MANAGER_H
