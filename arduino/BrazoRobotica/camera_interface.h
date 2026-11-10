#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

// =====================================================================
//  OV2640 camera (AI-Thinker ESP32-CAM)
//  - MJPEG stream served by a dedicated esp_http_server on STREAM_PORT
//    (http://<host>:81/stream) so streaming never blocks the control UI.
//  - Single-frame capture via captureFrame()/releaseFrame() (used by the
//    /capture route of the main web server).
//  - The flash LED starts OFF and is only switched on on request.
// =====================================================================

#include "esp_camera.h"
#include "settings.h"

bool         cameraBegin();                                  // uses g_settings.camera
bool         cameraIsReady();
bool         cameraHasPsram();
bool         cameraApply(const CameraSettings& s);           // sensor params + LED
void         cameraSetLed(bool on);
bool         cameraStartStreamServer();
bool         cameraIsStreaming();                            // a client is receiving the stream

camera_fb_t* captureFrame();                                 // nullptr on failure
void         releaseFrame(camera_fb_t* fb);

#endif // CAMERA_INTERFACE_H
