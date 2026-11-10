#include "emergency_stop.h"
#include "config.h"
#include <Arduino.h>

namespace {
portMUX_TYPE   s_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool  s_active = false;
volatile uint32_t s_startMs = 0;
} // namespace

void estopTrigger() {
  portENTER_CRITICAL(&s_mux);
  s_startMs = millis();
  s_active = true;
  portEXIT_CRITICAL(&s_mux);
}

uint32_t estopRemainingMs() {
  portENTER_CRITICAL(&s_mux);
  uint32_t remaining = 0;
  if (s_active) {
    const uint32_t elapsed = millis() - s_startMs;   // wrap-safe
    if (elapsed >= EMERGENCY_LOCKOUT_MS) s_active = false;
    else remaining = EMERGENCY_LOCKOUT_MS - elapsed;
  }
  portEXIT_CRITICAL(&s_mux);
  return remaining;
}

bool estopIsActive() { return estopRemainingMs() > 0; }
