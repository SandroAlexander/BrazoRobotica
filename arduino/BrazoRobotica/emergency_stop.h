#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

// =====================================================================
//  Emergency stop
//  - estopTrigger() freezes the arm at its current position and rejects
//    every movement command for EMERGENCY_LOCKOUT_MS.
//  - Safe to call from any task (WebSocket callback, loop, ISR-free code).
//  - Re-triggering during the lockout restarts the timer.
// =====================================================================

#include <stdint.h>

void     estopTrigger();
bool     estopIsActive();        // true while movement is locked out
uint32_t estopRemainingMs();     // 0 when not active

#endif // EMERGENCY_STOP_H
