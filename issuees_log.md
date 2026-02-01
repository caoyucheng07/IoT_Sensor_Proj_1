# Issues & Debugging Log

## Issue 1: MQTT disconnects on browser client
**Cause:** Wrong port (8883 vs 8884 for WebSocket)
**Fix:** Switched to correct port for browser

## Issue 2: IR frame truncated
**Cause:** TIM2 16-bit overflow
**Fix:** Extended timer to 32-bit using overflow ISR
**Result:** Reliable capture of long AC frames

## Issue 3: Random memory values after reset
**Cause:** Watchpoint triggered before ClearBuffers executed
**Fix:** Verified execution order, moved buffer clear after init
**Result:** Stable startup

## Issue 4: Aircon beeps but does not turn on
**Cause:** IR carrier frequency mismatch (38kHz vs 36kHz)
**Fix:** Adjusted TIM1 PWM ARR to generate 36kHz carrier
**Result:** Successful appliance control
