/* Explicit intermediate heartbeat-only implementation; no WiFi/ROS yet.
 * This same task becomes the communication loop, never a proxy beating for it. */
#include "net_task.h"
#include "task.h"
#include "shared_state.h"
#include "hal_time.h"
void net_task(void *param) {
    (void)param;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        shared_core0_beat(hal_now_ms());
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
    }
}
