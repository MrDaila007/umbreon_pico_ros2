/* Explicit intermediate heartbeat-only implementation; no WiFi/ROS yet.
 * Platform hooks are active; Task6 wires this same owner to WiFi/session policy. */
#include "net_task.h"
#include "task.h"
#include "shared_state.h"
#include "hal_time.h"
#include "hal_microros_platform.h"
#include "umb_buzzer.h"
void net_task(void *param) {
    (void)param;
    if (!hal_microros_platform_init()) {
        shared_notify_network_loss(UMB_STOP_AGENT);
        shared_post_cue(UMB_CUE_ERROR);
        /* Do not claim a functioning communication owner if platform setup failed. */
        for (;;) vTaskDelay(pdMS_TO_TICKS(100));
    }
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        shared_core0_beat(hal_now_ms());
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
    }
}
