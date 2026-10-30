/* Core-0 network owner loop: bounded iterations, sole core-0 heartbeat writer.
 * WiFi/endpoint/ROS-session orchestration lives in net_owner; the rcl backend in
 * ros_entities_rcl. A session step blocks at most one entity creation
 * (ROS_CREATE_TIMEOUT_MS) or an executor spin with an arm ACK wait (60 ms) plus
 * one 20 ms ping/sync, so the beat below stays well inside the 500 ms limit. */
#include "net_task.h"
#include <stdio.h>
#include "task.h"
#include "net_owner.h"
#include "ros_entities_rcl.h"
#include "shared_state.h"
#include "hal_time.h"
#include "hal_wifi.h"
#include "hal_microros_platform.h"
#include "umb_buzzer.h"

#define NET_LOOP_MS 10u
#define NET_RADIO_RETRY_MS 5000u

static net_owner_t s_owner; /* static: keeps the large state off the task stack */

static const char *provision_reason(umb_prov_result_t r) {
    switch (r) {
    case UMB_PROV_RESULT_BUSY: return "car armed or no neutral acknowledgement";
    case UMB_PROV_RESULT_NO_ENTROPY: return "no hardware entropy";
    case UMB_PROV_RESULT_FLASH_FAILED: return "flash write/verify failed";
    default: return "unknown";
    }
}

/* One line per provisioning outcome; never the password itself. */
static void report_provisioning(const net_owner_t *o, uint32_t *reported_failures, bool *reported_ok) {
    static umb_prov_result_t last_printed = UMB_PROV_RESULT_NONE;
    if (o->provision.failures != *reported_failures) {
        *reported_failures = o->provision.failures;
        /* Busy retries every second while armed: report the first one only. */
        if (o->provision.last == UMB_PROV_RESULT_BUSY && last_printed == UMB_PROV_RESULT_BUSY) return;
        last_printed = o->provision.last;
        printf("network: AP password provisioning failed (%s); AP disabled, %s\n",
               provision_reason(o->provision.last),
               o->provision.stage == UMB_PROV_GAVE_UP ? "no further attempts this boot" : "will retry");
        if (o->provision.last != UMB_PROV_RESULT_BUSY) shared_post_cue(UMB_CUE_ERROR);
    }
    if (!*reported_ok && o->provision.stage == UMB_PROV_DONE && o->provision.last == UMB_PROV_RESULT_OK) {
        *reported_ok = true;
        printf("network: AP password generated and saved to flash\n");
    }
}

void net_task(void *param) {
    (void)param;
    if (!hal_microros_platform_init()) {
        shared_notify_network_loss(UMB_STOP_AGENT);
        shared_post_cue(UMB_CUE_ERROR);
        /* Do not claim a functioning communication owner if platform setup failed. */
        for (;;) vTaskDelay(pdMS_TO_TICKS(100));
    }
    net_owner_init(&s_owner, ros_rcl_backend(&s_owner.session));
    uint32_t next_radio_init = hal_now_ms(), reported_failures = 0;
    bool reported_ok = false;
    for (;;) {
        uint32_t start = hal_now_ms();
        shared_core0_beat(start);
        umb_outputs_t outputs;
        shared_get_outputs(&outputs);
        /* Boot is disarmed; a later retry is deferred while armed because the
         * bring-up blocks ~0.6 s (beyond the 500 ms core-0 stale-beat limit). */
        if (!s_owner.radio_ready && !outputs.armed && (int32_t)(start - next_radio_init) >= 0) {
            /* One-time CYW43 bring-up (firmware download, not bounded by us); it is
             * reported separately and excluded from the steady-state loop metric. */
            bool ok = hal_wifi_init();
            net_owner_set_radio_ready(&s_owner, ok, hal_now_ms() - start);
            if (!ok) next_radio_init = hal_now_ms() + NET_RADIO_RETRY_MS;
            start = hal_now_ms();
            shared_core0_beat(start);
        }
        net_owner_step(&s_owner); /* at most one bounded blocking operation */
        uint32_t end = hal_now_ms();
        shared_core0_beat(end);
        net_owner_record_loop(&s_owner, end - start);
        report_provisioning(&s_owner, &reported_failures, &reported_ok);
        /* Always yield at least one tick so the lower-priority console stays responsive. */
        uint32_t elapsed = hal_now_ms() - start;
        vTaskDelay(pdMS_TO_TICKS(elapsed >= NET_LOOP_MS ? 1u : NET_LOOP_MS - elapsed));
    }
}
