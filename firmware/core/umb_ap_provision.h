/* First-boot AP password provisioning policy. Pure: the owner generates the
 * password and performs the maintenance-reserved flash write; this module only
 * decides when an attempt is allowed and bounds flash writes per boot. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#define UMB_PROV_BUSY_RETRY_MS 1000u      /* armed / no neutral ACK: nothing written */
#define UMB_PROV_FIRST_BACKOFF_MS 10000u
#define UMB_PROV_MAX_BACKOFF_MS 60000u
#define UMB_PROV_MAX_WRITES 3u            /* flash write attempts per boot, then give up */
typedef enum { UMB_PROV_RESULT_NONE, UMB_PROV_RESULT_OK, UMB_PROV_RESULT_ALREADY,
               UMB_PROV_RESULT_BUSY, UMB_PROV_RESULT_NO_ENTROPY,
               UMB_PROV_RESULT_FLASH_FAILED } umb_prov_result_t;
typedef enum { UMB_PROV_IDLE, UMB_PROV_BACKOFF, UMB_PROV_DONE, UMB_PROV_GAVE_UP } umb_prov_stage_t;
typedef struct {
    umb_prov_stage_t stage;
    umb_prov_result_t last;
    uint32_t retry_ms, backoff_ms, failures;
    uint8_t writes;
} umb_ap_provision_t;
umb_ap_provision_t umb_ap_provision_init(void);
/* True when the owner may attempt one generate+persist+verify now. */
bool umb_ap_provision_due(umb_ap_provision_t s, bool have_password, uint32_t now_ms);
umb_ap_provision_t umb_ap_provision_result(umb_ap_provision_t s, umb_prov_result_t r, uint32_t now_ms);
