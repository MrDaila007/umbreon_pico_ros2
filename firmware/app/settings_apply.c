#include "settings_apply.h"
#include "shared_state.h"
#include "hal_flash_store.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#define NEUTRAL_ACK_MS 60u

/* Reserve maintenance and wait (bounded) for control's neutral acknowledgement. */
static settings_save_result_t reserve_neutral(uint32_t *token) {
    if (!shared_begin_maintenance(token)) return SETTINGS_SAVE_BUSY;
    TickType_t start = xTaskGetTickCount();
    while (!shared_maintenance_acknowledged(*token) &&
           (TickType_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(NEUTRAL_ACK_MS))
        vTaskDelay(pdMS_TO_TICKS(1));
    if (shared_maintenance_acknowledged(*token)) return SETTINGS_SAVE_OK;
    shared_end_maintenance(*token);
    return SETTINGS_SAVE_NEUTRAL_TIMEOUT;
}

settings_save_result_t settings_save_current(void) {
    uint32_t token;
    settings_save_result_t reserved = reserve_neutral(&token);
    if (reserved != SETTINGS_SAVE_OK) return reserved;
    umb_settings_t settings;
    shared_get_settings(&settings);
    bool saved = hal_settings_save(&settings);
    shared_end_maintenance(token);
    return saved ? SETTINGS_SAVE_OK : SETTINGS_SAVE_FLASH_FAILED;
}

static bool valid_wpa2_pass(const char *p) {
    if (!p) return false;
    size_t n = 0;
    while (n < 64 && p[n]) {
        if ((unsigned char)p[n] < 32 || (unsigned char)p[n] > 126) return false;
        n++;
    }
    return n >= 8 && n <= 63;
}

settings_save_result_t settings_provision_ap_pass(const char *password) {
    if (!valid_wpa2_pass(password)) return SETTINGS_SAVE_INVALID;
    uint32_t token;
    settings_save_result_t reserved = reserve_neutral(&token);
    if (reserved != SETTINGS_SAVE_OK) return reserved;
    /* Write what flash already holds plus the credential, never unsaved RAM
     * edits: provisioning must not silently save a pending `set`/`defaults`. */
    umb_settings_t persisted, ram;
    settings_save_result_t result = SETTINGS_SAVE_FLASH_FAILED;
    shared_get_settings(&ram);
    if (hal_settings_persisted(&persisted)) {
        if (persisted.ap_pass[0]) {
            /* Already provisioned in flash: restore RAM if it lost it; no write. */
            if (!ram.ap_pass[0]) (void)shared_provision_ap_pass(token, persisted.ap_pass);
            result = SETTINGS_SAVE_ALREADY_SET;
        } else if (ram.ap_pass[0]) {
            result = SETTINGS_SAVE_ALREADY_SET; /* owner set one in RAM; adopt it */
        } else {
            strncpy(persisted.ap_pass, password, sizeof persisted.ap_pass - 1);
            persisted.ap_pass[sizeof persisted.ap_pass - 1] = 0;
            /* Commits are refused while the reservation is held, so RAM cannot
             * gain a different password between this check and the publish. */
            bool ok = hal_settings_save(&persisted) && shared_provision_ap_pass(token, password);
            result = ok ? SETTINGS_SAVE_OK : SETTINGS_SAVE_FLASH_FAILED;
        }
    }
    memset(&persisted, 0, sizeof persisted); memset(&ram, 0, sizeof ram);
    shared_end_maintenance(token);
    return result;
}

settings_save_result_t settings_restore_ap_pass(void) {
    uint32_t token;
    settings_save_result_t reserved = reserve_neutral(&token);
    if (reserved != SETTINGS_SAVE_OK) return reserved;
    umb_settings_t persisted, ram;
    shared_get_settings(&ram);
    settings_save_result_t result = SETTINGS_SAVE_ALREADY_SET;
    if (!ram.ap_pass[0]) {
        result = SETTINGS_SAVE_INVALID;
        if (hal_settings_persisted(&persisted) && persisted.ap_pass[0] &&
            shared_provision_ap_pass(token, persisted.ap_pass)) result = SETTINGS_SAVE_OK;
    }
    memset(&persisted, 0, sizeof persisted); memset(&ram, 0, sizeof ram);
    shared_end_maintenance(token);
    return result;
}
