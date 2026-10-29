#include "settings_apply.h"
#include "shared_state.h"
#include "hal_flash_store.h"
#include "FreeRTOS.h"
#include "task.h"
#define NEUTRAL_ACK_MS 60u

settings_save_result_t settings_save_current(void) {
    uint32_t token;
    if (!shared_begin_maintenance(&token)) return SETTINGS_SAVE_BUSY;
    TickType_t start = xTaskGetTickCount();
    while (!shared_maintenance_acknowledged(token) &&
           (TickType_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(NEUTRAL_ACK_MS))
        vTaskDelay(pdMS_TO_TICKS(1));
    if (!shared_maintenance_acknowledged(token)) {
        shared_end_maintenance(token);
        return SETTINGS_SAVE_NEUTRAL_TIMEOUT;
    }
    umb_settings_t settings;
    shared_get_settings(&settings);
    bool saved = hal_settings_save(&settings);
    shared_end_maintenance(token);
    return saved ? SETTINGS_SAVE_OK : SETTINGS_SAVE_FLASH_FAILED;
}
