/* ros_save.c - see ros_save.h. Messages never contain credentials. */
#include "ros_save.h"
#include <stdio.h>
#include "hal_flash_store.h"
#include "settings_apply.h"
#include "shared_state.h"

static ros_save_reply_t fail(const char *text) {
    ros_save_reply_t r = {.success = false};
    snprintf(r.message, sizeof r.message, "%s", text);
    return r;
}

ros_save_reply_t ros_services_save(uint32_t generation, void (*progress)(void)) {
    if (!shared_session_ready(generation)) return fail("refused: session");
    switch (settings_save_with_progress(progress)) {
    case SETTINGS_SAVE_OK: {
        ros_save_reply_t r = {.success = true};
        snprintf(r.message, sizeof r.message, "saved: slot %d seq %lu", hal_settings_slot(),
                 (unsigned long)hal_settings_seq());
        return r;
    }
    case SETTINGS_SAVE_BUSY: return fail("refused: armed or busy");
    case SETTINGS_SAVE_NEUTRAL_TIMEOUT: return fail("error: neutral ack timeout");
    case SETTINGS_SAVE_FLASH_FAILED:
    case SETTINGS_SAVE_ALREADY_SET:
    case SETTINGS_SAVE_INVALID: break;
    }
    return fail("error: flash write failed");
}
