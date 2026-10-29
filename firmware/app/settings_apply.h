/* Shared neutral/reservation coordinator for console and future ROS save service. */
#pragma once
typedef enum { SETTINGS_SAVE_OK, SETTINGS_SAVE_BUSY, SETTINGS_SAVE_NEUTRAL_TIMEOUT,
               SETTINGS_SAVE_FLASH_FAILED, SETTINGS_SAVE_ALREADY_SET,
               SETTINGS_SAVE_INVALID } settings_save_result_t;
settings_save_result_t settings_save_current(void);
/* First-boot AP credential: under one maintenance reservation, persist the complete
 * current settings plus `password` (8..63 printable bytes), verify the flash copy,
 * and only then publish it to the shared settings. Never replaces an existing
 * ap_pass (SETTINGS_SAVE_ALREADY_SET, no write). The password is never logged. */
settings_save_result_t settings_provision_ap_pass(const char *password);
