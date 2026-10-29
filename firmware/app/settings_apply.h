/* Shared neutral/reservation coordinator for console and future ROS save service. */
#pragma once
typedef enum { SETTINGS_SAVE_OK, SETTINGS_SAVE_BUSY, SETTINGS_SAVE_NEUTRAL_TIMEOUT,
               SETTINGS_SAVE_FLASH_FAILED, SETTINGS_SAVE_ALREADY_SET,
               SETTINGS_SAVE_INVALID } settings_save_result_t;
settings_save_result_t settings_save_current(void);
/* First-boot AP credential: under one maintenance reservation, persist the settings
 * flash already holds (never unsaved RAM edits) plus `password` (8..63 printable
 * bytes), verify, and only then publish the password to shared settings. If flash
 * or RAM already has an ap_pass: SETTINGS_SAVE_ALREADY_SET, no write, and a RAM copy
 * that lost the persisted password is restored. The password is never logged. */
settings_save_result_t settings_provision_ap_pass(const char *password);
/* RAM lost the persisted AP password (provisioned device): put it back from the
 * flash copy under a reservation. Never generates and never writes flash.
 * OK = restored, ALREADY_SET = RAM has one, INVALID = flash has none. */
settings_save_result_t settings_restore_ap_pass(void);
