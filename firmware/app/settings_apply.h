/* Shared neutral/reservation coordinator for console and future ROS save service. */
#pragma once
typedef enum { SETTINGS_SAVE_OK, SETTINGS_SAVE_BUSY, SETTINGS_SAVE_NEUTRAL_TIMEOUT,
               SETTINGS_SAVE_FLASH_FAILED } settings_save_result_t;
settings_save_result_t settings_save_current(void);
