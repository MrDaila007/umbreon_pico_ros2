/* main.c - boot: settings, neutral PWM, tachometer, shared state, tasks, scheduler. */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "console_task.h"
#include "control_task.h"
#include "hal_flash_store.h"
#include "hal_pwm.h"
#include "hal_tach.h"
#include "shared_state.h"
#include "umb_settings.h"

void vApplicationMallocFailedHook(void) {
    panic("FreeRTOS: malloc failed");
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
    (void)task;
    panic("FreeRTOS: stack overflow in %s", name);
}

/* Read before the control task calls watchdog_enable, which rewrites the scratch flag. */
static const char *reset_reason(void) {
    if (watchdog_enable_caused_reboot()) return "watchdog timeout";
    if (watchdog_caused_reboot()) return "watchdog reboot (reboot command or SDK)";
    return "power-on / reset pin / debugger";
}

static void create_pinned(TaskFunction_t fn, const char *name, uint32_t stack_words, void *param,
                          UBaseType_t prio, UBaseType_t core_mask) {
    TaskHandle_t h = NULL;
    if (xTaskCreateAffinitySet(fn, name, stack_words, param, prio, core_mask, &h) != pdPASS)
        panic("cannot create task %s", name);
}

int main(void) {
    const char *reason = reset_reason();

    umb_settings_t settings;
    (void)hal_settings_load(&settings);      /* false -> defaults, reported by `status` */
    hal_pwm_init();                           /* neutral 1500 / 1500 before the slice runs */
    hal_tach_init(settings.tach_glitch_filter_us);
    shared_init(&settings);
    stdio_init_all();

    create_pinned(control_task, "control", CONTROL_TASK_STACK_WORDS, NULL,
                  CONTROL_TASK_PRIORITY, CONTROL_TASK_CORE_MASK);
    create_pinned(console_task, "console", CONSOLE_TASK_STACK_WORDS, (void *)reason,
                  CONSOLE_TASK_PRIORITY, CONSOLE_TASK_CORE_MASK);
    vTaskStartScheduler();
    panic("scheduler returned");
}
