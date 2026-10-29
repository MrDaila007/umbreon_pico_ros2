/* console_task.c - see console_task.h */
#include "console_task.h"
#include <stdio.h>
#include "task.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
#include "pico/stdio_usb.h"
#include "console_cmds.h"
#include "hal_time.h"
#include "shared_state.h"

#define LINE_MAX 160u
#define POLL_MS 5u

typedef struct {
    stdio_driver_t *drv;
    console_origin_t origin;
    char buf[LINE_MAX];
    unsigned len;
    bool overflow;      /* drop the rest of an over-long line */
} line_src_t;

static void echo(line_src_t *src, const char *s, int n) {
    if (src->drv->out_chars) src->drv->out_chars(s, n);
}

/* Feeds one received character; returns true when src->buf holds a complete line. */
static bool feed(line_src_t *src, char c) {
    if (c == '\r' || c == '\n') {
        if (src->len == 0u && !src->overflow) return false;   /* blank line or CRLF tail */
        echo(src, "\r\n", 2);
        bool ok = !src->overflow;
        src->buf[src->len] = '\0';
        src->len = 0u;
        src->overflow = false;
        if (!ok) printf("error: line too long\n");
        return ok;
    }
    if (c == '\b' || c == 0x7f) {
        if (src->len > 0u) { src->len--; echo(src, "\b \b", 3); }
        return false;
    }
    if (src->len >= LINE_MAX - 1u) { src->overflow = true; return false; }
    src->buf[src->len++] = c;
    echo(src, &c, 1);
    return false;
}

static void poll_source(line_src_t *src, console_repeat_t *rep) {
    char chunk[32];
    int n = src->drv->in_chars(chunk, (int)sizeof chunk);
    for (int i = 0; i < n; i++) {
        if (feed(src, chunk[i])) console_execute(src->buf, src->origin, rep, hal_now_ms());
    }
}

/* A USB-origin repeat ends silently when the host disconnects (unplug, port
 * closed, helper killed): no neutral is posted, so the 250 ms timeout stops
 * the car and motion then needs a fresh neutral. End of [ms] posts no
 * neutral either (same latch). */
static void run_repeat(console_repeat_t *rep, uint32_t now) {
    if (!rep->active) return;
    if (rep->origin == CONSOLE_ORIGIN_USB && !stdio_usb_connected()) { rep->active = false; return; }
    if ((int32_t)(now - rep->end_ms) >= 0) { rep->active = false; return; }
    if ((int32_t)(now - rep->next_ms) < 0) return;
    rep->cmd.rx_ms = now;
    if (!shared_post_cmd(&rep->cmd)) rep->active = false;
    rep->next_ms = now + CONSOLE_REPEAT_MS;
}

void console_task(void *param) {
    console_cmds_init((const char *)param);
    line_src_t usb = { .drv = &stdio_usb, .origin = CONSOLE_ORIGIN_USB };
    line_src_t uart = { .drv = &stdio_uart, .origin = CONSOLE_ORIGIN_UART };
    console_repeat_t rep = { 0 };
    printf("\numbreon_pico_ros2 console ready; type 'help'\n");

    for (;;) {
        poll_source(&usb, &rep);
        poll_source(&uart, &rep);
        run_repeat(&rep, hal_now_ms());
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}
