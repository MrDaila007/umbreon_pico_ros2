#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Minimal public API shape for fake lifecycle tests; ARM build checks target ABI. */
struct uxrCustomTransport;
typedef bool (*open_custom_func)(struct uxrCustomTransport *);
typedef bool (*close_custom_func)(struct uxrCustomTransport *);
typedef size_t (*write_custom_func)(struct uxrCustomTransport *, const uint8_t *, size_t, uint8_t *);
typedef size_t (*read_custom_func)(struct uxrCustomTransport *, uint8_t *, size_t, int, uint8_t *);
typedef struct uxrCommunication { void *instance; } uxrCommunication;
typedef struct uxrCustomTransport {
    void *args; bool framing;
    open_custom_func open; close_custom_func close; write_custom_func write; read_custom_func read;
    uxrCommunication comm;
} uxrCustomTransport;
void uxr_set_custom_transport_callbacks(uxrCustomTransport *, bool, open_custom_func, close_custom_func, write_custom_func, read_custom_func);
bool uxr_init_custom_transport(uxrCustomTransport *, void *);
bool uxr_close_custom_transport(uxrCustomTransport *);
