#pragma once
#include "lwip/raw_types.h"
struct pbuf { struct pbuf *next; void *payload; u16_t len, tot_len; };
#define PBUF_TRANSPORT 0
#define PBUF_RAM 0
struct pbuf *pbuf_alloc(int layer, u16_t length, int type);
u16_t pbuf_copy_partial(const struct pbuf *p, void *out, u16_t length, u16_t offset);
err_t pbuf_take(struct pbuf *p, const void *input, u16_t length);
u8_t pbuf_free(struct pbuf *p);
