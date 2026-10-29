#pragma once
#include <stdint.h>
typedef uint16_t u16_t;
typedef uint8_t u8_t;
typedef int err_t;
#define ERR_OK 0
#define ERR_MEM (-1)
#define IPADDR_TYPE_V4 0
typedef struct { uint32_t addr; } ip_addr_t;
#define IP_ANY_TYPE ((const ip_addr_t *)0)
#define IP_ADDR4(p,a,b,c,d) ((p)->addr = (uint32_t)(a)<<24 | (uint32_t)(b)<<16 | (uint32_t)(c)<<8 | (uint32_t)(d))
