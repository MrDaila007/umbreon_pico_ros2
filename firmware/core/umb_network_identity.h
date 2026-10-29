/* Entropy is supplied by hardware adapter; this pure helper never logs secrets. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { char ssid[13]; char password[64]; bool generated; } umb_network_identity_t;
/* Empty existing password generates 32 hex chars from 16 entropy bytes.
 * Existing WPA2 passwords are accepted only at 8..63 bytes; NULL inputs rejected. */
bool umb_network_identity(const uint8_t mac[6], const uint8_t entropy[16],
                          const char *existing_password, umb_network_identity_t *out);
