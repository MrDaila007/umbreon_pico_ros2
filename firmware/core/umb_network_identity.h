/* Entropy is supplied by hardware adapter; this pure helper never logs secrets. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
/* WPA2 passphrase alphabet: 32 symbols (5 bits each) without 0/O/1/I/l/o, so a
 * password copied from the OLED/QR fallback text cannot be misread. */
#define UMB_AP_PASSWORD_ALPHABET "23456789ABCDEFGHJKLMNPQRSTUVWXYZ"
#define UMB_AP_PASSWORD_LEN 16u /* 80 bits drawn from the first 10 entropy bytes */
typedef struct { char ssid[13]; char password[64]; bool generated; } umb_network_identity_t;
/* "umbreon-XXXX" from the last two MAC bytes; false on NULL. */
bool umb_network_ssid(const uint8_t mac[6], char out[13]);
/* Stable XRCE client key for this device: FNV-1a over the whole MAC, never 0
 * (0 only for NULL). Reconnects reuse it so the agent replaces the old session. */
uint32_t umb_network_client_key(const uint8_t mac[6]);
/* Empty existing password generates UMB_AP_PASSWORD_LEN chars from entropy.
 * Existing WPA2 passwords are accepted only at 8..63 bytes; NULL inputs rejected. */
bool umb_network_identity(const uint8_t mac[6], const uint8_t entropy[16],
                          const char *existing_password, umb_network_identity_t *out);
