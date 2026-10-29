#include "umb_network_identity.h"
#include <string.h>
bool umb_network_identity(const uint8_t mac[6], const uint8_t entropy[16],
                          const char *existing, umb_network_identity_t *out) {
    if (!mac || !existing || !out) return false;
    size_t n = 0;
    while (n < 64 && existing[n]) {
        unsigned char ch = (unsigned char)existing[n];
        if (ch < 32 || ch > 126) return false;
        n++;
    }
    if (n && (n < 8 || n > 63)) return false;
    if (!n && !entropy) return false;
    if (!n) {
        bool all_zero = true, all_ones = true;
        for (int i = 0; i < 16; i++) {
            if (entropy[i] != 0) all_zero = false;
            if (entropy[i] != 255) all_ones = false;
        }
        if (all_zero || all_ones) return false;
    }
    static const char hex[] = "0123456789ABCDEF";
    umb_network_identity_t result = {0};
    memcpy(result.ssid, "umbreon-", 8);
    for (int i = 0; i < 2; i++) {
        result.ssid[8 + i * 2] = hex[mac[4 + i] >> 4];
        result.ssid[9 + i * 2] = hex[mac[4 + i] & 15];
    }
    if (n) memcpy(result.password, existing, n);
    else {
        for (int i = 0; i < 16; i++) {
            result.password[i * 2] = hex[entropy[i] >> 4];
            result.password[i * 2 + 1] = hex[entropy[i] & 15];
        }
        result.generated = true;
    }
    *out = result;
    return true;
}
