#include "umb_network_identity.h"
#include <string.h>
_Static_assert(sizeof UMB_AP_PASSWORD_ALPHABET - 1 == 32, "5 bits per symbol");
_Static_assert(UMB_AP_PASSWORD_LEN * 5u <= 16u * 8u, "entropy covers the password");
bool umb_network_ssid(const uint8_t mac[6], char out[13]) {
    if (!mac || !out) return false;
    static const char hex[] = "0123456789ABCDEF";
    memcpy(out, "umbreon-", 8);
    for (int i = 0; i < 2; i++) {
        out[8 + i * 2] = hex[mac[4 + i] >> 4];
        out[9 + i * 2] = hex[mac[4 + i] & 15];
    }
    out[12] = 0;
    return true;
}
static void generate(const uint8_t entropy[16], char *password) {
    static const char alphabet[] = UMB_AP_PASSWORD_ALPHABET;
    for (unsigned i = 0; i < UMB_AP_PASSWORD_LEN; i++) {
        unsigned value = 0;
        for (unsigned b = 0; b < 5; b++) {
            unsigned bit = i * 5u + b;
            value = (value << 1) | ((entropy[bit / 8u] >> (bit % 8u)) & 1u);
        }
        password[i] = alphabet[value];
    }
}
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
    umb_network_identity_t result = {0};
    (void)umb_network_ssid(mac, result.ssid);
    if (n) memcpy(result.password, existing, n);
    else { generate(entropy, result.password); result.generated = true; }
    *out = result;
    return true;
}
