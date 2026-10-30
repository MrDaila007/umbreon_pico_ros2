#include "test_runner.h"
#include "umb_network_identity.h"
#include <string.h>
TEST(test_identity_preserves_existing_and_uses_mac_suffix) {
    uint8_t mac[6] = {0,1,2,3,0xab,0xcd}; umb_network_identity_t id;
    ASSERT_TRUE(umb_network_identity(mac, NULL, "owner-test-pass", &id));
    ASSERT_TRUE(strcmp(id.ssid, "umbreon-ABCD") == 0);
    ASSERT_TRUE(strcmp(id.password, "owner-test-pass") == 0); ASSERT_FALSE(id.generated);
}
TEST(test_identity_generates_from_entropy_and_rejects_bad_inputs) {
    uint8_t mac[6] = {0}; uint8_t entropy[16] = {1,2,3}; umb_network_identity_t first, second;
    ASSERT_TRUE(umb_network_identity(mac, entropy, "", &first));
    ASSERT_TRUE(first.generated); ASSERT_EQ(strlen(first.password), UMB_AP_PASSWORD_LEN);
    for (size_t i = 0; i < UMB_AP_PASSWORD_LEN; i++)
        ASSERT_TRUE(strchr(UMB_AP_PASSWORD_ALPHABET, first.password[i]) != NULL);
    entropy[5] = 42;
    ASSERT_TRUE(umb_network_identity(mac, entropy, "", &second));
    ASSERT_TRUE(strcmp(first.password, second.password) != 0);
    memset(entropy, 0, sizeof entropy);
    ASSERT_FALSE(umb_network_identity(mac, entropy, "", &first));
    memset(entropy, 255, sizeof entropy);
    ASSERT_FALSE(umb_network_identity(mac, entropy, "", &first));
    ASSERT_FALSE(umb_network_identity(mac, entropy, "bad\tpass", &first));
    ASSERT_FALSE(umb_network_identity(mac, NULL, "", &first));
    ASSERT_FALSE(umb_network_identity(mac, entropy, "short", &first));
    char long_pass[65]; memset(long_pass, 'x', 64); long_pass[64] = 0;
    ASSERT_FALSE(umb_network_identity(mac, entropy, long_pass, &first));
    ASSERT_FALSE(umb_network_identity(NULL, entropy, "owner-test-pass", &first));
}
TEST(test_identity_alphabet_is_unambiguous_and_uses_all_entropy_bits) {
    ASSERT_EQ(strlen(UMB_AP_PASSWORD_ALPHABET), 32);
    ASSERT_TRUE(strpbrk(UMB_AP_PASSWORD_ALPHABET, "01IOlo") == NULL);
    uint8_t mac[6] = {0}; uint8_t entropy[16]; umb_network_identity_t a, b;
    /* Each of the 80 consumed bits must influence the password. */
    for (int bit = 0; bit < 80; bit++) {
        memset(entropy, 0x5a, sizeof entropy);
        ASSERT_TRUE(umb_network_identity(mac, entropy, "", &a));
        entropy[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        ASSERT_TRUE(umb_network_identity(mac, entropy, "", &b));
        ASSERT_TRUE(strcmp(a.password, b.password) != 0);
    }
    char ssid[13]; uint8_t m2[6] = {1,2,3,4,0x0f,0xa0};
    ASSERT_TRUE(umb_network_ssid(m2, ssid)); ASSERT_TRUE(strcmp(ssid, "umbreon-0FA0") == 0);
    ASSERT_FALSE(umb_network_ssid(NULL, ssid));
}
TEST(test_client_key_is_device_unique_nonzero_and_stable) {
    uint8_t a[6] = {2, 1, 2, 3, 0xab, 0xcd}, b[6] = {2, 1, 2, 3, 0xab, 0xce};
    uint8_t c[6] = {3, 1, 2, 3, 0xab, 0xcd}, zero[6] = {0};
    uint32_t ka = umb_network_client_key(a);
    ASSERT_TRUE(ka != 0); ASSERT_EQ(umb_network_client_key(a), ka);
    ASSERT_TRUE(umb_network_client_key(b) != ka); ASSERT_TRUE(umb_network_client_key(c) != ka);
    ASSERT_TRUE(umb_network_client_key(zero) != 0);
    ASSERT_TRUE(umb_network_client_key(NULL) == 0u);
    /* Every MAC bit changes the key: two cars on one agent never share a session. */
    for (int bit = 0; bit < 48; bit++) {
        uint8_t m[6]; memcpy(m, a, 6); m[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        ASSERT_TRUE(umb_network_client_key(m) != ka);
    }
}
int main(void) {
    RUN_TEST(test_client_key_is_device_unique_nonzero_and_stable);
    RUN_TEST(test_identity_preserves_existing_and_uses_mac_suffix);
    RUN_TEST(test_identity_generates_from_entropy_and_rejects_bad_inputs);
    RUN_TEST(test_identity_alphabet_is_unambiguous_and_uses_all_entropy_bits);
    TEST_SUMMARY();
}
