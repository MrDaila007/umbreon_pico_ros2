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
    ASSERT_TRUE(first.generated); ASSERT_EQ(strlen(first.password), 32);
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
int main(void) {
    RUN_TEST(test_identity_preserves_existing_and_uses_mac_suffix);
    RUN_TEST(test_identity_generates_from_entropy_and_rejects_bad_inputs);
    TEST_SUMMARY();
}
