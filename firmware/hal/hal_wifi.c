#include "hal_wifi.h"
#include "hal_dhcp_server.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#define AP_IP 0xc0a80401u
static bool s_initialized, s_ap, s_sta, s_pm_disabled;

void hal_wifi_mark_tcpip_thread(void) {
    vTaskCoreAffinitySet(NULL, 1u << 0);
    configASSERT(get_core_num() == 0);
}
bool hal_wifi_init(void) {
    if (s_initialized) return true;
    if (get_core_num() != 0 || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return false;
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE) != 0) return false;
    /* cyw43_arch_init does not power the WLAN chip: the first ioctl downloads its
     * firmware (~0.8 s measured) and only then fills the cached MAC. Do that here,
     * with a read-only query, so later mode switches stay short and the AP SSID
     * never derives from an all-zero MAC. */
    uint32_t pm;
    if (cyw43_wifi_get_pm(&cyw43_state, &pm) != 0) { cyw43_arch_deinit(); return false; }
    s_initialized = true;
    return true;
}
static bool length_between(const char *s, size_t lo, size_t hi) {
    if (!s) return false;
    size_t n = 0; while (n <= hi && s[n]) n++;
    return n >= lo && n <= hi;
}
bool hal_wifi_start_sta(const char *ssid, const char *password) {
    if (!s_initialized || s_ap || !length_between(ssid, 1, 32) || !length_between(password, 0, 63)) return false;
    cyw43_arch_enable_sta_mode(); s_sta = true; s_pm_disabled = false;
    /* Empty password represents an explicitly configured open station network. */
    uint32_t auth = password[0] ? CYW43_AUTH_WPA2_AES_PSK : CYW43_AUTH_OPEN;
    return cyw43_arch_wifi_connect_async(ssid, password[0] ? password : NULL, auth) == 0;
}
void hal_wifi_stop_sta(void) {
    if (!s_initialized || !s_sta) return;
    cyw43_arch_disable_sta_mode(); s_sta = false; s_pm_disabled = false;
}
bool hal_wifi_start_ap(const char *ssid, const char *password) {
    if (!s_initialized || s_sta || !length_between(ssid, 1, 32) || !length_between(password, 8, 63)) return false;
    cyw43_arch_enable_ap_mode(ssid, password, CYW43_AUTH_WPA2_AES_PSK); s_ap = true;
    cyw43_arch_lwip_begin();
    ip4_addr_t ip, mask, gw;
    IP4_ADDR(&ip, 192, 168, 4, 1); IP4_ADDR(&mask, 255, 255, 255, 0); gw = ip;
    struct netif *ap = &cyw43_state.netif[CYW43_ITF_AP];
    netif_set_addr(ap, &ip, &mask, &gw);
    cyw43_arch_lwip_end();
    if (hal_dhcp_server_start(ap, AP_IP)) return true;
    cyw43_arch_disable_ap_mode(); s_ap = false; return false;
}
void hal_wifi_stop_ap(void) {
    if (!s_initialized || !s_ap) return;
    hal_dhcp_server_stop(); cyw43_arch_disable_ap_mode(); s_ap = false;
}
void hal_wifi_status(hal_wifi_status_t *out) {
    memset(out, 0, sizeof *out);
    if (!s_initialized) return;
    cyw43_arch_lwip_begin();
    out->sta_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    out->sta_up = s_sta && out->sta_status == CYW43_LINK_UP;
    out->ap_up = s_ap && cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_AP) == CYW43_LINK_UP;
    if (out->sta_up) {
        struct netif *sta = &cyw43_state.netif[CYW43_ITF_STA];
        out->sta_ip = ntohl(ip4_addr_get_u32(netif_ip4_addr(sta)));
        out->sta_mask = ntohl(ip4_addr_get_u32(netif_ip4_netmask(sta)));
        out->rssi_valid = cyw43_wifi_get_rssi(&cyw43_state, &out->rssi) == 0;
        if (!s_pm_disabled) s_pm_disabled = cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM) == 0;
    }
    if (out->ap_up) out->ap_ip = AP_IP;
    out->power_save_disabled = s_pm_disabled;
    cyw43_arch_lwip_end();
}
bool hal_wifi_mac(uint8_t mac[6]) {
    if (!s_initialized || !mac) return false;
    cyw43_arch_lwip_begin();
    bool ok = cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, mac) == 0;
    cyw43_arch_lwip_end();
    bool zero = true;
    for (int i = 0; i < 6; i++) if (mac[i]) zero = false;
    return ok && !zero; /* zero = chip not brought up yet */
}
int hal_wifi_ap_clients(uint8_t macs[HAL_WIFI_AP_CLIENTS][6]) {
    if (!s_initialized || !s_ap || !macs) return -1;
    int count = HAL_WIFI_AP_CLIENTS;
    cyw43_arch_lwip_begin(); cyw43_wifi_ap_get_stas(&cyw43_state, &count, &macs[0][0]); cyw43_arch_lwip_end();
    return count >= 0 && count <= HAL_WIFI_AP_CLIENTS ? count : -1;
}
