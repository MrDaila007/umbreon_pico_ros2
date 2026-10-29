/* CYW43 adapter: async join and bounded snapshots; policy belongs in umb_net_fsm. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#define HAL_WIFI_AP_CLIENTS 8
typedef struct {
    bool sta_up, ap_up, rssi_valid, power_save_disabled;
    uint32_t sta_ip, sta_mask, ap_ip;
    int32_t rssi;
    int sta_status;
} hal_wifi_status_t;
bool hal_wifi_init(void); /* once, from core0 task after scheduler starts */
bool hal_wifi_start_sta(const char *ssid, const char *password);
void hal_wifi_stop_sta(void);
bool hal_wifi_start_ap(const char *ssid, const char *password);
void hal_wifi_stop_ap(void);
void hal_wifi_status(hal_wifi_status_t *out);
bool hal_wifi_mac(uint8_t mac[6]);
int hal_wifi_ap_clients(uint8_t macs[HAL_WIFI_AP_CLIENTS][6]); /* -1 unavailable */
void hal_wifi_mark_tcpip_thread(void); /* lwIP startup hook: pin before networking starts */
