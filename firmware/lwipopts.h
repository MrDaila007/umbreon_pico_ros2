/* Production lwIP options (initially carried from the measured Phase 0 spike): NO_SYS=0 (FreeRTOS sys_arch) with the BSD socket API.
 * Based on pico-examples pico_w/wifi lwipopts_examples_common.h + freertos/ntp_client_socket. */
#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

#define NO_SYS                      0
#define LWIP_SOCKET                 1
#define LWIP_NETCONN                0     /* sockets work without the netconn API exposed */
#define MEM_LIBC_MALLOC             0     /* incompatible with non-poll cyw43 arch */
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    16000
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define MEMP_NUM_NETCONN            8
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_CHKSUM_ALGORITHM       3
#define LWIP_DHCP                   1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_TCP_KEEPALIVE          1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0
#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0

/* FreeRTOS sys_arch. Pin tcpip_thread at its first startup hook, before IP work. */
#ifdef __cplusplus
extern "C" {
#endif
void hal_wifi_mark_tcpip_thread(void);
#ifdef __cplusplus
}
#endif
#define LWIP_MARK_TCPIP_THREAD() hal_wifi_mark_tcpip_thread()
#define TCPIP_THREAD_STACKSIZE      8192 /* bytes: includes bounded DHCP callback frames */
#define TCPIP_THREAD_PRIO           (configMAX_PRIORITIES - 4)
#define DEFAULT_THREAD_STACKSIZE    1024
#define DEFAULT_RAW_RECVMBOX_SIZE   8
#define DEFAULT_UDP_RECVMBOX_SIZE   16
#define DEFAULT_TCP_RECVMBOX_SIZE   8
#define DEFAULT_ACCEPTMBOX_SIZE     8
#define TCPIP_MBOX_SIZE             16
#define LWIP_TIMEVAL_PRIVATE        0     /* use newlib's struct timeval */
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

/* Sockets used by the micro-ROS UDP transport */
#define LWIP_SO_RCVTIMEO            1
#define LWIP_SO_SNDTIMEO            1
#define SO_REUSE                    1
#define IP_SOF_BROADCAST            1     /* only discovery context explicitly enables SO_BROADCAST */

#endif
