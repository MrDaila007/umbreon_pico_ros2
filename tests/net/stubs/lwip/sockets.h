#pragma once
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stddef.h>
#define lwip_socket umb_test_socket
#define lwip_bind umb_test_bind
#define lwip_close umb_test_close
#define lwip_sendto umb_test_sendto
#define lwip_setsockopt umb_test_setsockopt
#define lwip_recvmsg umb_test_recvmsg
int umb_test_socket(int domain, int type, int protocol);
int umb_test_bind(int fd, const struct sockaddr *addr, socklen_t length);
int umb_test_close(int fd);
int umb_test_sendto(int fd, const void *buf, size_t size, int flags, const struct sockaddr *peer, socklen_t length);
int umb_test_setsockopt(int fd, int level, int option, const void *value, socklen_t length);
ssize_t umb_test_recvmsg(int fd, struct msghdr *msg, int flags);
