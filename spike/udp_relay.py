#!/usr/bin/env python3
"""Phase 0 spike helper: UDP relay between the Pico (WiFi) and a local micro-ROS agent.

Why: on the spike PC a VPN interface routes 10.0.0.0/8, so the agent's replies to
the Pico's WiFi subnet leave through the VPN and are lost. This relay listens on
LISTEN_PORT with its socket bound (SO_BINDTODEVICE, unprivileged) to the wired LAN
interface, so replies use that interface's default gateway. It forwards datagrams
to the agent on 127.0.0.1:AGENT_PORT.

usage: udp_relay.py <lan-iface> <listen-port> <agent-port>
"""
import select
import socket
import sys


def main():
    iface, listen_port, agent_port = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    outer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    outer.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface.encode())
    outer.bind(("0.0.0.0", listen_port))
    inner = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    inner.bind(("127.0.0.1", 0))
    client = None
    print(f"relay {iface}:{listen_port} <-> 127.0.0.1:{agent_port}", flush=True)
    while True:
        ready, _, _ = select.select([outer, inner], [], [])
        if outer in ready:
            data, addr = outer.recvfrom(2048)
            if addr != client:
                print(f"client {addr}", flush=True)
                client = addr
            inner.sendto(data, ("127.0.0.1", agent_port))
        if inner in ready:
            data, _ = inner.recvfrom(2048)
            if client:
                outer.sendto(data, client)


if __name__ == "__main__":
    main()
