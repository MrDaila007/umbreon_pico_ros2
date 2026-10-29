# Plan 3 Task 4 — production networking adapters

Date: 2026-10-29. Pure/fake-network evidence; radio has not been initialized.

## UDP and discovery

- Production callbacks compile with real SDK2.2.0 lwIP and generated XRCE headers.
  Active peer IP/port are fixed until close. Short/error sends are failures;
  SO_SNDTIMEO is configured20ms and option failures close/retry ownership safely.
- timeout<=0 reads use MSG_DONTWAIT. Positive reads preserve an overall deadline
  across rejected peers, empty/truncated/oversized packets. recvmsg handles the
  actual lwIP MSG_TRUNC contract. A16-packet cap also bounds nonblocking floods.
- A failed close retains the descriptor for retry (EBADF means already closed).
  It must not be overwritten with init until cleanup succeeds.
- Dedicated private XRCE probe uses one bounded ping attempt, enables broadcast
  only explicitly, and selects a source IP only after XRCE engine success. It
  never changes the registered production transport. The private retained context
  retries failed close before another probe; hal_udp_probe_close is required
  before mode changes. All probe calls belong to the single network owner.
- Host tests control the dependency's ping result to verify validation gating and
  resource ownership; they do not pretend to implement or prove XRCE wire parsing.
  Actual discovery over the owner's STA/AP network remains an interoperability check.

## DHCP and CYW43

- DHCP codec is host-testable and carries the original SDK/MicroPython MIT credit.
  It bounds input600/reply300 bytes, parses PAD/END and exact option lengths,
  rejects duplicate critical options/invalid MAC/relay packets, and distinguishes
 30s offers from24h bound leases. Pool is AP subnet .2–.9. REQUEST renewal, NAK,
  RELEASE and DECLINE60s quarantine are covered; only bound/fresh leases become
  endpoint candidates. No nonexistent DNS server is advertised.
- RAW wrapper copies chained pbufs, binds PCB/replies to AP netif, frees input and
  output on failures, and uses the CYW43/lwIP core lock for public RAW operations.
  Its receive callback runs under lwIP core context and makes no ROS/RTOS calls.
- WiFi HAL implements asynchronous STA join, explicit STA/AP stop, WPA2 AP with
 192.168.4.1/24, status/RSSI/power-save-disable, MAC and AP-client snapshots.
  Missing/invalid AP password cannot enable an open AP. It does no fallback policy.
- SDK async task follows the calling core; init requires a core0 scheduled task.
  LWIP_MARK_TCPIP_THREAD pins the separate tcpip task before IP processing.
  TCPIP_THREAD_STACKSIZE8192 is bytes, including DHCP callback frames; runtime
  stack/heap measurement remains required. No shared timer-service pin was changed.

## Checks

- Docker firmware build passes with project -Werror. No init/radio call is made
  by the current staged heartbeat-only network task.
- 331 C tests in26 executables and10 Python contract tests pass. Core/driver line
  coverage1533/1543 (99.35%); DHCP codec97/97 lines. HAL tests are additional
  integration evidence outside that core/driver coverage metric.
- UDP8 fake-socket tests include shrinking deadlines, immutable endpoint, close
  failures and separate probe validation; DHCP7 codec tests include every packet
  truncation and10000 deterministic malformed option mutations.
- RAW2 integration tests exercise100 chained-pbuf ACKs, one live PCB/zero retained
  pbufs, idempotent start/stop, create/bind/alloc/take/send errors and core locking.
- `tests/net/run_sanitizers.sh` passes AddressSanitizer/UBSan for the DHCP codec.
  The codec allocates no heap; LeakSanitizer is disabled for restricted runners.
- Current ELF text114136/data0/bss209384 bytes. Dead-code collection means this
  is not the final active network runtime footprint. Logs: build/plan3-task4-fw.log,
  plan3-task4-coverage.log; earlier Phase0 link budget still separate.

A socket timeout does not guarantee interruption of a hung lwIP core/thread;
communication heartbeat and independent core1 protection remain the final safety
boundary. Actual radio association, AP/DHCP interoperability, discovery, RSSI,
latency and resource watermarks are not claimed from fake tests or successful
compilation. Those are checked after runtime/entity wiring in Tasks6/9/13.
No board flash, network mode switch, arm, motion or save command was issued.
