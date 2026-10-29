# Plan3 Task6: endpoint selection software checkpoint

This is the first part of Task6, not completed WiFi orchestration or live ROS.
The network task remains staged offline; no Plan3 image has been flashed.

`umb_endpoint` keeps socket-independent endpoint selection bounded and testable:

- STA uses the configured address first. A zero address probes the local subnet
  broadcast. Invalid/missing subnet data causes no probe; limited broadcast is
  never used. Broadcast selection requires a validated XRCE reply from a local
  unicast address.
- AP ignores the STA configured address and walks at most eight DHCP leases,
  requiring both an active lease and a currently associated matching MAC.
  Failed candidates advance round-robin, so a phone leasing first cannot starve
  the PC leasing second. Unknown association data fails closed.
- One probe is outstanding, with a 100 ms deadline and 100 ms retry spacing.
  The transport adapter must enforce that timeout; this pure policy cannot
  interrupt a blocked socket/core call. The owner reconciles observations before
  delivering results and sends loss to control before changing active resources.
- Only a successful dedicated XRCE probe may supply `xrce_ok`. Ordinary UDP
  traffic cannot select an endpoint. Nonzero result tokens reject late replies
  after deadline, mode/route/port change, lease expiry or client disconnection.
- A selected endpoint remains fixed until invalidated or explicitly cleared on
  session failure. Membership uses both IP and MAC, rejecting IP reuse.

Seven host tests cover fixed/broadcast selection, a phone-first/PC-second AP,
lease reorder/expiry/disconnect/reuse, timeout and counter wrap, stale results,
wrong-source replies, round-robin retries, malformed subnet/candidate snapshots,
and bounded processing even when snapshot counts exceed the array capacities.
Docker ARM build and coverage suite pass: 343 C tests in 28 executables plus
10 Python contract tests. See ignored `build/plan3-task6-{fw,coverage}.log`.

Remaining Task6 work: connect owner FSM to real adapters and session lifecycle,
provision a missing AP credential through verified maintenance persistence,
publish shared network snapshots, and test teardown before reconfiguration.
Real flash erase/program latency versus the 250 ms control watchdog remains
unresolved; this checkpoint does not enable boot-time flash writes.
