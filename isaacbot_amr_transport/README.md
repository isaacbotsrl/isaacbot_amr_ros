# isaacbot_amr_transport

The isaacbot's transport layer: a hard fork of
[ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers) v1.2.0,
relayouted into this workspace's Clean Architecture convention.

| Package | Role |
|---------|------|
| [`isaacbot_amr_asio_cmake_module`](isaacbot_amr_asio_cmake_module/) | `find_package(ASIO)` support. No code. |
| [`isaacbot_amr_io_context`](isaacbot_amr_io_context/) | Shared ports, byte bridges and the ASIO thread pool. No ROS dependency. |
| [`isaacbot_amr_udp_driver`](isaacbot_amr_udp_driver/) | UDP <-> `udp_read` / `udp_write`. |
| [`isaacbot_amr_modbus_driver`](isaacbot_amr_modbus_driver/) | Synchronous Modbus TCP client. **Library only - no node, no topics.** |

Upstream's serial driver was forked as `isaacbot_amr_serial_driver` and **removed in 2026-10**: its
only consumer, the ELRS receiver, moved onto the RUTX11 router, which forwards the UART over
UDP to `isaacbot_amr_udp_driver` (see `isaacbot_amr_crsf_teleop`). Recover it from git history if a UART
consumer ever returns.

Each package's README lists what changed from upstream. Several real defects were fixed
along the way - two use-after-frees in the async send paths, a receiver that stopped on a
zero-length datagram, and a `-O0` flag that leaked onto every consumer.

`isaacbot_amr_modbus_driver` is the odd one out and deliberately so: it exports libraries rather
than a bridge node, because Modbus TCP is request/response rather than a byte stream, and its
consumer (the ros2_control safety controller) needs in-process calls rather than a topic
round-trip on the e-stop path. Its README explains the reasoning.

## Provenance

`isaacbot_amr_modbus_driver` is **not** part of the transport_drivers fork described below. It was
extracted from `isaacbot_amr_hardware_interface` and builds on `isaacbot_amr_modbus`, a separate vendored
fork of [Mazurel/Modbus](https://github.com/Mazurel/Modbus) (MIT). Everything else here comes
from transport_drivers:

Forked from [ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers)
**v1.2.0** (Apache-2.0; LeoDrive, The Autoware Foundation, Apex.AI, Trimble, TierIV,
Evan Flynn). Relayouted into this workspace's Clean Architecture layout, renamed to the
`isaacbot_amr_` prefix and re-namespaced from `drivers::` to `isaacbot::transport::`.

**Upstream is no longer merged - this is a hard fork.** Upstream has no release for ROS 2
`lyrical`, which is why it was vendored in the first place; port fixes by hand.

Topic names (`udp_read`, `udp_write`) and parameter names (`ip`, `port`) are
**unchanged from upstream** (`isaacbot_amr_udp_driver`'s `source_ip` is an addition), so upstream documentation still describes the wire interface.

> The exact upstream commit is not recoverable from this tree - it carried no VCS metadata
> and no vcs manifest entry; every `package.xml` read `1.2.0`. Resolve the sha of the
> `1.2.0` tag and record it here.
