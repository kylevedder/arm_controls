# arm-controls Rust API

This crate is a native Rust client for the existing arm_controls node. It exposes
`ArmSession`, `ArmConfig`, `FollowerArm`, `PositionCommand`, `ArmState` and strict
capabilities. The C++ node exclusively owns CAN, motor control, native gravity,
feedback protection and disable behavior. No Python interpreter or package
lookup is used.

Current supported configuration is a strict YAM follower with native gravity,
seven-joint state including the gripper, and explicit native/model/instance/URDF
paths. `zmq_library` can select an exact attested native library (VR requires it).
A unique session isolates topics. Connect requires protocol 1.1, strict/direct/
gravity capabilities, READY and feedback. HOLD requires a correlated successful
lifecycle ACK. Close requires shutdown ACK plus successful child exit; failure
still terminates and reaps the child. Session failure/close attempts every arm.
Feedback reports original host receive clocks, mode and gap/hold evidence.
Both native output streams are also retained in per-arm logs. `log_directory`
overrides `ARM_CONTROLS_LOG_DIR`, then `$HOME/arm_controls-data/logs`; an existing
session log is rotated to a unique file before native startup.

Production connect requires Linux and an available up/unknown SocketCAN interface;
the node remains the sole device owner. Tests substitute only a private host
validation step, then execute the actual worker, child process and ZMQ transport
against a no-CAN fixture. The fixture is compiled only for tests. Install native
ZeroMQ to run the tests. No fixture or test bypass is exposed in the public API.

This new crate is separate from any inherited native overlay in the worktree.
Review and pin the native source/build artifact separately; this client cannot
prove arbitrary native binaries have the strict overlay.

Coherent observation telemetry uses a state-only 496-byte message: the unchanged
252-byte raw state, a native-endian u32 version (1), and ten 24-byte records
(f32 position, f32 velocity, u64 CLOCK_MONOTONIC receive ns, u64 motor sample ID).
Only complete DM/ENCOS q/dq reports advance a motor ID. Re-reading a cache,
partial ENCOS reports, and register ACKs do not create samples. Each record is
copied under the existing native receive lock. ArmState.feedback_samples reports
seven optional snapshots in calibrated radians and normalized gripper opening;
velocity uses the same coordinate per second. Legacy raw state/velocity fields
retain their prior semantics. Unsupported/malformed optional samples are absent.
Receive time is host parse time, not the motor's sensing instant.
On an inverted joint, coherent velocity therefore has the sign of calibrated
position's derivative and can differ from the unchanged legacy raw velocity.
Production native and Rust receive clocks both use Linux CLOCK_MONOTONIC;
the macOS Rust clock is used only by the no-hardware test fixture.

Paired native state subscribers and Python readers accept both the legacy
252-byte state and the 496-byte state, preserving the raw prefix even when the
optional telemetry version is unknown. Command messages remain exactly 252 bytes.

`command_with_receipt` returns the worker's monotonic time immediately after a
successful native IPC send; it proves submission, never actuator execution.
`lifecycle_sequence` and `lifecycle_received_mono_ns` expose changes even when no
new state payload arrives. A sequence jump means intermediate mode/hold changes
were not observed. The native strict safety overlay is unchanged by telemetry.
