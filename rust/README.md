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
