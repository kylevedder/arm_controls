# arm_controls

`arm_controls` is a native C++ robot-control library. Each arm
runs one `arm_controls_node` process that communicates with the main
Python process via ZeroMQ.

| Arm | Follower effector | Leader effector |
| --- | --- | --- |
| `Yam` | `E_Yam` | `E_Yam_Handle` |
| `ARX_L5` | `E_ARX` | `E_ARX` |
| `ARX_X5` | `E_ARX` | `E_ARX` |
| `SO101` | `E_SO101` | `E_SO101` |
| `Trossen_wai_ctrl` | `E_Trossen_ctrl` | `E_Trossen_ctrl` |
| `ARX_ENC` | — | `E_ARX_ENC` |
| `FR3` | `Robotiq` | — |

```python
from arm_controls import ArmConfig, ArmSession, PositionCommand, SocketCanConnection

with ArmSession() as session:
    follower = session.add_follower(
        ArmConfig(
            "right_follower",
            "Yam",
            SocketCanConnection("can_follower_r"),
            effector_model="E_Yam",
            follower_gravity_compensation=True,
        )
    )
    session.connect()
    follower.command(PositionCommand([0, 0, 0, 0, 0, 0], 1.0))
```

FR3 uses the same `ArmSession`, `FollowerArm`, and `PositionCommand` API. Its
connection selects one of the two real Robotiq Modbus transports:

```python
from arm_controls import FR3Connection, RobotiqConnection

config = ArmConfig(
    "follower",
    "FR3",
    FR3Connection("192.168.1.10"),
    effector_model="Robotiq",
    effector_connection=RobotiqConnection.rtu("/dev/serial/by-id/usb-robotiq"),
    # Or: RobotiqConnection.tcp("192.168.1.11", port=502),
)
```

See [docs/fr3.md](docs/fr3.md) for firmware, networking, controller, and
hardware-validation details.

The Trossen maintenance SDK currently provides wheels for Python 3.11–3.13.
Other arm types remain importable on Python 3.14; requesting a Trossen-specific
operation there reports the missing SDK with an actionable error.

## Building from source

Building requires CMake and a C++17 compiler. The dependency builder pins and
builds Pinocchio, ZeroMQ, cppzmq, Trossen, libfranka 0.21.3, and libmodbus;
the resulting libfranka and libmodbus archives are linked into
`arm_controls_node`. It has been tested on Ubuntu 22.04 and 24.04.

```bash
sudo ./scripts/install_build_deps_ubuntu.sh
./scripts/build_deps.sh
uv build --wheel
```

The wheel is written to `dist/`.

## License

This project is available under the MIT License. See [LICENSE](LICENSE).
