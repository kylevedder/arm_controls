"""Per-servo-family drivers and the servo model registry.

Each servo family and bus type (``dm_can``, ``encos_can``, ``dxl_serial``, ...)
exposes the same function API. This module provides the registry keyed by the
canonical servo model string.

Naming policy: the registry keys
are the exact ``servo_model`` strings used in the arm/effector config JSONs
under ``arm_controls/models/``. No translation layer exists — read the
JSON, look the string up here. A new servo model must use the same canonical
string in the model JSON and in this table.

Driver module API (uniform across families):
    PORT_TYPE: str             "can", "serial", or "ethernet" — which bus
                               session the driver needs (see ``buses.open_bus``)
    WHOLE_ARM_ZERO: bool       False: the family zeroes one servo at a time and
                               defines ``set_zero(bus, servo_id)``. True: the
                               family (whole-arm Ethernet controllers, e.g.
                               Trossen iNerve) zeroes every joint in one
                               controller transaction and defines
                               ``set_zero_whole_arm(bus)`` instead.
    set_zero* functions        return None on success or an error string

Adding a servo family: create ``<family>_<bus>.py`` with that API, then register
every canonical model string of the family below. Nothing else changes — the zeroing tool
dispatches through this table.
"""

from __future__ import annotations

import types

from arm_controls.servos import dm_can, dxl_serial, encos_can, ft_serial, trossen_eth

# Servo model string -> driver module, or None for read-only encoders whose
# zero is fixed in hardware (reported as skipped by the zeroing tool, never
# as a failure).
SERVO_ZERO_DRIVERS: dict[str, types.ModuleType | None] = {
    "DM J4310": dm_can,
    "DM J4340": dm_can,
    "DM J3507": dm_can,
    "DM S3519": dm_can,
    "Encos EC-A4310-P2-36": encos_can,
    "Encos EC-A6013-H20-100": encos_can,
    "Encos EC-A4315-P2-36": encos_can,
    "Encos EC-A6408-P2-25": encos_can,
    "Encos EC-A10020-P2-24": encos_can,
    "Dynamixel XM430-W210": dxl_serial,
    "Dynamixel XH430-W210": dxl_serial,
    "Dynamixel XC330-T288": dxl_serial,
    "Dynamixel XH430-W350": dxl_serial,
    # FeeTech SMS/STS serial (SO-ARM100/101; also covers Hiwonder HX-30HM/HX-10HM).
    "FeeTech STS3215": ft_serial,
    # Whole-arm Ethernet controller joints (zeroed via one EEPROM write).
    "Trossen WXAI Joint": trossen_eth,
    # Read-only encoders: no motor, zero reference fixed in hardware.
    "ARX Remote Encoder": None,
    "CAN Passive Encoder": None,
}


def zero_driver(servo_model: str) -> types.ModuleType | None:
    """Driver module for ``servo_model``; None means read-only (nothing to zero)."""
    if servo_model not in SERVO_ZERO_DRIVERS:
        raise SystemExit(
            f"servo model {servo_model!r} is not in the servo registry "
            "(arm_controls/servos): add the canonical model string "
            "and its driver module to SERVO_ZERO_DRIVERS"
        )
    return SERVO_ZERO_DRIVERS[servo_model]
