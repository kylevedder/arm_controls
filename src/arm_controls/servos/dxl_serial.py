"""Dynamixel serial servo driver — zeroing seam (not integrated yet).

Registered in the servo registry so the structure is in place for serial
servos, but no Dynamixel hardware has been brought up on this stack yet.
Zeroing an X-series servo is a multi-step EEPROM procedure — force Extended
Position Mode, clear the Homing Offset, read the folded present position,
write the single-shot offset, reboot, and verify. Implement that routine here
with plain pyserial and Protocol 2.0 when integrating the first Dynamixel arm;
the registry, port-type dispatch, and tooling need no changes.
"""

from __future__ import annotations

from arm_controls.servos import buses

PORT_TYPE = buses.PORT_TYPE_SERIAL
WHOLE_ARM_ZERO = False


def set_zero(bus: object, servo_id: int) -> str | None:
    """Set the current position as the zero reference (Homing Offset calibration)."""
    raise NotImplementedError(
        f"Dynamixel zeroing (servo id {servo_id}) is not integrated yet: "
        "add the reset-to-current-position command sequence"
    )
