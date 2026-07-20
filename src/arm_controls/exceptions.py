"""Typed public exceptions."""


class ArmControlsError(Exception):
    """Base class for package errors."""


class ConfigurationError(ArmControlsError, ValueError):
    """A configuration or command is invalid."""


class ConnectionUnavailableError(ArmControlsError):
    """A requested hardware connection is not ready."""


class NativeProcessError(ArmControlsError):
    """The owned native process failed."""


class HardwareFaultError(NativeProcessError):
    """The native runtime stopped because hardware reported an actionable fault."""


class ProtocolError(ArmControlsError):
    """The native process uses an incompatible or malformed protocol."""


class StateTimeoutError(ArmControlsError, TimeoutError):
    """No fresh state arrived before the deadline."""


class StaleStateError(ArmControlsError):
    """A state is too old to use safely."""


class CommandRejectedError(ArmControlsError):
    """The native runtime rejected a command."""


class RoleError(ArmControlsError):
    """An operation is unavailable for this arm role."""


class AlignmentError(ArmControlsError):
    """Follower alignment could not complete safely."""
