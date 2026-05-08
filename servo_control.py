#!/usr/bin/env python3
"""Object-oriented wrappers for gripper servo control.

Usage example:
    from servo_control import Gripper

    gripper = Gripper(servo_no=2, speed=120)
    gripper.open()
    gripper.close()
"""

from __future__ import annotations

from typing import Dict

from uservice import service


class Servo:
    """Base servo interface backed by the robot MQTT service."""

    PRESETS: Dict[str, int] = {}
    DEFAULT_SPEED = 10

    def __init__(self, servo_no: int = 2, speed: int | None = None) -> None:
        self.servo_no = int(servo_no)
        use_speed = self.DEFAULT_SPEED if speed is None else speed
        self.speed = max(0, int(use_speed))

    @staticmethod
    def _clamp(value: int, low: int, high: int) -> int:
        return max(low, min(high, int(value)))

    def set_speed(self, speed: int) -> None:
        """Update default speed used by movement methods."""
        self.speed = max(0, int(speed))

    def set_servo(self, servo_no: int) -> None:
        """Change which servo this instance controls."""
        self.servo_no = int(servo_no)

    def set_position(self, value: int, speed: int | None = None) -> bool:
        """Set servo to an absolute position."""
        use_speed = self.speed if speed is None else max(0, int(speed))
        target = self._clamp(value, -120000, 30000)
        return service.send(
            "robobot/cmd/T0", f"servo {self.servo_no} {target} {use_speed}"
        )

    def get_preset_value(self, name: str) -> int:
        """Get a named preset value for this class."""
        if name not in self.PRESETS:
            raise KeyError(
                f"Preset '{name}' not configured for {self.__class__.__name__}"
            )
        return int(self.PRESETS[name])

    def set_preset_value(self, name: str, value: int) -> None:
        """Set a named preset value for this class."""
        self.PRESETS[name] = self._clamp(value, -120000, 30000)


class Gripper(Servo):
    """Stateful gripper controller bound to one servo and default speed."""

    DEFAULT_SPEED = 400
    PRESETS: Dict[str, int] = {
        "open": -440,
        "close_ball": 480,
        "close_luggage": 400,
    }

    def __init__(self, servo_no: int = 2, speed: int | None = None) -> None:
        super().__init__(servo_no=servo_no, speed=speed)

    def get_preset(self) -> tuple[int, int]:
        """Return (open, close) presets for this servo."""
        return self.get_preset_value("open"), self.get_preset_value("close")

    def set_preset(self, *, open_value: int, close_value: int) -> None:
        """Set (open, close) presets for this servo."""
        self.set_preset_value("open", open_value)
        self.set_preset_value("close", close_value)

    def open(self, speed: int | None = None) -> bool:
        """Move to open preset using instance defaults unless overridden."""
        return self.set_position(self.get_preset_value("open"), speed=speed)

    def close_ball(self, speed: int | None = None) -> bool:
        """Move to close ball preset using instance defaults unless overridden."""
        return self.set_position(self.get_preset_value("close_ball"), speed=speed)

    def close_luggage(self, speed: int | None = None) -> bool:
        """Move to close luggage preset using instance defaults unless overridden."""
        return self.set_position(self.get_preset_value("close_luggage"), speed=speed)


class Hand(Gripper):
    """Convenience gripper preset for hand servo (default servo 3)."""

    DEFAULT_SPEED = 10
    PRESETS: Dict[str, int] = {
        "opened": 600,
        "closed": -80,
    }

    def __init__(self, servo_no: int = 3, speed: int | None = None) -> None:
        super().__init__(servo_no=servo_no, speed=speed)

    def get_preset(self) -> tuple[int, int]:
        """Return (opened, closed) presets for this hand servo."""
        return self.get_preset_value("opened"), self.get_preset_value("closed")

    def set_preset(self, *, opened_value: int, closed_value: int) -> None:
        """Set (opened, closed) presets for this hand servo."""
        self.set_preset_value("opened", opened_value)
        self.set_preset_value("closed", closed_value)

    def open(self, speed: int | None = None) -> bool:
        """Move to opened preset using instance defaults unless overridden."""
        return self.opened(speed=speed)

    def opened(self, speed: int | None = None) -> bool:
        """Move to opened preset using instance defaults unless overridden."""
        return self.set_position(self.get_preset_value("opened"), speed=speed)

    def close(self, speed: int | None = None) -> bool:
        """Move to closed preset using instance defaults unless overridden."""
        return self.closed(speed=speed)

    def closed(self, speed: int | None = None) -> bool:
        """Move to closed preset using instance defaults unless overridden."""
        return self.set_position(self.get_preset_value("closed"), speed=speed)


class MainServo(Gripper):
    """Convenience gripper preset for main servo channel (default servo 1)."""

    DEFAULT_SPEED = 20
    PRESETS: Dict[str, int] = {
        "up": -900,
        "middle_up": -400,
        "middle": -200,
        "semi_down": -150,
        "down": -90,
    }

    def __init__(self, servo_no: int = 1, speed: int | None = None) -> None:
        super().__init__(servo_no=servo_no, speed=speed)

    def goto_up(self, speed: int | None = None) -> bool:
        """Move main servo to the configured UP position."""
        return self.set_position(self.get_preset_value("up"), speed=speed)

    def goto_middle_up(self, speed: int | None = None) -> bool:
        """Move main servo to the configured MIDDLE-UP position."""
        return self.set_position(self.get_preset_value("middle_up"), speed=speed)

    def goto_middle(self, speed: int | None = None) -> bool:
        """Move main servo to the configured MIDDLE position."""
        return self.set_position(self.get_preset_value("middle"), speed=speed)

    def goto_semi_down(self, speed: int | None = None) -> bool:
        """Move main servo to the configured SEMI-DOWN position."""
        return self.set_position(self.get_preset_value("semi_down"), speed=speed)

    def goto_down(self, speed: int | None = None) -> bool:
        """Move main servo to the configured DOWN position."""
        return self.set_position(self.get_preset_value("down"), speed=speed)
