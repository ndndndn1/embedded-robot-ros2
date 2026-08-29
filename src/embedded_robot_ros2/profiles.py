from dataclasses import dataclass

from .models import CommandKind


@dataclass(frozen=True, slots=True)
class RobotProfile:
    product_class: str
    product_name: str
    capabilities: frozenset[CommandKind]
    joints: tuple[str, ...]


PROFILES: dict[str, RobotProfile] = {
    "MH-01": RobotProfile(
        product_class="industrial humanoid",
        product_name="MockHumanoid MH-01",
        capabilities=frozenset({CommandKind.MANIPULATE, CommandKind.PROTECTIVE_STOP}),
        joints=tuple(f"joint_{index:02d}" for index in range(1, 13)),
    ),
    "MM-01": RobotProfile(
        product_class="mobile manipulator",
        product_name="MockMobileManipulator MM-01",
        capabilities=frozenset(CommandKind),
        joints=tuple(f"arm_joint_{index}" for index in range(1, 7)),
    ),
}

