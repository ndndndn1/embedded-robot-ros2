from dataclasses import dataclass

from .models import CommandType, ProductProfile

PRODUCTS = (
    ProductProfile(
        product_id="mock-humanoid-mh-01",
        classification="industrial_humanoid",
        product_name="MockHumanoid",
        model_name="MH-01",
        capabilities=frozenset(CommandType),
        joint_count=12,
        connection_interface="RobotPort REST v1; replace mock with a conforming hardware adapter",
    ),
    ProductProfile(
        product_id="mock-mobile-manipulator-mm-01",
        classification="autonomous_mobile_manipulator",
        product_name="MockMobileManipulator",
        model_name="MM-01",
        capabilities=frozenset(CommandType),
        joint_count=6,
        connection_interface="RobotPort REST v1; replace mock with a conforming hardware adapter",
    ),
)


@dataclass(frozen=True, slots=True)
class RobotProfile:
    product_id: str
    joints: tuple[str, ...]


ROBOTS: dict[str, RobotProfile] = {
    "mh-01-a": RobotProfile(
        product_id="mock-humanoid-mh-01",
        joints=tuple(f"joint_{index:02d}" for index in range(1, 13)),
    ),
    "mm-01-a": RobotProfile(
        product_id="mock-mobile-manipulator-mm-01",
        joints=tuple(f"arm_joint_{index}" for index in range(1, 7)),
    ),
}


PRODUCTS_BY_ID = {product.product_id: product for product in PRODUCTS}
