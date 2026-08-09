"""AirSim-compatible client for the fused CARLA + ProjectAirSim world.

Written so existing AirSim teaching material runs against this stack:

    import airsim_shim as airsim

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToPositionAsync(10, 0, -20, 5).join()
    print(client.getMultirotorState().kinematics_estimated.position)
    client.landAsync().join()

The package is deliberately NOT named `airsim`: shadowing the real package
would make a machine with both installed behave differently depending on
sys.path order, and that is a bug nobody enjoys finding. Alias it at the import
line instead, where the substitution is visible.

Coverage and the places this departs from AirSim are documented in
docs/airsim-shim.md.
"""

from .async_bridge import CommandFuture
from .client import DEFAULT_SCENE, MultirotorClient
from .types import (
    BarometerData,
    DrivetrainType,
    GeoPoint,
    GpsData,
    ImageRequest,
    ImageType,
    ImuData,
    KinematicsState,
    LandedState,
    MagnetometerData,
    MultirotorState,
    Pose,
    Quaternionr,
    Vector3r,
    YawMode,
)

__all__ = [
    "BarometerData",
    "CommandFuture",
    "DEFAULT_SCENE",
    "DrivetrainType",
    "GeoPoint",
    "GpsData",
    "ImageRequest",
    "ImageType",
    "ImuData",
    "KinematicsState",
    "LandedState",
    "MagnetometerData",
    "MultirotorClient",
    "MultirotorState",
    "Pose",
    "Quaternionr",
    "Vector3r",
    "YawMode",
]
