"""AirSim-compatible value types.

Field names follow the classic `airsim` package (`x_val`, `kinematics_estimated`,
...) so tutorial code reads unchanged. Every value is built from a payload that
ProjectAirSim really returns — see tools/diag_pas_api_shapes.py for the dumps
these mappings were written against.

Where AirSim exposes something this stack cannot source, the attribute is left
out rather than filled with a plausible zero. An AttributeError names the gap;
a fabricated `has_collided = False` would answer a safety question wrongly.
"""

import math
from enum import IntEnum
from typing import Dict, List


class Vector3r:
    def __init__(self, x_val: float = 0.0, y_val: float = 0.0, z_val: float = 0.0):
        self.x_val = float(x_val)
        self.y_val = float(y_val)
        self.z_val = float(z_val)

    @staticmethod
    def from_pas(vec: Dict) -> "Vector3r":
        return Vector3r(vec.get("x", 0.0), vec.get("y", 0.0), vec.get("z", 0.0))

    def to_pas(self) -> Dict:
        return {"x": self.x_val, "y": self.y_val, "z": self.z_val}

    def __repr__(self) -> str:
        return f"Vector3r({self.x_val:.3f}, {self.y_val:.3f}, {self.z_val:.3f})"


class Quaternionr:
    def __init__(self, x_val: float = 0.0, y_val: float = 0.0,
                 z_val: float = 0.0, w_val: float = 1.0):
        self.x_val = float(x_val)
        self.y_val = float(y_val)
        self.z_val = float(z_val)
        self.w_val = float(w_val)

    @staticmethod
    def from_pas(quat: Dict) -> "Quaternionr":
        return Quaternionr(quat.get("x", 0.0), quat.get("y", 0.0),
                           quat.get("z", 0.0), quat.get("w", 1.0))

    def to_pas(self) -> Dict:
        return {"x": self.x_val, "y": self.y_val, "z": self.z_val, "w": self.w_val}

    def __repr__(self) -> str:
        return (f"Quaternionr({self.x_val:.3f}, {self.y_val:.3f}, "
                f"{self.z_val:.3f}, {self.w_val:.3f})")


class Pose:
    def __init__(self, position_val: Vector3r = None, orientation_val: Quaternionr = None):
        self.position = position_val if position_val is not None else Vector3r()
        self.orientation = orientation_val if orientation_val is not None else Quaternionr()

    @staticmethod
    def from_pas(transform: Dict) -> "Pose":
        # PAS calls it "translation"/"rotation"; AirSim calls it
        # "position"/"orientation". Reading the wrong key here once produced a
        # telemetry log full of zeros for a drone that was flying normally.
        return Pose(Vector3r.from_pas(transform.get("translation", {})),
                    Quaternionr.from_pas(transform.get("rotation", {})))

    def to_pas(self) -> Dict:
        return {"translation": self.position.to_pas(),
                "rotation": self.orientation.to_pas()}

    def __repr__(self) -> str:
        return f"Pose({self.position}, {self.orientation})"


class GeoPoint:
    def __init__(self, latitude: float = 0.0, longitude: float = 0.0, altitude: float = 0.0):
        self.latitude = float(latitude)
        self.longitude = float(longitude)
        self.altitude = float(altitude)

    @staticmethod
    def from_pas(geo: Dict) -> "GeoPoint":
        return GeoPoint(geo.get("latitude", 0.0), geo.get("longitude", 0.0),
                        geo.get("altitude", 0.0))

    def __repr__(self) -> str:
        return (f"GeoPoint(lat={self.latitude:.7f}, lon={self.longitude:.7f}, "
                f"alt={self.altitude:.2f})")


class LandedState(IntEnum):
    Landed = 0
    Flying = 1


class DrivetrainType(IntEnum):
    # Same numbering as ProjectAirSim's YawControlMode, so these pass straight
    # through. Verified against projectairsim.drone.YawControlMode.
    MaxDegreeOfFreedom = 0
    ForwardOnly = 1


class ImageType(IntEnum):
    """AirSim's numbering, which is NOT ProjectAirSim's.

    AirSim puts DepthVis at 3 and Segmentation at 5; ProjectAirSim puts
    SEGMENTATION at 3 and DEPTH_VIS at 4. Passing an AirSim value straight
    through would silently return a different image than the caller asked for,
    so requests are translated by IMAGE_TYPE_TO_PAS below.
    """

    Scene = 0
    DepthPlanar = 1
    DepthPerspective = 2
    DepthVis = 3
    DisparityNormalized = 4
    Segmentation = 5
    SurfaceNormals = 6
    Infrared = 7


# AirSim ImageType -> ProjectAirSim ImageType. Absent keys are unsupported and
# raise, rather than quietly resolving to whatever shares the number.
IMAGE_TYPE_TO_PAS = {
    ImageType.Scene: 0,               # SCENE
    ImageType.DepthPlanar: 1,         # DEPTH_PLANAR
    ImageType.DepthPerspective: 2,    # DEPTH_PERSPECTIVE
    ImageType.DepthVis: 4,            # DEPTH_VIS
    ImageType.DisparityNormalized: 5, # DISPARITY_NORMALIZED
    ImageType.Segmentation: 3,        # SEGMENTATION
    ImageType.SurfaceNormals: 6,      # SURFACE_NORMALS
}


class YawMode:
    """AirSim expresses yaw in DEGREES; ProjectAirSim expresses it in radians."""

    def __init__(self, is_rate: bool = True, yaw_or_rate: float = 0.0):
        self.is_rate = is_rate
        self.yaw_or_rate = float(yaw_or_rate)

    def radians(self) -> float:
        return math.radians(self.yaw_or_rate)


class KinematicsState:
    def __init__(self, kinematics: Dict):
        pose = kinematics.get("pose", {})
        twist = kinematics.get("twist", {})
        accels = kinematics.get("accels", {})
        self.position = Vector3r.from_pas(pose.get("position", {}))
        self.orientation = Quaternionr.from_pas(pose.get("orientation", {}))
        self.linear_velocity = Vector3r.from_pas(twist.get("linear", {}))
        self.angular_velocity = Vector3r.from_pas(twist.get("angular", {}))
        self.linear_acceleration = Vector3r.from_pas(accels.get("linear", {}))
        self.angular_acceleration = Vector3r.from_pas(accels.get("angular", {}))

    def __repr__(self) -> str:
        return f"KinematicsState(pos={self.position}, vel={self.linear_velocity})"


class MultirotorState:
    """AirSim's MultirotorState, limited to fields this stack can source.

    Deliberately absent: `collision` and `rc_data`. ProjectAirSim publishes
    collision on a topic rather than a getter, and there is no RC link in SITL.
    Code touching them gets an AttributeError naming the gap instead of a
    confident wrong answer.
    """

    def __init__(self, kinematics: Dict, geo: Dict, landed: int, can_arm: bool):
        self.kinematics_estimated = KinematicsState(kinematics)
        self.gps_location = GeoPoint.from_pas(geo)
        self.timestamp = int(kinematics.get("time_stamp", 0))
        self.landed_state = LandedState(int(landed))
        self.can_arm = bool(can_arm)

    def __repr__(self) -> str:
        return (f"MultirotorState({self.kinematics_estimated}, "
                f"{self.landed_state.name})")


class ImuData:
    def __init__(self, data: Dict):
        self.time_stamp = int(data.get("time_stamp", 0))
        self.orientation = Quaternionr.from_pas(data.get("orientation", {}))
        self.angular_velocity = Vector3r.from_pas(data.get("angular_velocity", {}))
        self.linear_acceleration = Vector3r.from_pas(
            data.get("linear_acceleration", {}))


class GpsData:
    def __init__(self, data: Dict):
        self.time_stamp = int(data.get("time_stamp", 0))
        self.gnss = GeoPoint(data.get("latitude", 0.0), data.get("longitude", 0.0),
                             data.get("altitude", 0.0))
        self.velocity = Vector3r.from_pas(data.get("velocity", {}))
        self.fix_type = int(data.get("fix_type", 0))
        self.eph = float(data.get("eph", 0.0))
        self.epv = float(data.get("epv", 0.0))


class BarometerData:
    def __init__(self, data: Dict):
        self.time_stamp = int(data.get("time_stamp", 0))
        self.altitude = float(data.get("altitude", 0.0))
        self.pressure = float(data.get("pressure", 0.0))
        self.qnh = float(data.get("qnh", 0.0))


class MagnetometerData:
    def __init__(self, data: Dict):
        self.time_stamp = int(data.get("time_stamp", 0))
        self.magnetic_field_body = Vector3r.from_pas(
            data.get("magnetic_field_body", {}))
        self.magnetic_field_covariance: List[float] = list(
            data.get("magnetic_field_covariance", []))


class ImageRequest:
    def __init__(self, camera_name: str, image_type: int = ImageType.Scene,
                 pixels_as_float: bool = False, compress: bool = True):
        self.camera_name = camera_name
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress
