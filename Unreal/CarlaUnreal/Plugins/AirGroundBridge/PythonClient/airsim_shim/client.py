"""MultirotorClient — the classic AirSim multirotor API over ProjectAirSim.

Existing AirSim teaching material is written against `client.takeoffAsync()
.join()`, `client.moveToPositionAsync(...)`, `getMultirotorState()`. This maps
that surface onto the ProjectAirSim client so those scripts run against the
fused CARLA + ProjectAirSim world with the import line changed and little else.

Two deliberate departures from AirSim, both because the underlying stack works
differently and hiding it would mislead:

  * ProjectAirSim loads a *scene* (robots, sensors, physics) at connect time,
    where AirSim inherits whatever the level already contains. So the scene
    name is a constructor argument, and `confirmConnection()` loads it.
  * Camera and sensor names come from that scene config ("DownCamera", "IMU1"),
    not from AirSim's fixed names ("front_center", ""). Passing an empty name
    resolves to the first sensor of the right kind, which keeps most tutorial
    code working; anything else must use the scene's real names.
"""

import math
from pathlib import Path
from typing import Dict, List

from projectairsim import Drone, ProjectAirSimClient, World

from .async_bridge import AsyncBridge, CommandFuture
from .types import (
    BarometerData,
    DrivetrainType,
    GpsData,
    ImageRequest,
    ImageType,
    ImuData,
    IMAGE_TYPE_TO_PAS,
    MagnetometerData,
    MultirotorState,
    Pose,
    Vector3r,
    YawMode,
)

DEFAULT_SCENE = "scene_town10_drone.jsonc"
DEFAULT_SIM_CONFIG = str(Path(__file__).resolve().parents[2] / "SimConfig") + "/"

# Which topic key identifies each sensor kind, read off the scene config rather
# than assumed from the sensor's name.
_SENSOR_TOPIC_BY_KIND = {
    "imu": "imu_kinematics",
    "gps": "gps",
    "barometer": "barometer",
    "magnetometer": "magnetometer",
}


class MultirotorClient:
    def __init__(self, ip: str = "127.0.0.1", port: int = 8990,
                 scene: str = DEFAULT_SCENE, sim_config_path: str = DEFAULT_SIM_CONFIG,
                 delay_after_load_sec: int = 2):
        self._client = ProjectAirSimClient(address=ip, port_services=port)
        self._scene = scene
        self._sim_config_path = sim_config_path
        self._delay_after_load_sec = delay_after_load_sec
        self._bridge = AsyncBridge()
        self._world: World = None
        self._drones: Dict[str, Drone] = {}

    # ------------------------------------------------------------- lifecycle

    def confirmConnection(self) -> None:
        """Connect and load the scene. AirSim only prints; here it also binds."""
        self._bridge.call(self._client.connect)
        self._load_scene()

    def _load_scene(self) -> None:
        self._world = self._bridge.call(
            World, self._client, self._scene,
            delay_after_load_sec=self._delay_after_load_sec,
            sim_config_path=self._sim_config_path,
        )
        config = self._bridge.call(self._world.get_configuration) or {}
        names = [a.get("name") for a in config.get("actors", [])
                 if a.get("type") == "robot" and a.get("name")]
        if not names:
            raise RuntimeError(
                f"scene '{self._scene}' declares no robot actors; "
                f"cannot resolve a vehicle to command"
            )
        self._drones = {
            name: self._bridge.call(Drone, self._client, self._world, name)
            for name in names
        }

    def reset(self) -> None:
        """Reload the scene, which returns every vehicle to its spawn state."""
        self._load_scene()

    def ping(self) -> bool:
        return self._world is not None

    def listVehicles(self) -> List[str]:
        return list(self._drones)

    def close(self) -> None:
        try:
            self._bridge.call(self._client.disconnect)
        finally:
            self._bridge.close()

    def __enter__(self) -> "MultirotorClient":
        self.confirmConnection()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    # --------------------------------------------------------- vehicle/sensor

    def _drone(self, vehicle_name: str = "") -> Drone:
        if not self._drones:
            raise RuntimeError("not connected — call confirmConnection() first")
        if not vehicle_name:
            return next(iter(self._drones.values()))
        try:
            return self._drones[vehicle_name]
        except KeyError:
            raise ValueError(
                f"no vehicle '{vehicle_name}' in scene '{self._scene}'; "
                f"available: {', '.join(self._drones)}"
            ) from None

    def _sensor_name(self, drone: Drone, kind: str, requested: str) -> str:
        """Resolve AirSim's empty sensor name to this scene's sensor of that kind."""
        if requested:
            return requested
        topic_key = _SENSOR_TOPIC_BY_KIND[kind]
        for name, topics in (getattr(drone, "sensors", {}) or {}).items():
            if topic_key in topics:
                return name
        raise ValueError(
            f"scene '{self._scene}' declares no {kind} sensor; "
            f"available sensors: {', '.join(getattr(drone, 'sensors', {}) or {})}"
        )

    # ------------------------------------------------------------- API control

    def enableApiControl(self, is_enabled: bool = True, vehicle_name: str = "") -> bool:
        drone = self._drone(vehicle_name)
        fn = drone.enable_api_control if is_enabled else drone.disable_api_control
        return self._bridge.call(fn)

    def isApiControlEnabled(self, vehicle_name: str = "") -> bool:
        return self._bridge.call(self._drone(vehicle_name).is_api_control_enabled)

    def armDisarm(self, arm: bool, vehicle_name: str = "") -> bool:
        drone = self._drone(vehicle_name)
        return self._bridge.call(drone.arm if arm else drone.disarm)

    def cancelLastTask(self, vehicle_name: str = "") -> bool:
        return self._bridge.call(self._drone(vehicle_name).cancel_last_task)

    # ------------------------------------------------------------------ motion

    def takeoffAsync(self, timeout_sec: float = 20, vehicle_name: str = "") -> CommandFuture:
        return self._bridge.submit(self._drone(vehicle_name).takeoff_async,
                                   timeout_sec=timeout_sec)

    def landAsync(self, timeout_sec: float = 3e38, vehicle_name: str = "") -> CommandFuture:
        # AirSim's default is unbounded and descent here runs about 0.2 m/s, so
        # a "sensible" 60 s default would silently abandon the landing partway
        # down and report success. Callers who want a bound pass one.
        return self._bridge.submit(self._drone(vehicle_name).land_async,
                                   timeout_sec=timeout_sec)

    def goHomeAsync(self, timeout_sec: float = 60, velocity: float = 0.5,
                    vehicle_name: str = "") -> CommandFuture:
        return self._bridge.submit(self._drone(vehicle_name).go_home_async,
                                   timeout_sec=timeout_sec, velocity=velocity)

    def hoverAsync(self, vehicle_name: str = "") -> CommandFuture:
        return self._bridge.submit(self._drone(vehicle_name).hover_async)

    def moveToPositionAsync(self, x: float, y: float, z: float, velocity: float,
                            timeout_sec: float = 3e38,
                            drivetrain: int = DrivetrainType.MaxDegreeOfFreedom,
                            yaw_mode: YawMode = None, lookahead: float = -1,
                            adaptive_lookahead: float = 1,
                            vehicle_name: str = "") -> CommandFuture:
        yaw_mode = yaw_mode or YawMode()
        return self._bridge.submit(
            self._drone(vehicle_name).move_to_position_async,
            north=x, east=y, down=z, velocity=velocity, timeout_sec=timeout_sec,
            yaw_control_mode=int(drivetrain), yaw_is_rate=yaw_mode.is_rate,
            yaw=yaw_mode.radians(), lookahead=lookahead,
            adaptive_lookahead=adaptive_lookahead,
        )

    def moveToZAsync(self, z: float, velocity: float, timeout_sec: float = 3e38,
                     yaw_mode: YawMode = None, lookahead: float = -1,
                     adaptive_lookahead: float = 1,
                     vehicle_name: str = "") -> CommandFuture:
        """Change altitude, holding the current north/east position."""
        drone = self._drone(vehicle_name)
        pose = Pose.from_pas(self._bridge.call(drone.get_ground_truth_pose))
        return self.moveToPositionAsync(
            pose.position.x_val, pose.position.y_val, z, velocity,
            timeout_sec=timeout_sec, yaw_mode=yaw_mode, lookahead=lookahead,
            adaptive_lookahead=adaptive_lookahead, vehicle_name=vehicle_name,
        )

    def moveByVelocityAsync(self, vx: float, vy: float, vz: float, duration: float,
                            drivetrain: int = DrivetrainType.MaxDegreeOfFreedom,
                            yaw_mode: YawMode = None,
                            vehicle_name: str = "") -> CommandFuture:
        yaw_mode = yaw_mode or YawMode()
        return self._bridge.submit(
            self._drone(vehicle_name).move_by_velocity_async,
            v_north=vx, v_east=vy, v_down=vz, duration=duration,
            yaw_control_mode=int(drivetrain), yaw_is_rate=yaw_mode.is_rate,
            yaw=yaw_mode.radians(),
        )

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float,
                             drivetrain: int = DrivetrainType.MaxDegreeOfFreedom,
                             yaw_mode: YawMode = None,
                             vehicle_name: str = "") -> CommandFuture:
        yaw_mode = yaw_mode or YawMode()
        return self._bridge.submit(
            self._drone(vehicle_name).move_by_velocity_z_async,
            v_north=vx, v_east=vy, z=z, duration=duration,
            yaw_control_mode=int(drivetrain), yaw_is_rate=yaw_mode.is_rate,
            yaw=yaw_mode.radians(),
        )

    def moveOnPathAsync(self, path: List[Vector3r], velocity: float,
                        timeout_sec: float = 3e38,
                        drivetrain: int = DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode: YawMode = None, lookahead: float = -1,
                        adaptive_lookahead: float = 1,
                        vehicle_name: str = "") -> CommandFuture:
        yaw_mode = yaw_mode or YawMode()
        # AirSim passes Vector3r objects; ProjectAirSim wants [n, e, d] lists.
        points = [[p.x_val, p.y_val, p.z_val] if isinstance(p, Vector3r) else list(p)
                  for p in path]
        return self._bridge.submit(
            self._drone(vehicle_name).move_on_path_async,
            path=points, velocity=velocity, timeout_sec=timeout_sec,
            yaw_control_mode=int(drivetrain), yaw_is_rate=yaw_mode.is_rate,
            yaw=yaw_mode.radians(), lookahead=lookahead,
            adaptive_lookahead=adaptive_lookahead,
        )

    def rotateToYawAsync(self, yaw: float, timeout_sec: float = 3e38,
                         margin: float = 5, vehicle_name: str = "") -> CommandFuture:
        """Yaw is in degrees, as in AirSim; ProjectAirSim is told radians."""
        return self._bridge.submit(
            self._drone(vehicle_name).rotate_to_yaw_async,
            yaw=math.radians(yaw), timeout_sec=timeout_sec,
            margin=math.radians(margin),
        )

    def rotateByYawRateAsync(self, yaw_rate: float, duration: float,
                             vehicle_name: str = "") -> CommandFuture:
        """Rate is in degrees/second, as in AirSim."""
        return self._bridge.submit(
            self._drone(vehicle_name).rotate_by_yaw_rate_async,
            yaw_rate=math.radians(yaw_rate), duration=duration,
        )

    # ------------------------------------------------------------------- state

    def getMultirotorState(self, vehicle_name: str = "") -> MultirotorState:
        drone = self._drone(vehicle_name)
        return MultirotorState(
            kinematics=self._bridge.call(drone.get_ground_truth_kinematics),
            geo=self._bridge.call(drone.get_ground_truth_geo_location),
            landed=self._bridge.call(drone.get_landed_state),
            can_arm=self._bridge.call(drone.can_arm),
        )

    def simGetVehiclePose(self, vehicle_name: str = "") -> Pose:
        return Pose.from_pas(
            self._bridge.call(self._drone(vehicle_name).get_ground_truth_pose))

    def simSetVehiclePose(self, pose: Pose, ignore_collision: bool = True,
                          vehicle_name: str = "") -> bool:
        # AirSim's ignore_collision has no ProjectAirSim equivalent; the pose is
        # applied either way, so a caller passing False is told, not ignored.
        if not ignore_collision:
            raise NotImplementedError(
                "ProjectAirSim always teleports regardless of collision; "
                "call with ignore_collision=True to acknowledge that"
            )
        return self._bridge.call(self._drone(vehicle_name).set_pose, pose.to_pas())

    def simGetGroundTruthKinematics(self, vehicle_name: str = "") -> Dict:
        return self._bridge.call(
            self._drone(vehicle_name).get_ground_truth_kinematics)

    # ----------------------------------------------------------------- sensors

    def getImuData(self, imu_name: str = "", vehicle_name: str = "") -> ImuData:
        drone = self._drone(vehicle_name)
        name = self._sensor_name(drone, "imu", imu_name)
        return ImuData(self._bridge.call(drone.get_imu_data, name))

    def getGpsData(self, gps_name: str = "", vehicle_name: str = "") -> GpsData:
        drone = self._drone(vehicle_name)
        name = self._sensor_name(drone, "gps", gps_name)
        return GpsData(self._bridge.call(drone.get_gps_data, name))

    def getBarometerData(self, barometer_name: str = "",
                         vehicle_name: str = "") -> BarometerData:
        drone = self._drone(vehicle_name)
        name = self._sensor_name(drone, "barometer", barometer_name)
        return BarometerData(self._bridge.call(drone.get_barometer_data, name))

    def getMagnetometerData(self, magnetometer_name: str = "",
                            vehicle_name: str = "") -> MagnetometerData:
        drone = self._drone(vehicle_name)
        name = self._sensor_name(drone, "magnetometer", magnetometer_name)
        return MagnetometerData(self._bridge.call(drone.get_magnetometer_data, name))

    def simGetImages(self, requests: List[ImageRequest], vehicle_name: str = "") -> Dict:
        """Capture camera images.

        NOT exercised by the headless test suite: scene capture needs a real
        RHI, and this machine runs the sim with -nullrhi. Treat it as untested
        until it has run on the GPU box.

        Image types are translated because the two enums disagree on numbering
        (see types.IMAGE_TYPE_TO_PAS); an unsupported type raises rather than
        silently returning a different image than was asked for.
        """
        if not requests:
            return {}
        cameras = {r.camera_name for r in requests}
        if len(cameras) != 1:
            raise ValueError(
                "ProjectAirSim fetches images per camera; issue one call per "
                f"camera (got {sorted(cameras)})"
            )
        type_ids = []
        for req in requests:
            try:
                type_ids.append(IMAGE_TYPE_TO_PAS[ImageType(req.image_type)])
            except (KeyError, ValueError):
                raise ValueError(
                    f"image type {req.image_type} has no ProjectAirSim "
                    f"equivalent; supported: "
                    f"{[t.name for t in IMAGE_TYPE_TO_PAS]}"
                ) from None
        return self._bridge.call(self._drone(vehicle_name).get_images,
                                 cameras.pop(), type_ids)
