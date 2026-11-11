// AgGimbal: a CARLA camera that points like a stabilised gimbal.
//
// Console commands (also over the loopback console, -AgExecPort):
//   ag.Gimbal.Attach <carla actor id>   make that camera a gimbal camera
//   ag.Gimbal.Quat <w> <x> <y> <z> earth|vehicle
//                                       point every gimbal camera: the attitude
//                                       quaternion of MAVLink's
//                                       GIMBAL_DEVICE_ATTITUDE_STATUS, as sent
//   ag.Gimbal.Point <pitch> <yaw> <roll> earth|vehicle
//                                       the same in degrees, for a person
//   ag.Gimbal.Point                     print where they point
//   ag.Gimbal.Fov <horizontal degrees>  zoom every gimbal camera
//
// Attitude as MAVLink's gimbal protocol reports it: pitch and roll against the
// horizon (NED), yaw against north ("earth") or against the heading of the
// actor the camera is attached to ("vehicle", q_earth = q_heading * q). A
// quaternion goes through straight down, where pitch/yaw/roll lose the yaw.
// NED to Unreal: (w, x, y, z) -> FQuat(-x, -y, z, w), as ProjectAirSim draws
// its robots (TransformUtils::NedToNeuQuat); Unreal's pitch, yaw and roll are
// then NED's.
#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"

#include "AgGimbalComponent.generated.h"

/// Sits on the camera actor and turns it every frame, after the drone it hangs
/// from has moved (TG_PostUpdateWork comes after ProjectAirSim's robot tick)
/// and before CARLA captures (OnWorldPostActorTick), so the view never lags
/// the drone by a frame.
UCLASS()
class UAgGimbalComponent : public UActorComponent
{
  GENERATED_BODY()

public:
  UAgGimbalComponent();

  virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                             FActorComponentTickFunction *ThisTickFunction) override;

  /// The world rotation the camera gets this frame.
  FQuat WorldRotation() const;

  /// Unreal rotation against the horizon frame named by bYawInEarthFrame.
  FQuat Attitude = FQuat::Identity;
  bool bYawInEarthFrame = false;
};
