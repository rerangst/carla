// AgGimbal: a CARLA camera that points like a stabilised gimbal. See AgGimbalComponent.h.
#include "AgGimbalComponent.h"

#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Sensor/SceneCaptureSensor.h"
#include "Components/SceneCaptureComponent2D.h"
#include "HAL/IConsoleManager.h"

namespace
{
TArray<TWeakObjectPtr<UAgGimbalComponent>> GGimbals;

/// The gimbal cameras still alive (a camera destroyed by its client drops out).
TArray<UAgGimbalComponent *> LiveGimbals()
{
  GGimbals.RemoveAll([](const TWeakObjectPtr<UAgGimbalComponent> &G) { return !G.IsValid(); });
  TArray<UAgGimbalComponent *> Live;
  for (const TWeakObjectPtr<UAgGimbalComponent> &G : GGimbals)
  {
    Live.Add(G.Get());
  }
  return Live;
}

void Attach(const TArray<FString> &Args, UWorld *World, FOutputDevice &Ar)
{
  int32 Id = 0;
  if (Args.Num() != 1 || !LexTryParseString(Id, *Args[0]) || Id <= 0)
  {
    Ar.Logf(TEXT("usage: ag.Gimbal.Attach <carla actor id of a camera>"));
    return;
  }
  UCarlaEpisode *Episode = World != nullptr ? UCarlaStatics::GetCurrentEpisode(World) : nullptr;
  FCarlaActor *Found = Episode != nullptr ? Episode->FindCarlaActor((FCarlaActor::IdType)Id) : nullptr;
  ASceneCaptureSensor *Camera = Found != nullptr ? Cast<ASceneCaptureSensor>(Found->GetActor()) : nullptr;
  if (Camera == nullptr || Camera->GetRootComponent() == nullptr)
  {
    Ar.Logf(TEXT("ag.Gimbal.Attach: actor %d is not a camera in this episode"), Id);
    return;
  }
  UAgGimbalComponent *Gimbal = Camera->FindComponentByClass<UAgGimbalComponent>();
  if (Gimbal == nullptr)
  {
    Gimbal = NewObject<UAgGimbalComponent>(Camera, TEXT("AgGimbal"));
    Gimbal->RegisterComponent();
  }
  // The mount stays where it is on the airframe; only the turning is the gimbal's.
  Camera->GetRootComponent()->SetUsingAbsoluteRotation(true);
  AActor *Drone = Camera->GetAttachParentActor();
  if (Drone != nullptr)
  {
    // The airframe's own mesh sits around the gimbal and does not turn with
    // it; a real gimbal camera hangs below it and sees past it.
    Camera->GetCaptureComponent2D()->HiddenActors.AddUnique(Drone);
  }
  GGimbals.AddUnique(Gimbal);
  Ar.Logf(TEXT("gimbal camera %d on %s, horizontal fov %.1f"), Id,
          Drone != nullptr ? *Drone->GetName() : TEXT("nothing (yaw always against north)"), Camera->GetFOVAngle());
}

/// The last word of a pointing command: true for "earth", false for "vehicle".
bool ParseFrame(const FString &Word, bool &bEarth)
{
  bEarth = Word == TEXT("earth");
  return bEarth || Word == TEXT("vehicle");
}

void SetAttitude(const FQuat &Attitude, bool bEarth, FOutputDevice &Ar)
{
  const TArray<UAgGimbalComponent *> Gimbals = LiveGimbals();
  for (UAgGimbalComponent *G : Gimbals)
  {
    G->Attitude = Attitude;
    G->bYawInEarthFrame = bEarth;
  }
  Ar.Logf(TEXT("%s"), Gimbals.IsEmpty() ? TEXT("no gimbal camera (ag.Gimbal.Attach first)") : TEXT("ok"));
}

void Quat(const TArray<FString> &Args, UWorld *, FOutputDevice &Ar)
{
  double W = 0.0, X = 0.0, Y = 0.0, Z = 0.0;
  bool bEarth = false;
  if (Args.Num() != 5 || !LexTryParseString(W, *Args[0]) || !LexTryParseString(X, *Args[1]) ||
      !LexTryParseString(Y, *Args[2]) || !LexTryParseString(Z, *Args[3]) || !ParseFrame(Args[4], bEarth))
  {
    Ar.Logf(TEXT("usage: ag.Gimbal.Quat <w> <x> <y> <z> earth|vehicle (NED, as MAVLink sends it)"));
    return;
  }
  FQuat Attitude(-X, -Y, Z, W);
  if (Attitude.SizeSquared() < 1.e-6)
  {
    Ar.Logf(TEXT("ag.Gimbal.Quat: not a rotation"));
    return;
  }
  Attitude.Normalize();
  SetAttitude(Attitude, bEarth, Ar);
}

void Point(const TArray<FString> &Args, UWorld *, FOutputDevice &Ar)
{
  if (Args.IsEmpty())
  {
    const TArray<UAgGimbalComponent *> Gimbals = LiveGimbals();
    if (Gimbals.IsEmpty())
    {
      Ar.Logf(TEXT("no gimbal camera (ag.Gimbal.Attach first)"));
    }
    for (const UAgGimbalComponent *G : Gimbals)
    {
      const FRotator Set = G->Attitude.Rotator();
      const FRotator Now = G->GetOwner()->GetActorRotation();
      Ar.Logf(TEXT("%s: pitch %.2f yaw %.2f roll %.2f %s; world pitch %.2f yaw %.2f roll %.2f; fov %.2f"),
              *G->GetOwner()->GetName(), Set.Pitch, Set.Yaw, Set.Roll,
              G->bYawInEarthFrame ? TEXT("earth") : TEXT("vehicle"), Now.Pitch, Now.Yaw, Now.Roll,
              CastChecked<ASceneCaptureSensor>(G->GetOwner())->GetFOVAngle());
    }
    return;
  }
  float Pitch = 0.0f, Yaw = 0.0f, Roll = 0.0f;
  bool bEarth = false;
  if (Args.Num() != 4 || !LexTryParseString(Pitch, *Args[0]) || !LexTryParseString(Yaw, *Args[1]) ||
      !LexTryParseString(Roll, *Args[2]) || !ParseFrame(Args[3], bEarth))
  {
    Ar.Logf(TEXT("usage: ag.Gimbal.Point <pitch> <yaw> <roll> earth|vehicle (degrees)"));
    return;
  }
  SetAttitude(FRotator(Pitch, Yaw, Roll).Quaternion(), bEarth, Ar);
}

void Fov(const TArray<FString> &Args, UWorld *, FOutputDevice &Ar)
{
  float Degrees = 0.0f;
  if (Args.Num() != 1 || !LexTryParseString(Degrees, *Args[0]) || Degrees <= 0.0f || Degrees >= 180.0f)
  {
    Ar.Logf(TEXT("usage: ag.Gimbal.Fov <horizontal degrees, 0..180>"));
    return;
  }
  const TArray<UAgGimbalComponent *> Gimbals = LiveGimbals();
  for (UAgGimbalComponent *G : Gimbals)
  {
    CastChecked<ASceneCaptureSensor>(G->GetOwner())->SetFOVAngle(Degrees);
  }
  Ar.Logf(TEXT("fov %.2f on %d gimbal camera(s)"), Degrees, Gimbals.Num());
}

FAutoConsoleCommandWithWorldArgsAndOutputDevice GGimbalAttach(
    TEXT("ag.Gimbal.Attach"),
    TEXT("<carla actor id>: turn that camera like a stabilised gimbal (and hide the drone it hangs from from it)."),
    FConsoleCommandWithWorldArgsAndOutputDeviceDelegate::CreateStatic(&Attach));
FAutoConsoleCommandWithWorldArgsAndOutputDevice GGimbalQuat(
    TEXT("ag.Gimbal.Quat"),
    TEXT("<w> <x> <y> <z> earth|vehicle: point the gimbal cameras by the NED attitude quaternion of "
         "GIMBAL_DEVICE_ATTITUDE_STATUS (yaw against north or the drone's heading)."),
    FConsoleCommandWithWorldArgsAndOutputDeviceDelegate::CreateStatic(&Quat));
FAutoConsoleCommandWithWorldArgsAndOutputDevice GGimbalPoint(
    TEXT("ag.Gimbal.Point"),
    TEXT("<pitch> <yaw> <roll> earth|vehicle: point the gimbal cameras (degrees; pitch and roll against the horizon, "
         "yaw against north or the drone's heading). No arguments: print where they point."),
    FConsoleCommandWithWorldArgsAndOutputDeviceDelegate::CreateStatic(&Point));
FAutoConsoleCommandWithWorldArgsAndOutputDevice GGimbalFov(
    TEXT("ag.Gimbal.Fov"), TEXT("<horizontal degrees>: zoom the gimbal cameras."),
    FConsoleCommandWithWorldArgsAndOutputDeviceDelegate::CreateStatic(&Fov));
} // namespace

UAgGimbalComponent::UAgGimbalComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
  PrimaryComponentTick.TickGroup = TG_PostUpdateWork;
}

FQuat UAgGimbalComponent::WorldRotation() const
{
  const AActor *Drone = GetOwner() != nullptr ? GetOwner()->GetAttachParentActor() : nullptr;
  if (bYawInEarthFrame || Drone == nullptr)
  {
    return Attitude;
  }
  return FRotator(0.0, Drone->GetActorRotation().Yaw, 0.0).Quaternion() * Attitude;
}

void UAgGimbalComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                       FActorComponentTickFunction *ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
  if (USceneComponent *Root = GetOwner() != nullptr ? GetOwner()->GetRootComponent() : nullptr)
  {
    Root->SetWorldRotation(WorldRotation());
  }
}
