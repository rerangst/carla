#include "AirGroundGameMode.h"

#include "EngineUtils.h"
#include "Misc/Paths.h"
#include "UObject/ConstructorHelpers.h"

#include "Carla/Actor/ActorDescription.h"

#include "Carla/AI/AIControllerFactory.h"
#include "Carla/Actor/CarlaActorFactory.h"
#include "Carla/Actor/StaticMeshFactory.h"
#include "Carla/Actor/UtilActorFactory.h"
#include "Carla/Sensor/SensorFactory.h"
#include "Carla/Trigger/TriggerFactory.h"
#include "Carla/Weather/Weather.h"

AAirGroundGameMode::AAirGroundGameMode(
    const FObjectInitializer &ObjectInitializer)
    : Super(ObjectInitializer),
      SimLoader(FPaths::ConvertRelativePathToFull(FPaths::ProjectDir()))
{
  // ACarlaGameModeBase declares ActorFactories/WeatherClass as EditAnywhere and
  // BP_CarlaGameMode fills them in. A C++ subclass inherits the class, not the
  // Blueprint's values, so without this the blueprint library is empty and
  // every spawn request fails.
  ActorFactories.Add(ASensorFactory::StaticClass());
  ActorFactories.Add(AStaticMeshFactory::StaticClass());
  ActorFactories.Add(ATriggerFactory::StaticClass());
  ActorFactories.Add(AUtilActorFactory::StaticClass());
  ActorFactories.Add(AAIControllerFactory::StaticClass());

  static ConstructorHelpers::FClassFinder<ACarlaActorFactory>
      VehicleFactory(TEXT("/Game/Carla/Blueprints/Vehicles/BP_VehicleFactory"));
  if (VehicleFactory.Succeeded())
  {
    ActorFactories.Add(VehicleFactory.Class);
  }

  // Walkers stay unregistered: CARLA's pedestrian animations abort on load
  // under UE 5.8 with "no data model found past upgrade object version",
  // taking the process with it.
  //
  // Two fixes were tried and both failed, so neither is worth repeating:
  //   * ResavePackages over Content/Carla/Animations rewrote all 72 assets
  //     (package version 1013 -> 1018, zero save failures) and rebuilt their
  //     compressed data. The crash was unchanged: a resave rewrites what is in
  //     memory, and the source data model is absent from the shipped content,
  //     so there is nothing to restore.
  //   * Moving the Death_Anim folder aside only advanced the crash to
  //     Animations/Depot/man_xfull_Anim. The defect is spread across the
  //     animation set, not isolated to one asset, so deleting offenders one at
  //     a time is not a path to working pedestrians.
  //
  // This needs animation content that carries a data model - re-imported from
  // source, or a CARLA content release built against UE 5.8.

  static ConstructorHelpers::FClassFinder<ACarlaActorFactory>
      PropFactory(TEXT("/Game/Carla/Blueprints/Props/BP_PropFactory"));
  if (PropFactory.Succeeded())
  {
    ActorFactories.Add(PropFactory.Class);
  }

  static ConstructorHelpers::FClassFinder<AWeather>
      WeatherBP(TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather"));
  if (WeatherBP.Succeeded())
  {
    WeatherClass = WeatherBP.Class;
  }
}

void AAirGroundGameMode::StartPlay()
{
  // CARLA first: episode and dispatcher must exist before any robot spawned
  // by PAS can later be registered with them.
  Super::StartPlay();

  SimLoader.LaunchSimulation(GetWorld());
}

void AAirGroundGameMode::Tick(float DeltaSeconds)
{
  Super::Tick(DeltaSeconds);
  TryRegisterAirRobots();
}

void AAirGroundGameMode::TryRegisterAirRobots()
{
  for (TActorIterator<AActor> It(GetWorld()); It; ++It)
  {
    AActor *Actor = *It;
    // Matched by class name rather than type: UnrealRobot lives in the
    // ProjectAirSim plugin's Private folder, and depending on its headers
    // here would couple the bridge to that plugin's internals.
    if (Actor == nullptr ||
        !Actor->GetClass()->GetName().Contains(TEXT("UnrealRobot")) ||
        RegisteredAirRobots.Contains(Actor))
    {
      continue;
    }

    FActorDescription Description;
    Description.Id = TEXT("airground.quadrotor");
    Description.Class = Actor->GetClass();

    if (RegisterExternalActor(*Actor, Description) != nullptr)
    {
      RegisteredAirRobots.Add(Actor);
      UE_LOG(LogTemp, Log,
             TEXT("AirGroundBridge: registered '%s' with CARLA as '%s'"),
             *Actor->GetName(), *Description.Id);
    }
  }
}

void AAirGroundGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  // PAS robots live inside the CARLA world; tear them down while it is alive.
  SimLoader.TeardownSimulation();

  Super::EndPlay(EndPlayReason);
}
