#include "AirGroundGameMode.h"

#include "Misc/Paths.h"
#include "UObject/ConstructorHelpers.h"

#include "Carla/AI/AIControllerFactory.h"
#include "Carla/Actor/CarlaActorFactoryBlueprint.h"
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

  static ConstructorHelpers::FClassFinder<ACarlaActorFactoryBlueprint>
      VehicleFactory(TEXT("/Game/Carla/Blueprints/Vehicles/BP_VehicleFactory"));
  if (VehicleFactory.Succeeded())
  {
    ActorFactories.Add(VehicleFactory.Class);
  }

  static ConstructorHelpers::FClassFinder<ACarlaActorFactoryBlueprint>
      WalkerFactory(TEXT("/Game/Carla/Blueprints/Walkers/BP_WalkerFactory"));
  if (WalkerFactory.Succeeded())
  {
    ActorFactories.Add(WalkerFactory.Class);
  }

  static ConstructorHelpers::FClassFinder<ACarlaActorFactoryBlueprint>
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

void AAirGroundGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  // PAS robots live inside the CARLA world; tear them down while it is alive.
  SimLoader.TeardownSimulation();

  Super::EndPlay(EndPlayReason);
}
