#include "AirGroundGameMode.h"
#include "AirGroundPedestrian.h"
#include "ReconTwin.h"

#include "EngineUtils.h"
#include "Misc/Paths.h"

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

  // The Blueprint factories (vehicles, walkers, props) and the weather class
  // are loaded in InitGame, see the header.
  ActorFactories.Add(AAirGroundPedestrianFactory::StaticClass());
}

namespace
{
template <typename T>
TSubclassOf<T> LoadBlueprintClass(const TCHAR *Path)
{
  UClass *Class = LoadClass<T>(nullptr, Path);
  if (Class == nullptr)
  {
    UE_LOG(LogTemp, Warning, TEXT("AirGroundBridge: Blueprint class %s not found"), Path);
  }
  return Class;
}
}  // namespace

void AAirGroundGameMode::InitGame(
    const FString &MapName, const FString &Options, FString &ErrorMessage)
{
  // CARLA's pedestrians were unloadable from this module's constructor: their
  // animations carry an AnimationSequencerDataModel from the AnimationData
  // plugin (PreDefault phase) and need AnimationDataController (loaded with
  // the startup modules), neither of which exists at PostConfigInit. Loaded
  // here, after every module, they work as upstream intends.
  for (const TCHAR *Path : {
           TEXT("/Game/Carla/Blueprints/Vehicles/BP_VehicleFactory.BP_VehicleFactory_C"),
           TEXT("/Game/Carla/Blueprints/Walkers/BP_WalkerFactory.BP_WalkerFactory_C"),
           TEXT("/Game/Carla/Blueprints/Props/BP_PropFactory.BP_PropFactory_C")})
  {
    if (TSubclassOf<ACarlaActorFactory> Factory = LoadBlueprintClass<ACarlaActorFactory>(Path))
    {
      ActorFactories.Add(Factory);
    }
  }
  if (TSubclassOf<AWeather> Weather = LoadBlueprintClass<AWeather>(
          TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather.BP_CarlaWeather_C")))
  {
    WeatherClass = Weather;
  }

  // Spawns the factories above, so they must be in place first.
  Super::InitGame(MapName, Options, ErrorMessage);
}

void AAirGroundGameMode::StartPlay()
{
  // CARLA first: episode and dispatcher must exist before any robot spawned
  // by PAS can later be registered with them.
  Super::StartPlay();

  // A scanned site, when one is named on the command line, goes in before
  // ProjectAirSim starts so its collision exists by the time the sim serves.
  FReconTwinLoader::LoadFromCommandLine(GetWorld());

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
