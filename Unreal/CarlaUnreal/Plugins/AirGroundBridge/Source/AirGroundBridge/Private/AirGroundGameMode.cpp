#include "AirGroundGameMode.h"

#include "Misc/Paths.h"

AAirGroundGameMode::AAirGroundGameMode(
    const FObjectInitializer &ObjectInitializer)
    : Super(ObjectInitializer),
      SimLoader(FPaths::ConvertRelativePathToFull(FPaths::ProjectDir()))
{
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
