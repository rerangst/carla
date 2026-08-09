// AirGroundGameMode — one GameMode owning both worlds:
// inherits ACarlaGameModeBase (episode, traffic, recorder, dispatcher) and
// bootstraps ProjectAirSim's SimServer/scene as ordinary members of the same
// UWorld, so neither side competes for the GameMode slot.

#pragma once

#include "CoreMinimal.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "UnrealSimLoader.h"

#include "AirGroundGameMode.generated.h"

UCLASS()
class AIRGROUNDBRIDGE_API AAirGroundGameMode : public ACarlaGameModeBase
{
  GENERATED_BODY()

public:
  explicit AAirGroundGameMode(const FObjectInitializer &ObjectInitializer);

  void StartPlay() override;

  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected:
  void Tick(float DeltaSeconds) override;

private:
  /// ProjectAirSim spawns its robots asynchronously after the scene config
  /// arrives from a client, so the drone cannot be registered at StartPlay.
  /// Polls until it appears, then registers it once with CARLA.
  void TryRegisterAirRobots();

  AUnrealSimLoader SimLoader;

  /// Robots already handed to CARLA's registry, so each is registered once.
  UPROPERTY()
  TSet<AActor *> RegisteredAirRobots;
};
