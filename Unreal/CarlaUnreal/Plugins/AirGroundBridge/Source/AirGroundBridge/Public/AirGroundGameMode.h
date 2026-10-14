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

  /// Blueprint factories and the weather class are loaded here, not in the
  /// constructor: the CDO is built when this module loads (PostConfigInit),
  /// before the AnimationData plugin and AnimationDataController exist, and
  /// any Blueprint that reaches an animation asset at that point aborts the
  /// process. By InitGame every module is up.
  void InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage) override;

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
