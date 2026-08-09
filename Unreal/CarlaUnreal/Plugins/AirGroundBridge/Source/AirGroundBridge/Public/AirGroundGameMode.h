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

private:
  AUnrealSimLoader SimLoader;
};
