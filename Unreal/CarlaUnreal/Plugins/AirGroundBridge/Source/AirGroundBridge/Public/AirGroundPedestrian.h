// AirGroundPedestrian — a CARLA-controllable pedestrian that does not depend on
// CARLA's walker content.
//
// CARLA's own walkers abort at load on UE 5.8 (their animations carry no data
// model; see AirGroundGameMode.cpp), so the walker factory stays unregistered.
// This character uses the engine's tutorial mannequin and its two animation
// sequences instead, which the engine ships for every version, and registers
// itself with CARLA as `walker.pedestrian.mannequin`. The actor registry types
// any ACharacter as a walker, so the Python API's WalkerControl drives it
// through CARLA's AWalkerController exactly like a stock pedestrian.
//
// The mesh and animations are loaded per instance, not in the constructor:
// the CDO is built when the module loads (PostConfigInit), before the
// animation data-model modules exist, and loading an AnimSequence then aborts
// the process (the same ordering that kept CARLA's walkers out).
//
// Control semantics differ from stock walkers in one way that suits a
// first-person view: the control's direction is where the pedestrian faces,
// and the speed is signed along it (negative walks backwards), so a view
// camera attached to the character does not swing round when it backs up.
//
// Movement is kinematic rather than CharacterMovement's capsule sweep: a
// scanned mesh is noisy at the centimetre scale and the capsule snagged on it
// every few steps (1.4 m/s requested, 0.1 m/s achieved on the open plaza).
// Each tick the pedestrian steps along its facing, drops onto the mesh with a
// vertical trace, and refuses steps that climb more than a kerb, drop more
// than a ledge, leave the mesh, or run into a near-vertical face at chest
// height - the same rule the storyboard's walking camera used.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "Carla/Actor/ActorSpawnResult.h"
#include "Carla/Actor/CarlaActorFactory.h"

#include "AirGroundPedestrian.generated.h"

class UAnimSequence;

UCLASS()
class AIRGROUNDBRIDGE_API AAirGroundPedestrian : public ACharacter
{
  GENERATED_BODY()

public:
  explicit AAirGroundPedestrian(const FObjectInitializer &ObjectInitializer);

  void PostInitializeComponents() override;
  void BeginPlay() override;
  void Tick(float DeltaSeconds) override;

private:
  UPROPERTY()
  TObjectPtr<UAnimSequence> IdleAnim;

  UPROPERTY()
  TObjectPtr<UAnimSequence> WalkAnim;

  bool bWalking = false;

  /// Ground speed the walk cycle was authored at; the play rate scales with
  /// the actual speed so the feet do not slide.
  static constexpr float WalkCycleSpeedCmPerSec = 150.0f;
  static constexpr float TurnRateDegPerSec = 240.0f;
  static constexpr float CapsuleRadiusCm = 35.0f;
  static constexpr float CapsuleHalfHeightCm = 90.0f;
  static constexpr float MaxStepUpCm = 60.0f;
  static constexpr float MaxDropCm = 150.0f;

  /// Capsule centre over the mesh at (x, y) near the current height, or
  /// unset when there is no mesh within the step limits.
  TOptional<FVector> GroundedAt(const FVector &Centre) const;
  bool WallAhead(const FVector &Centre, const FVector &Step) const;

  /// A scene capture attached inside the body (a first-person camera) must
  /// not render the body itself, or the walk cycle swings an arm through the
  /// frame; captures further out (chase cameras) keep seeing it.
  void HideFromCamerasInside();
  static constexpr float FirstPersonRadiusCm = 100.0f;
};

UCLASS()
class AIRGROUNDBRIDGE_API AAirGroundPedestrianFactory : public ACarlaActorFactory
{
  GENERATED_BODY()

  TArray<FActorDefinition> GetDefinitions() final;

  FActorSpawnResult SpawnActor(
      const FTransform &SpawnAtTransform,
      const FActorDescription &ActorDescription) final;
};
