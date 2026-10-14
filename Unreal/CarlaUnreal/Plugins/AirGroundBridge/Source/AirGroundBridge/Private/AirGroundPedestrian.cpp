#include "AirGroundPedestrian.h"

#include "Animation/AnimSequence.h"
#include "Components/CapsuleComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include "CollisionQueryParams.h"
#include "GameFramework/CharacterMovementComponent.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Walker/WalkerController.h"

namespace
{
const TCHAR *const kMannequinDir =
    TEXT("/Engine/Tutorial/SubEditors/TutorialAssets/Character/");
}

AAirGroundPedestrian::AAirGroundPedestrian(const FObjectInitializer &ObjectInitializer)
    : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;

  // CARLA's WalkerController possesses the character and feeds its
  // WalkerControl into AddMovementInput every tick.
  AIControllerClass = AWalkerController::StaticClass();
  AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;

  // CharacterMovement stays idle (see the header); Tick moves the actor and
  // feeds the component's Velocity so CARLA's get_velocity() stays truthful.
  UCharacterMovementComponent *Movement = GetCharacterMovement();
  Movement->GravityScale = 0.0f;
  Movement->DefaultLandMovementMode = MOVE_None;
  Movement->bOrientRotationToMovement = false;
  Movement->bUseControllerDesiredRotation = false;
  bUseControllerRotationYaw = false;

  GetCapsuleComponent()->InitCapsuleSize(CapsuleRadiusCm, CapsuleHalfHeightCm);
  // Queries only: sensors can see the pedestrian, nothing pushes it.
  GetCapsuleComponent()->SetCollisionEnabled(ECollisionEnabled::QueryOnly);

  // The mannequin's root is between its feet and it faces +Y; the capsule is
  // centred and faces +X.
  USkeletalMeshComponent *MeshComp = GetMesh();
  MeshComp->SetRelativeLocationAndRotation(FVector(0.0f, 0.0f, -90.0f),
                                           FRotator(0.0f, -90.0f, 0.0f));
  MeshComp->SetAnimationMode(EAnimationMode::AnimationSingleNode);
  MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void AAirGroundPedestrian::PostInitializeComponents()
{
  Super::PostInitializeComponents();
  if (HasAnyFlags(RF_ClassDefaultObject))
  {
    return;
  }
  const FString Dir(kMannequinDir);
  if (USkeletalMesh *Mesh = LoadObject<USkeletalMesh>(nullptr, *(Dir + TEXT("TutorialTPP.TutorialTPP"))))
  {
    GetMesh()->SetSkeletalMesh(Mesh);
  }
  else
  {
    UE_LOG(LogTemp, Warning, TEXT("AirGroundPedestrian: engine mannequin not found under %s"), kMannequinDir);
  }
  IdleAnim = LoadObject<UAnimSequence>(nullptr, *(Dir + TEXT("Tutorial_Idle.Tutorial_Idle")));
  WalkAnim = LoadObject<UAnimSequence>(nullptr, *(Dir + TEXT("Tutorial_Walk_Fwd.Tutorial_Walk_Fwd")));
}

void AAirGroundPedestrian::BeginPlay()
{
  Super::BeginPlay();
  GetCharacterMovement()->SetMovementMode(MOVE_None);
  if (IdleAnim)
  {
    GetMesh()->PlayAnimation(IdleAnim, true);
  }
}

TOptional<FVector> AAirGroundPedestrian::GroundedAt(const FVector &Centre) const
{
  const FVector Feet = Centre - FVector(0.0f, 0.0f, CapsuleHalfHeightCm);
  FCollisionQueryParams Params(SCENE_QUERY_STAT(PedestrianGround), /*bTraceComplex=*/true, this);
  FHitResult Hit;
  const FVector From = Feet + FVector(0.0f, 0.0f, MaxStepUpCm);
  const FVector To = Feet - FVector(0.0f, 0.0f, MaxDropCm);
  if (!GetWorld()->LineTraceSingleByChannel(Hit, From, To, ECC_Visibility, Params))
  {
    return {};
  }
  return FVector(Centre.X, Centre.Y, Hit.ImpactPoint.Z + CapsuleHalfHeightCm + 1.0f);
}

bool AAirGroundPedestrian::WallAhead(const FVector &Centre, const FVector &Step) const
{
  if (Step.IsNearlyZero())
  {
    return false;
  }
  FCollisionQueryParams Params(SCENE_QUERY_STAT(PedestrianWall), /*bTraceComplex=*/true, this);
  FHitResult Hit;
  const FVector Chest = Centre + FVector(0.0f, 0.0f, 30.0f);
  const FVector Ahead = Chest + Step.GetSafeNormal2D() * (CapsuleRadiusCm + 30.0f);
  return GetWorld()->LineTraceSingleByChannel(Hit, Chest, Ahead, ECC_Visibility, Params) &&
         FMath::Abs(Hit.ImpactNormal.Z) < 0.6f;
}

void AAirGroundPedestrian::Tick(float DeltaSeconds)
{
  Super::Tick(DeltaSeconds);

  float SpeedCmPerSec = 0.0f;
  if (const AWalkerController *Walker = Cast<AWalkerController>(GetController()))
  {
    const FWalkerControl Control = Walker->GetWalkerControl();
    if (!Control.Direction.IsNearlyZero())
    {
      FRotator Rotation = GetActorRotation();
      Rotation.Yaw = FMath::FixedTurn(Rotation.Yaw, Control.Direction.Rotation().Yaw,
                                      TurnRateDegPerSec * DeltaSeconds);
      SetActorRotation(Rotation);
    }
    SpeedCmPerSec = Control.Speed;  // signed, along the facing
  }

  const FVector Centre = GetActorLocation();
  FVector Step = GetActorForwardVector() * SpeedCmPerSec * DeltaSeconds;
  Step.Z = 0.0f;
  TOptional<FVector> Next = GroundedAt(Centre + Step);
  if (!Next.IsSet() || WallAhead(Centre, Step))
  {
    Step = FVector::ZeroVector;
    Next = GroundedAt(Centre);  // at least stay on the mesh where we stand
  }
  if (Next.IsSet())
  {
    SetActorLocation(Next.GetValue(), /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);
  }
  const FVector Velocity = DeltaSeconds > 0.0f ? Step / DeltaSeconds : FVector::ZeroVector;
  GetCharacterMovement()->Velocity = Velocity;

  // Signed speed along the facing: the walk cycle plays backwards when the
  // pedestrian backs up.
  const float Forward = FVector::DotProduct(Velocity, GetActorForwardVector());
  const bool bMoving = FMath::Abs(Forward) > 15.0f;
  if (bMoving != bWalking)
  {
    bWalking = bMoving;
    UAnimSequence *Anim = bWalking ? WalkAnim : IdleAnim;
    if (Anim)
    {
      GetMesh()->PlayAnimation(Anim, true);
    }
  }
  if (bWalking)
  {
    GetMesh()->SetPlayRate(FMath::Clamp(Forward / WalkCycleSpeedCmPerSec, -2.5f, 2.5f));
  }
  HideFromCamerasInside();
}

void AAirGroundPedestrian::HideFromCamerasInside()
{
  TArray<AActor *> Children;
  GetAttachedActors(Children);
  for (AActor *Child : Children)
  {
    USceneCaptureComponent2D *Capture = Child ? Child->FindComponentByClass<USceneCaptureComponent2D>() : nullptr;
    if (Capture == nullptr)
    {
      continue;
    }
    const bool bInside = FVector::Dist(Child->GetActorLocation(), GetActorLocation()) < FirstPersonRadiusCm;
    if (bInside)
    {
      Capture->HiddenActors.AddUnique(this);
    }
    else
    {
      Capture->HiddenActors.Remove(this);
    }
  }
}

TArray<FActorDefinition> AAirGroundPedestrianFactory::GetDefinitions()
{
  // Same shape CARLA's own MakeGenericDefinition produces (it is not
  // exported from the Carla module): dotted id, comma tags, a role_name.
  FActorDefinition Definition;
  Definition.Id = TEXT("walker.pedestrian.mannequin");
  Definition.Tags = TEXT("walker,pedestrian,mannequin");
  Definition.Class = AAirGroundPedestrian::StaticClass();
  FActorVariation Role;
  Role.Id = TEXT("role_name");
  Role.Type = EActorAttributeType::String;
  Role.RecommendedValues = {TEXT("default")};
  Role.bRestrictToRecommended = false;
  Definition.Variations.Emplace(Role);
  return {Definition};
}

FActorSpawnResult AAirGroundPedestrianFactory::SpawnActor(
    const FTransform &SpawnAtTransform,
    const FActorDescription &ActorDescription)
{
  UWorld *World = GetWorld();
  if (World == nullptr)
  {
    return {};
  }
  FActorSpawnParameters Params;
  Params.SpawnCollisionHandlingOverride =
      ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  AActor *Spawned = World->SpawnActor<AAirGroundPedestrian>(
      ActorDescription.Class ? ActorDescription.Class.Get()
                             : AAirGroundPedestrian::StaticClass(),
      SpawnAtTransform, Params);
  return FActorSpawnResult(Spawned);
}
