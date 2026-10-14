// ReconTwin — a scanned site's collision geometry loaded straight from its
// twin package (docs/recon-twin-spec.md). The mesh is the only thing sensors
// and the drone's physics ever touch; the visual layer is separate and
// optional, so this must work headless and without any editor import.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "ReconTwin.generated.h"

class UProceduralMeshComponent;

/// One collision tile: a position-only triangle mesh with per-triangle
/// collision on every channel the two stacks query. The mesh is the root
/// component because ProjectAirSim reads a collision's semantic label from
/// the root component's custom-depth stencil.
UCLASS()
class AIRGROUNDBRIDGE_API AReconTwinTile : public AActor
{
  GENERATED_BODY()

public:
  AReconTwinTile();

  UPROPERTY(VisibleAnywhere)
  TObjectPtr<UProceduralMeshComponent> Mesh;
};

/// Keeps the collision mesh out of the pictures once the splat layer is up:
/// the tiles are flagged scene-capture-only (so the spectator never sees
/// them) and every CARLA RGB camera gets them in its HiddenActors list as it
/// appears. Depth, semantic and other encoding cameras keep seeing the mesh,
/// as do lidar and the drone's physics.
UCLASS()
class AIRGROUNDBRIDGE_API AReconTwinVisualCoordinator : public AActor
{
  GENERATED_BODY()

public:
  AReconTwinVisualCoordinator();
  virtual void Tick(float DeltaSeconds) override;

  TArray<TWeakObjectPtr<AActor>> TilesToHide;
};

struct FReconTwinSummary
{
  int32 Tiles = 0;
  int64 Vertices = 0;
  int64 Triangles = 0;
  int32 Splats = 0; // visual layer, 0 when off or absent
  FBox Bounds = FBox(ForceInit); // UE world, cm
  double Seconds = 0.0;
};

class AIRGROUNDBRIDGE_API FReconTwinLoader
{
public:
  /// Loads the twin named on the command line:
  ///   -ReconTwin=<path/twin.jsonc>  [-ReconTwinOffset=x,y,z]  (metres, UE axes)
  /// Returns false when no twin was requested or loading failed; the sim
  /// carries on either way, with the failure in the log.
  static bool LoadFromCommandLine(UWorld *World);

  /// Spawns one AReconTwinTile per collision tile of the manifest, with the
  /// twin's local origin placed at OriginCm.
  static bool Load(UWorld *World, const FString &ManifestPath,
                   const FVector &OriginCm, FReconTwinSummary &Summary);
};
