// GaussianSplatLayer — the twin's visual layer: the scan's 3D Gaussian splats,
// drawn by a scene view extension into every view (the spectator and every
// CARLA camera alike) after lighting and before post-processing. It writes
// colour only: no depth, no stencil, no collision, so depth, semantic and
// lidar sensors keep reading the collision mesh (docs/recon-twin-spec.md §0,
// §7 branch B).
//
// Input is an `.agsplat` pack from tools/make_twin_splats.py: a 64-byte header
// ("AGSP", version 3, count, flags, bbox, env count, cell count, range count),
// a 32-byte cell table (uniform grid cells: range span, tight bbox), a 16-byte
// range table (a cell's records at one level, finest first: record span,
// level, typical splat size) and 32-byte records in the scan's grid frame
// (metres): float3 position, half4 quaternion (w x y z), half3 scale, uint8
// rgba (display sRGB colour, linear opacity), 2 bytes padding. Version 1
// packs (no tables) load as one cell at one level. The scan's sky sphere
// comes last; -ReconTwinSplatsNoEnv drops it.
//
// Per view the renderer draws every cell in the frustum at the coarsest
// level whose typical splat projects to at most ag.Splat.LodPixels, within
// ag.Splat.Budget splats: level changes are never coarser than the grid.

#pragma once

#include "CoreMinimal.h"
#include "PrimitiveComponentId.h"

/// Scan grid metres -> UE world centimetres, row-major 3x3 (row = output axis).
/// The same map ReconTwin applies to the collision mesh.
struct FSplatFrame
{
  double M[3][3] = {{100.0, 0.0, 0.0}, {0.0, 100.0, 0.0}, {0.0, 0.0, 100.0}};
};

struct FGaussianSplatSummary
{
  int32 InFile = 0;
  int32 Loaded = 0;
  int32 Nodes = 0;    // grid cells (1 for a single-tier pack)
  int32 Capacity = 0; // splats sorted per view at most
  FBox Bounds = FBox(ForceInit); // UE world, cm
  double Seconds = 0.0;
};

class AIRGROUNDBRIDGE_API FGaussianSplatLayer
{
public:
  /// Reads a pack, moves it into the UE frame and hands it to the renderer.
  /// One layer per process; loading again replaces the previous one.
  static bool Load(const FString &PackPath, const FSplatFrame &Frame, const FVector &OriginCm,
                   FGaussianSplatSummary &Summary, FString &Error);
  static void Unload();
  static int32 NumLoaded();

  /// Tells the renderer how to recognise a view in which the collision mesh
  /// is hidden (one tile's primitive id, looked up in the view's hidden set;
  /// the main view by the flag). In such views the splats also write depth,
  /// so depth of field, TAA and fog treat them as the surfaces they stand
  /// for. In every other view, the encoding cameras among them, they write
  /// colour only and the mesh keeps its depth.
  static void SetMeshProbe(FPrimitiveComponentId TilePrimitive, bool bHiddenFromMainView);
};
