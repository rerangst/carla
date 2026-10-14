#include "ReconTwin.h"

#include "GaussianSplatLayer.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "Materials/MaterialInterface.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "ProceduralMeshComponent.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#include "Carla/Game/Tagger.h"
#include "Carla/Sensor/SceneCaptureCamera.h"

DEFINE_LOG_CATEGORY_STATIC(LogReconTwin, Log, All);

namespace crp = carla::rpc;

// LCC2 is right-handed, metres, (x east, y north, z up) on its UTM grid. The
// twin's GPS goes through ProjectAirSim, whose NED->UE mapping is
// (north, east, -down) -> (X, Y, Z) (UnrealHelpers::ToFVectorTranslation), so
// true north must land on +X and east on +Y. Grid north is not true north
// (grid convergence, about 1 deg at this sample's longitude) and a grid metre
// is not a ground metre (scale factor), so the manifest carries a 2x2 matrix
// from grid (east, north) to true (north, east), computed by
// tools/make_twin_package.py; without it the axes are simply swapped.
// Either way the map is a reflection, which flips the sign of every cross
// product; UE's own face normal is (P1-P2)x(P0-P2), the negative of the
// right-handed rule. The two flips cancel, so the original index order is
// kept and normals come out facing outward.
struct FGridToTrue
{
  double NorthOfEast = 0.0, NorthOfNorth = 1.0;
  double EastOfEast = 1.0, EastOfNorth = 0.0;

  FVector ToUnreal(const FVector3f &P) const
  {
    const double North = NorthOfEast * P.X + NorthOfNorth * P.Y;
    const double East = EastOfEast * P.X + EastOfNorth * P.Y;
    return FVector(North * 100.0, East * 100.0, P.Z * 100.0);
  }

  /// The same map as a matrix, for the splat layer: grid metres -> UE cm.
  FSplatFrame ToSplatFrame() const
  {
    FSplatFrame F;
    F.M[0][0] = NorthOfEast * 100.0; F.M[0][1] = NorthOfNorth * 100.0; F.M[0][2] = 0.0;
    F.M[1][0] = EastOfEast * 100.0;  F.M[1][1] = EastOfNorth * 100.0;  F.M[1][2] = 0.0;
    F.M[2][0] = 0.0;                 F.M[2][1] = 0.0;                  F.M[2][2] = 100.0;
    return F;
  }
};

/// CARLA's empty test level has no light at all; a twin loaded there would
/// render black. Only added when the level brings none of its own.
static void EnsureLighting(UWorld *World)
{
  for (TActorIterator<ADirectionalLight> It(World); It; ++It)
  {
    UE_LOG(LogReconTwin, Display, TEXT("ReconTwin: level lights itself (%s at %.0f lux)"),
           *It->GetName(), It->GetLightComponent()->Intensity);
    return;
  }
  ADirectionalLight *Sun = World->SpawnActor<ADirectionalLight>(
      FVector::ZeroVector, FRotator(-62.0f, 35.0f, 0.0f));
  if (Sun != nullptr)
  {
    Sun->SetMobility(EComponentMobility::Movable);
    if (UDirectionalLightComponent *Light = Cast<UDirectionalLightComponent>(Sun->GetLightComponent()))
    {
      // CARLA's cameras auto-expose within a bounded EV range, tuned for a
      // real sun; UE's own SunSky uses 75000 lux, 2000 lux renders as dusk.
      Light->SetIntensity(75000.0f);
      Light->SetAtmosphereSunLight(true);
    }
  }
  World->SpawnActor<ASkyAtmosphere>();
  ASkyLight *Sky = World->SpawnActor<ASkyLight>();
  if (Sky != nullptr)
  {
    Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Sky->GetLightComponent()->SetRealTimeCapture(true);
    Sky->GetLightComponent()->SetIntensity(2.5f); // fill for the faces the sun misses
  }
  UE_LOG(LogReconTwin, Display, TEXT("ReconTwin: level had no light; added sun, sky light and atmosphere"));
}

AReconTwinVisualCoordinator::AReconTwinVisualCoordinator()
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.bStartWithTickEnabled = true;
  SetActorTickInterval(0.5f);
}

void AReconTwinVisualCoordinator::Tick(float DeltaSeconds)
{
  Super::Tick(DeltaSeconds);
  UWorld *World = GetWorld();
  if (World == nullptr)
  {
    return;
  }
  for (TActorIterator<ASceneCaptureCamera> It(World); It; ++It)
  {
    USceneCaptureComponent2D *Capture = It->GetCaptureComponent2D();
    if (Capture == nullptr)
    {
      continue;
    }
    for (const TWeakObjectPtr<AActor> &Tile : TilesToHide)
    {
      AActor *Actor = Tile.Get();
      if (Actor != nullptr && !Capture->HiddenActors.Contains(Actor))
      {
        Capture->HiddenActors.Add(Actor);
      }
    }
  }
}

AReconTwinTile::AReconTwinTile()
{
  PrimaryActorTick.bCanEverTick = false;
  Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("CollisionMesh"));
  SetRootComponent(Mesh);

  // Spec §3, the four contracts. Complex-as-simple makes the triangle mesh
  // serve every query: CARLA's project_point traces simple collision, its
  // lidar traces complex, PAS's ground check traces Visibility complex, and
  // the drone's Chaos body hits simple. One geometry, no silent mismatch.
  Mesh->bUseComplexAsSimpleCollision = true;
  Mesh->bUseAsyncCooking = false; // ready before the sim starts serving
  Mesh->SetCollisionObjectType(ECC_WorldStatic);
  Mesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
  Mesh->SetCollisionResponseToAllChannels(ECR_Block);
  // SensorTrace defaults to Ignore in CARLA's DefaultEngine.ini; a lidar on
  // a tile that ignores it runs fine and returns nothing.
  Mesh->SetCollisionResponseToChannel(ECC_GameTraceChannel2, ECR_Block);
  Mesh->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
  Mesh->SetCanEverAffectNavigation(false);
}

// ------------------------------------------------------------------ PLY

namespace
{

struct FPlyMesh
{
  TArray<FVector3f> Positions;
  TArray<int32> Indices; // triangles, 3 per face
};

int32 PlyTypeSize(const FString &Type)
{
  if (Type == TEXT("char") || Type == TEXT("uchar") || Type == TEXT("int8") ||
      Type == TEXT("uint8"))
    return 1;
  if (Type == TEXT("short") || Type == TEXT("ushort") || Type == TEXT("int16") ||
      Type == TEXT("uint16"))
    return 2;
  if (Type == TEXT("int") || Type == TEXT("uint") || Type == TEXT("int32") ||
      Type == TEXT("uint32") || Type == TEXT("float") || Type == TEXT("float32"))
    return 4;
  if (Type == TEXT("double") || Type == TEXT("float64"))
    return 8;
  return 0;
}

uint32 ReadUnsigned(const uint8 *At, int32 Size)
{
  switch (Size)
  {
    case 1: return *At;
    case 2: { uint16 V; FMemory::Memcpy(&V, At, 2); return V; }
    default: { uint32 V; FMemory::Memcpy(&V, At, 4); return V; }
  }
}

/// Binary little-endian PLY with float x y z vertices and a face list of
/// 32-bit indices, which is what LCC2 writes. Anything else is refused with
/// a reason rather than read as garbage.
bool ReadPly(const FString &Path, FPlyMesh &Out, FString &Error)
{
  TArray<uint8> Bytes;
  if (!FFileHelper::LoadFileToArray(Bytes, *Path))
  {
    Error = TEXT("cannot read file");
    return false;
  }
  const int32 Total = Bytes.Num();
  int32 Pos = 0;
  auto ReadLine = [&](FString &Line) -> bool {
    if (Pos >= Total) return false;
    Line.Reset();
    while (Pos < Total && Bytes[Pos] != '\n')
    {
      Line.AppendChar(static_cast<TCHAR>(Bytes[Pos++]));
    }
    ++Pos; // the newline
    Line.TrimEndInline();
    return true;
  };

  int32 VertexCount = 0, FaceCount = 0, VertexStride = 0;
  int32 OffsetX = -1, OffsetY = -1, OffsetZ = -1;
  int32 CountSize = 0, IndexSize = 0;
  bool bLittleEndian = false;
  FString Element, Line;
  if (!ReadLine(Line) || Line != TEXT("ply"))
  {
    Error = TEXT("not a PLY file");
    return false;
  }
  while (ReadLine(Line))
  {
    if (Line == TEXT("end_header")) break;
    TArray<FString> Words;
    Line.ParseIntoArrayWS(Words);
    if (Words.Num() == 0 || Words[0] == TEXT("comment")) continue;
    if (Words[0] == TEXT("format"))
    {
      bLittleEndian = Words.Num() > 1 && Words[1] == TEXT("binary_little_endian");
    }
    else if (Words[0] == TEXT("element") && Words.Num() == 3)
    {
      Element = Words[1];
      if (Element == TEXT("vertex")) VertexCount = FCString::Atoi(*Words[2]);
      else if (Element == TEXT("face")) FaceCount = FCString::Atoi(*Words[2]);
    }
    else if (Words[0] == TEXT("property") && Element == TEXT("vertex") && Words.Num() == 3)
    {
      const int32 Size = PlyTypeSize(Words[1]);
      if (Size == 0) { Error = TEXT("unknown vertex property type ") + Words[1]; return false; }
      if (Words[2] == TEXT("x") || Words[2] == TEXT("y") || Words[2] == TEXT("z"))
      {
        if (Words[1] != TEXT("float") && Words[1] != TEXT("float32"))
        {
          Error = TEXT("vertex coordinates are not float32");
          return false;
        }
        (Words[2] == TEXT("x") ? OffsetX : Words[2] == TEXT("y") ? OffsetY : OffsetZ) = VertexStride;
      }
      VertexStride += Size;
    }
    else if (Words[0] == TEXT("property") && Element == TEXT("face") && Words.Num() == 5 &&
             Words[1] == TEXT("list"))
    {
      CountSize = PlyTypeSize(Words[2]);
      IndexSize = PlyTypeSize(Words[3]);
    }
  }
  if (!bLittleEndian) { Error = TEXT("not binary_little_endian"); return false; }
  if (OffsetX < 0 || OffsetY < 0 || OffsetZ < 0) { Error = TEXT("vertex lacks x y z"); return false; }
  if (CountSize == 0 || IndexSize != 4) { Error = TEXT("face list is not <count> int32[]"); return false; }
  if (Pos + static_cast<int64>(VertexCount) * VertexStride > Total)
  {
    Error = TEXT("vertex block runs past end of file");
    return false;
  }

  Out.Positions.SetNumUninitialized(VertexCount);
  for (int32 i = 0; i < VertexCount; ++i)
  {
    const uint8 *At = &Bytes[Pos + i * VertexStride];
    float X, Y, Z;
    FMemory::Memcpy(&X, At + OffsetX, 4);
    FMemory::Memcpy(&Y, At + OffsetY, 4);
    FMemory::Memcpy(&Z, At + OffsetZ, 4);
    Out.Positions[i] = FVector3f(X, Y, Z);
  }
  Pos += VertexCount * VertexStride;

  Out.Indices.Reserve(FaceCount * 3);
  for (int32 f = 0; f < FaceCount; ++f)
  {
    if (Pos + CountSize > Total) { Error = TEXT("face block truncated"); return false; }
    const int32 Count = static_cast<int32>(ReadUnsigned(&Bytes[Pos], CountSize));
    Pos += CountSize;
    if (Pos + Count * 4 > Total) { Error = TEXT("face block truncated"); return false; }
    int32 First = 0, Prev = 0;
    for (int32 k = 0; k < Count; ++k)
    {
      int32 Index;
      FMemory::Memcpy(&Index, &Bytes[Pos + k * 4], 4);
      if (Index < 0 || Index >= VertexCount)
      {
        Error = FString::Printf(TEXT("face %d references vertex %d of %d"), f, Index, VertexCount);
        return false;
      }
      if (k == 0) First = Index;
      else if (k >= 2) // fan-triangulate anything beyond a triangle
      {
        Out.Indices.Add(First);
        Out.Indices.Add(Prev);
        Out.Indices.Add(Index);
      }
      Prev = Index;
    }
    Pos += Count * 4;
  }
  return true;
}

/// JSONC -> JSON: drops // and /* */ comments outside string literals.
FString StripJsonComments(const FString &In)
{
  FString Out;
  Out.Reserve(In.Len());
  bool bInString = false, bEscape = false;
  const int32 N = In.Len();
  for (int32 i = 0; i < N; ++i)
  {
    const TCHAR C = In[i];
    if (bInString)
    {
      Out.AppendChar(C);
      if (bEscape) bEscape = false;
      else if (C == '\\') bEscape = true;
      else if (C == '"') bInString = false;
      continue;
    }
    if (C == '"') { bInString = true; Out.AppendChar(C); continue; }
    if (C == '/' && i + 1 < N && In[i + 1] == '/')
    {
      while (i < N && In[i] != '\n') ++i;
      Out.AppendChar('\n');
      continue;
    }
    if (C == '/' && i + 1 < N && In[i + 1] == '*')
    {
      i += 2;
      while (i + 1 < N && !(In[i] == '*' && In[i + 1] == '/')) ++i;
      ++i;
      continue;
    }
    Out.AppendChar(C);
  }
  return Out;
}

} // namespace

// --------------------------------------------------------- visual layer

namespace
{

/// Picks and loads a splat pack listed under `visual.packs` in the manifest
/// (first listed wins: make_twin_package.py puts the multi-level pack first).
///   -ReconTwinSplats=off        collision mesh only (the v0 vertex colours)
///   -ReconTwinSplats=<lod|name> a single tier by number, or a pack by file name
///   -ReconTwinSplatBudget=<n>   splats sorted and drawn per view at most (ag.Splat.Budget)
///   -ReconTwinSplatsKeepMesh    leave the mesh visible under the splats
///   -ReconTwinSplatsNoEnv       drop the scan's sky sphere; keep the level's sky and ground
void LoadVisualLayer(UWorld *World, const TSharedPtr<FJsonObject> &Root, const FString &BaseDir,
                     const FGridToTrue &Frame, const FVector &OriginCm,
                     const TArray<AReconTwinTile *> &TileActors, AStaticMeshActor *Ground,
                     FReconTwinSummary &Summary)
{
  FString Choice;
  FParse::Value(FCommandLine::Get(), TEXT("ReconTwinSplats="), Choice);
  if (Choice == TEXT("off"))
  {
    UE_LOG(LogReconTwin, Display, TEXT("ReconTwin splats: off by request"));
    return;
  }
  const TSharedPtr<FJsonObject> *Visual = nullptr;
  const TArray<TSharedPtr<FJsonValue>> *Packs = nullptr;
  if (!Root->TryGetObjectField(TEXT("visual"), Visual) || !(*Visual)->TryGetArrayField(TEXT("packs"), Packs) ||
      Packs->Num() == 0)
  {
    UE_LOG(LogReconTwin, Display, TEXT("ReconTwin splats: manifest lists no packs (tools/make_twin_splats.py)"));
    return;
  }
  const bool bWantNumber = !Choice.IsEmpty() && FCString::IsNumeric(*Choice);
  const int32 WantedLod = bWantNumber ? FCString::Atoi(*Choice) : -1;

  FString File;
  int32 Lod = -1, Count = 0;
  bool bFound = false;
  for (const TSharedPtr<FJsonValue> &Value : *Packs)
  {
    const TSharedPtr<FJsonObject> Pack = Value->AsObject();
    if (!Pack.IsValid()) continue;
    const FString PackFile = Pack->GetStringField(TEXT("file"));
    double PackLodNumber = -1.0;
    Pack->TryGetNumberField(TEXT("lod"), PackLodNumber);
    const int32 PackLod = (int32)PackLodNumber;
    const int32 PackCount = (int32)Pack->GetNumberField(TEXT("count"));
    const bool bTake = Choice.IsEmpty() ? true
                       : bWantNumber   ? (PackLod == WantedLod)
                                       : FPaths::GetBaseFilename(PackFile).Contains(Choice);
    if (bTake)
    {
      File = PackFile;
      Lod = PackLod;
      Count = PackCount;
      bFound = true;
      break;
    }
  }
  if (!bFound)
  {
    UE_LOG(LogReconTwin, Error, TEXT("ReconTwin splats: no pack matches -ReconTwinSplats=%s"), *Choice);
    return;
  }

  FGaussianSplatSummary Splats;
  FString Error;
  const FString PackPath = FPaths::Combine(BaseDir, File);
  if (!FGaussianSplatLayer::Load(PackPath, Frame.ToSplatFrame(), OriginCm, Splats, Error))
  {
    UE_LOG(LogReconTwin, Error, TEXT("ReconTwin splats: %s: %s"), *PackPath, *Error);
    return;
  }
  Summary.Splats = Splats.Loaded;

  const bool bKeepMesh = FParse::Param(FCommandLine::Get(), TEXT("ReconTwinSplatsKeepMesh"));
  if (!bKeepMesh)
  {
    AReconTwinVisualCoordinator *Coordinator = World->SpawnActor<AReconTwinVisualCoordinator>();
    for (AReconTwinTile *Tile : TileActors)
    {
      Tile->Mesh->SetVisibleInSceneCaptureOnly(true);
      if (Coordinator != nullptr)
      {
        Coordinator->TilesToHide.Add(Tile);
      }
    }
    if (TileActors.Num() > 0)
    {
      FGaussianSplatLayer::SetMeshProbe(TileActors[0]->Mesh->GetPrimitiveSceneId(), /*bHiddenFromMainView=*/true);
    }
    // The scan's own surroundings (the env sphere in the pack) stand in for
    // the ground plane in pictures; the plane stays for the encoding cameras.
    if (Ground != nullptr && !FParse::Param(FCommandLine::Get(), TEXT("ReconTwinSplatsNoEnv")))
    {
      Ground->GetStaticMeshComponent()->SetVisibleInSceneCaptureOnly(true);
      if (Coordinator != nullptr)
      {
        Coordinator->TilesToHide.Add(Ground);
      }
    }
  }
  const FVector Min = Splats.Bounds.Min, Max = Splats.Bounds.Max;
  UE_LOG(LogReconTwin, Display,
         TEXT("ReconTwin splats: %s, %d splats in %d grid cells (%d listed, lod %d), up to %d per view, "
              "bounds [%.1f %.1f %.1f]..[%.1f %.1f %.1f] cm, %.1f s, mesh %s"),
         *File, Splats.Loaded, Splats.Nodes, Count, Lod, Splats.Capacity, Min.X, Min.Y, Min.Z, Max.X, Max.Y, Max.Z,
         Splats.Seconds, bKeepMesh ? TEXT("kept visible") : TEXT("hidden from RGB cameras and the spectator"));
}

} // namespace

// --------------------------------------------------------------- loader

bool FReconTwinLoader::Load(UWorld *World, const FString &ManifestPath,
                            const FVector &OriginCm, FReconTwinSummary &Summary)
{
  const double Started = FPlatformTime::Seconds();
  FString Text;
  if (!FFileHelper::LoadFileToString(Text, *ManifestPath))
  {
    UE_LOG(LogReconTwin, Error, TEXT("ReconTwin: cannot read manifest %s"), *ManifestPath);
    return false;
  }
  TSharedPtr<FJsonObject> Root;
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(StripJsonComments(Text));
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
  {
    UE_LOG(LogReconTwin, Error, TEXT("ReconTwin: manifest %s is not valid JSON(C): %s"),
           *ManifestPath, *Reader->GetErrorMessage());
    return false;
  }
  const TSharedPtr<FJsonObject> *Collision = nullptr;
  const TArray<TSharedPtr<FJsonValue>> *Tiles = nullptr;
  if (!Root->TryGetObjectField(TEXT("collision"), Collision) ||
      !(*Collision)->TryGetArrayField(TEXT("tiles"), Tiles))
  {
    UE_LOG(LogReconTwin, Error, TEXT("ReconTwin: manifest has no collision.tiles"));
    return false;
  }

  FGridToTrue Frame;
  const TSharedPtr<FJsonObject> *Geo = nullptr;
  const TSharedPtr<FJsonObject> *Grid = nullptr;
  const TArray<TSharedPtr<FJsonValue>> *Rows = nullptr;
  if (Root->TryGetObjectField(TEXT("geo"), Geo) &&
      (*Geo)->TryGetObjectField(TEXT("grid_to_true"), Grid) &&
      (*Grid)->TryGetArrayField(TEXT("matrix"), Rows) && Rows->Num() == 2 &&
      (*Rows)[0]->AsArray().Num() == 2 && (*Rows)[1]->AsArray().Num() == 2)
  {
    Frame.NorthOfEast = (*Rows)[0]->AsArray()[0]->AsNumber();
    Frame.NorthOfNorth = (*Rows)[0]->AsArray()[1]->AsNumber();
    Frame.EastOfEast = (*Rows)[1]->AsArray()[0]->AsNumber();
    Frame.EastOfNorth = (*Rows)[1]->AsArray()[1]->AsNumber();
  }
  else
  {
    UE_LOG(LogReconTwin, Warning,
           TEXT("ReconTwin: manifest has no geo.grid_to_true matrix; taking grid north as true north"));
  }

  EnsureLighting(World);

  // The visual layer's v0: per-vertex colours taken from the scan's splats
  // (tools/make_twin_colors.py), shown through the engine's lit vertex-colour
  // material. (The editor's colour-only view-mode material renders black in a
  // game world, so an unlit look needs a material of our own later.) Absent
  // colours leave the default grey; sensors never read either.
  UMaterialInterface *VertexColorMaterial = LoadObject<UMaterialInterface>(
      nullptr, TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));

  const FString BaseDir = FPaths::GetPath(ManifestPath);
  TArray<AReconTwinTile *> TileActors;
  for (const TSharedPtr<FJsonValue> &Value : *Tiles)
  {
    const TSharedPtr<FJsonObject> Tile = Value->AsObject();
    FString File;
    if (!Tile.IsValid() || !Tile->TryGetStringField(TEXT("file"), File))
    {
      UE_LOG(LogReconTwin, Error, TEXT("ReconTwin: tile entry without 'file'"));
      return false;
    }
    FString LabelName = TEXT("Building");
    Tile->TryGetStringField(TEXT("label"), LabelName);
    crp::CityObjectLabel Label = ATagger::GetLabelByFolderName(LabelName);
    if (Label == crp::CityObjectLabel::None)
    {
      UE_LOG(LogReconTwin, Warning, TEXT("ReconTwin: unknown label '%s' on %s, using Building"),
             *LabelName, *File);
      Label = crp::CityObjectLabel::Buildings;
    }

    const FString PlyPath = FPaths::Combine(BaseDir, File);
    FPlyMesh Ply;
    FString Error;
    if (!ReadPly(PlyPath, Ply, Error))
    {
      UE_LOG(LogReconTwin, Error, TEXT("ReconTwin: %s: %s"), *PlyPath, *Error);
      return false;
    }

    TArray<FVector> Vertices;
    Vertices.SetNumUninitialized(Ply.Positions.Num());
    for (int32 i = 0; i < Ply.Positions.Num(); ++i)
    {
      Vertices[i] = Frame.ToUnreal(Ply.Positions[i]);
      Summary.Bounds += Vertices[i] + OriginCm;
    }
    TArray<FVector> Normals;
    Normals.SetNumZeroed(Vertices.Num());
    for (int32 t = 0; t + 2 < Ply.Indices.Num(); t += 3)
    {
      const int32 I0 = Ply.Indices[t], I1 = Ply.Indices[t + 1], I2 = Ply.Indices[t + 2];
      // UE's face normal; area-weighted by leaving it unnormalised.
      const FVector N = (Vertices[I1] - Vertices[I2]) ^ (Vertices[I0] - Vertices[I2]);
      Normals[I0] += N;
      Normals[I1] += N;
      Normals[I2] += N;
    }
    for (FVector &N : Normals)
    {
      N = N.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
    }

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(
        World->GetCurrentLevel(), AReconTwinTile::StaticClass(),
        FName(*FString::Printf(TEXT("ReconTwin_%s"), *FPaths::GetBaseFilename(File))));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AReconTwinTile *Actor = World->SpawnActor<AReconTwinTile>(
        AReconTwinTile::StaticClass(), FTransform(OriginCm), Params);
    if (Actor == nullptr)
    {
      UE_LOG(LogReconTwin, Error, TEXT("ReconTwin: could not spawn tile %s"), *File);
      return false;
    }
    TArray<FLinearColor> Colors;
    FString ColorFile;
    if (Tile->TryGetStringField(TEXT("colors"), ColorFile))
    {
      TArray<uint8> Bytes;
      if (FFileHelper::LoadFileToArray(Bytes, *FPaths::Combine(BaseDir, ColorFile)) &&
          Bytes.Num() == Vertices.Num() * 3)
      {
        Colors.SetNumUninitialized(Vertices.Num());
        for (int32 i = 0; i < Vertices.Num(); ++i)
        {
          // The bytes are display (sRGB) values; FLinearColor(FColor) decodes them.
          Colors[i] = FLinearColor(FColor(Bytes[i * 3], Bytes[i * 3 + 1], Bytes[i * 3 + 2]));
        }
      }
      else
      {
        UE_LOG(LogReconTwin, Warning, TEXT("ReconTwin: %s: colour file missing or not %d bytes"),
               *ColorFile, Vertices.Num() * 3);
      }
    }
    Actor->Mesh->CreateMeshSection_LinearColor(
        0, Vertices, Ply.Indices, Normals, TArray<FVector2D>(),
        Colors, TArray<FProcMeshTangent>(), /*bCreateCollision=*/true);
    // A scan is an open shell seen from either side; a second section with
    // the winding reversed draws the back faces, which the single-sided
    // material would otherwise cull into black holes. No collision on it:
    // the triangles are the same ones.
    TArray<int32> Reversed;
    Reversed.SetNumUninitialized(Ply.Indices.Num());
    for (int32 t = 0; t + 2 < Ply.Indices.Num(); t += 3)
    {
      Reversed[t] = Ply.Indices[t];
      Reversed[t + 1] = Ply.Indices[t + 2];
      Reversed[t + 2] = Ply.Indices[t + 1];
    }
    TArray<FVector> BackNormals;
    BackNormals.SetNumUninitialized(Normals.Num());
    for (int32 i = 0; i < Normals.Num(); ++i)
    {
      BackNormals[i] = -Normals[i];
    }
    Actor->Mesh->CreateMeshSection_LinearColor(
        1, Vertices, Reversed, BackNormals, TArray<FVector2D>(),
        Colors, TArray<FProcMeshTangent>(), /*bCreateCollision=*/false);
    if (Colors.Num() > 0 && VertexColorMaterial != nullptr)
    {
      Actor->Mesh->SetMaterial(0, VertexColorMaterial);
      Actor->Mesh->SetMaterial(1, VertexColorMaterial);
    }

    // Spec §3 trap 2: one stencil serves both readers. PAS copies the root
    // component's CustomDepthStencilValue into collision_info.segmentation_id;
    // CARLA's tagger reads the same field for untagged components, and its
    // semantic camera takes the label from custom primitive data.
    Actor->Mesh->SetCustomDepthStencilValue(static_cast<int32>(Label));
    Actor->Mesh->SetRenderCustomDepth(true);
    ATagger::SetStencilValue(*Actor->Mesh, Actor->GetUniqueID(), Label, true);
    // CARLA's lidars read the label from ComponentTags[0] without checking
    // the array has one (RayCastSemanticLidar.cpp:227); a hit on an untagged
    // component asserts and takes the sim down. The tagger itself writes the
    // label's name there, so do the same.
    Actor->Mesh->ComponentTags.Add(FName(*ATagger::GetTagAsString(Label)));

    Summary.Tiles += 1;
    Summary.Vertices += Vertices.Num();
    Summary.Triangles += Ply.Indices.Num() / 3;
    TileActors.Add(Actor);
  }

  // A level with nothing but the twin shows black void around and under it.
  // A plain ground plane below the lowest tile reads as terrain in pictures.
  // No collision and no tag: sensors and the drone never know it is there.
  AStaticMeshActor *Ground = nullptr;
  if (Summary.Bounds.IsValid)
  {
    UStaticMesh *Plane = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Plane.Plane"));
    if (Plane != nullptr)
    {
      const FVector Centre = Summary.Bounds.GetCenter();
      // 2 m below the lowest vertex, in cm. Collision is switched off before
      // the actor finishes spawning: done afterwards it did not take, and the
      // lidar found a floor the twin never had.
      const FTransform Where(FRotator::ZeroRotator,
                             FVector(Centre.X, Centre.Y, Summary.Bounds.Min.Z - 200.0),
                             FVector(20000.0, 20000.0, 1.0)); // the plane is 1 m: 20 km across
      Ground = World->SpawnActorDeferred<AStaticMeshActor>(
          AStaticMeshActor::StaticClass(), Where, nullptr, nullptr,
          ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
      if (Ground != nullptr)
      {
        Ground->SetMobility(EComponentMobility::Movable);
        UStaticMeshComponent *Comp = Ground->GetStaticMeshComponent();
        Comp->SetStaticMesh(Plane);
        Comp->SetCollisionProfileName(TEXT("NoCollision"));
        Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Comp->SetCollisionResponseToAllChannels(ECR_Ignore);
        Comp->SetGenerateOverlapEvents(false);
        Comp->SetRenderCustomDepth(false);
        if (UMaterialInterface *Earth = LoadObject<UMaterialInterface>(
                nullptr, TEXT("/Engine/EngineMaterials/DefaultMaterial.DefaultMaterial")))
        {
          Comp->SetMaterial(0, Earth);
        }
        Ground->FinishSpawning(Where);
      }
    }
  }

  LoadVisualLayer(World, Root, BaseDir, Frame, OriginCm, TileActors, Ground, Summary);

  Summary.Seconds = FPlatformTime::Seconds() - Started;
  const FVector Min = Summary.Bounds.Min, Max = Summary.Bounds.Max;
  // Display, not Log: the suite reads the sim through -stdout, which drops
  // anything below Display.
  UE_LOG(LogReconTwin, Display,
         TEXT("ReconTwin: %s -> %d tiles, %lld vertices, %lld triangles, origin (%.0f %.0f %.0f) cm, "
              "bounds [%.1f %.1f %.1f]..[%.1f %.1f %.1f] cm, %.1f s"),
         *ManifestPath, Summary.Tiles, Summary.Vertices, Summary.Triangles,
         OriginCm.X, OriginCm.Y, OriginCm.Z, Min.X, Min.Y, Min.Z, Max.X, Max.Y, Max.Z,
         Summary.Seconds);
  return true;
}

bool FReconTwinLoader::LoadFromCommandLine(UWorld *World)
{
  FString Manifest;
  if (!FParse::Value(FCommandLine::Get(), TEXT("ReconTwin="), Manifest))
  {
    return false;
  }
  FVector OriginCm = FVector::ZeroVector;
  FString Offset;
  // FParse::Value stops at a comma unless told otherwise, and "2000,0,200"
  // would silently become "2000".
  if (FParse::Value(FCommandLine::Get(), TEXT("ReconTwinOffset="), Offset,
                    /*bShouldStopOnSeparator=*/false))
  {
    TArray<FString> Parts;
    Offset.ParseIntoArray(Parts, TEXT(","));
    if (Parts.Num() != 3)
    {
      UE_LOG(LogReconTwin, Error, TEXT("ReconTwin: -ReconTwinOffset wants x,y,z metres, got '%s'"), *Offset);
      return false;
    }
    OriginCm = FVector(FCString::Atod(*Parts[0]), FCString::Atod(*Parts[1]),
                       FCString::Atod(*Parts[2])) * 100.0;
  }
  FReconTwinSummary Summary;
  return Load(World, Manifest, OriginCm, Summary);
}
