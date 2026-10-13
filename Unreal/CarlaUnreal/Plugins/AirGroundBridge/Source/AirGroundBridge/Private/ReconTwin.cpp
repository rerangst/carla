#include "ReconTwin.h"

#include "Dom/JsonObject.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "ProceduralMeshComponent.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#include "Carla/Game/Tagger.h"

DEFINE_LOG_CATEGORY_STATIC(LogReconTwin, Log, All);

namespace crp = carla::rpc;

// LCC2 is right-handed, metres, (x east, y north, z up) in its UTM zone. The
// twin's GPS goes through ProjectAirSim, whose NED->UE mapping is
// (north, east, -down) -> (X, Y, Z) (UnrealHelpers::ToFVectorTranslation), so
// north must land on +X and east on +Y: swap x and y, metres to cm.
// Swapping two axes is a reflection, which flips the sign of every cross
// product; UE's own face normal is (P1-P2)x(P0-P2), the negative of the
// right-handed rule. The two flips cancel, so the original index order is
// kept and normals come out facing outward.
static FVector ToUnreal(const FVector3f &P)
{
  return FVector(P.Y * 100.0, P.X * 100.0, P.Z * 100.0);
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

  const FString BaseDir = FPaths::GetPath(ManifestPath);
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
      Vertices[i] = ToUnreal(Ply.Positions[i]);
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
    Actor->Mesh->CreateMeshSection_LinearColor(
        0, Vertices, Ply.Indices, Normals, TArray<FVector2D>(),
        TArray<FLinearColor>(), TArray<FProcMeshTangent>(), /*bCreateCollision=*/true);

    // Spec §3 trap 2: one stencil serves both readers. PAS copies the root
    // component's CustomDepthStencilValue into collision_info.segmentation_id;
    // CARLA's tagger reads the same field for untagged components, and its
    // semantic camera takes the label from custom primitive data.
    Actor->Mesh->SetCustomDepthStencilValue(static_cast<int32>(Label));
    Actor->Mesh->SetRenderCustomDepth(true);
    ATagger::SetStencilValue(*Actor->Mesh, Actor->GetUniqueID(), Label, true);

    Summary.Tiles += 1;
    Summary.Vertices += Vertices.Num();
    Summary.Triangles += Ply.Indices.Num() / 3;
  }

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
