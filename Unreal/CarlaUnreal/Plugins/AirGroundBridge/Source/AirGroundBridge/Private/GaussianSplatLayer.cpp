#include "GaussianSplatLayer.h"

#include "CommonRenderResources.h"
#include "Async/ParallelFor.h"
#include "Engine/World.h"
#include "GPUSort.h"
#include "GlobalShader.h"
#include "HAL/IConsoleManager.h"
#include "Math/Float16.h"
#include "Misc/CommandLine.h"
#include "HAL/PlatformFileManager.h"
#include "GenericPlatform/GenericPlatformFile.h"
#include "Misc/FileHelper.h"
#include "PipelineStateCache.h"
#include "PixelShaderUtils.h"
#include "PostProcess/PostProcessInputs.h"
#include "RHIGPUReadback.h"
#include "RHIStaticStates.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "SceneRendering.h" // FViewInfo: the render-resolution rect and the shader map
#include "SceneTexturesConfig.h"
#include "SceneView.h"
#include "SceneViewExtension.h"
#include "ShaderParameterStruct.h"

#include <atomic>

DEFINE_LOG_CATEGORY_STATIC(LogGaussianSplat, Log, All);

static TAutoConsoleVariable<int32> CVarSplatEnable(
    TEXT("ag.Splat.Enable"), 1,
    TEXT("Draw the ReconTwin Gaussian splat layer (0 = collision mesh only)."),
    ECVF_RenderThreadSafe);

// LOD selection. A node of the scan's tree is drawn when its typical splat
// would cover at most LodPixels on screen; otherwise its children are
// visited. Budget caps the splats sorted per view; when the pick exceeds it
// the pixel threshold is raised until it fits.
static TAutoConsoleVariable<float> CVarSplatLodPixels(
    TEXT("ag.Splat.LodPixels"), 2.0f,
    TEXT("Projected splat size (px) above which a finer LOD level is used."),
    ECVF_RenderThreadSafe);

static TAutoConsoleVariable<int32> CVarSplatBudget(
    TEXT("ag.Splat.Budget"), 8000000,
    TEXT("Max splats sorted and drawn per view (sort buffers are sized for it at load)."),
    ECVF_RenderThreadSafe);

static TAutoConsoleVariable<int32> CVarSplatLogLod(
    TEXT("ag.Splat.LogLod"), 0,
    TEXT("Log every view's LOD pick (nodes, splats, threshold)."),
    ECVF_RenderThreadSafe);

static TAutoConsoleVariable<int32> CVarSplatWriteDepth(
    TEXT("ag.Splat.WriteDepth"), 1,
    TEXT("In views where the collision mesh is hidden, splats also write depth (0 = colour only everywhere)."),
    ECVF_RenderThreadSafe);

// Diagnostic: read the sorted keys back once and count out-of-order pairs.
// The engine's own sort test stops at 2^20 keys; this layer sorts millions.
static TAutoConsoleVariable<int32> CVarSplatVerifySort(
    TEXT("ag.Splat.VerifySort"), 0,
    TEXT("1 = read the next view's sorted keys back and log how many are out of order."),
    ECVF_RenderThreadSafe);

static TAutoConsoleVariable<float> CVarSplatIntensity(
    TEXT("ag.Splat.Intensity"), 1.0f,
    TEXT("Linear multiplier on splat colour before the tonemapper."),
    ECVF_RenderThreadSafe);

// The scan's colours are display values, 0..1; the scene is in physical
// units and the cameras expose for it. A white in the photo is a white
// surface under the twin's sun: ReconTwin lights the level with 75000 lux,
// a white diffuse surface then has 75000/pi = 23900 cd/m2. Left at 1 the
// splats are black next to the sunlit ground.
// 23900 would be the physical figure; 7000 is what matches the CPU reference's
// tonal level through CARLA's default camera (test view: mean luminance 0.51
// vs 0.48 at 8000; display-space blend, correlation 0.95), the camera's fixed EV range being the reason.
static TAutoConsoleVariable<float> CVarSplatWhiteNits(
    TEXT("ag.Splat.WhiteNits"), 7000.0f,
    TEXT("Scene luminance (cd/m2) that a display-white splat maps to; match the level's sun."),
    ECVF_RenderThreadSafe);

namespace
{

// Mirrors FSplat in GaussianSplat.usf: 40 bytes, tightly packed.
struct FSplatRecordGPU
{
  FVector3f Position;
  uint32 Color;
  float Cov[6]; // xx xy xz yy yz zz
};
static_assert(sizeof(FSplatRecordGPU) == 40, "FSplatRecordGPU must match the HLSL layout");

/// One node of the scan's LOD tree, in UE world units.
struct FSplatNode
{
  uint32 First = 0, Count = 0;
  uint32 ChildFirst = 0, ChildCount = 0;
  uint32 Level = 0;
  float SplatSizeCm = 0.0f;
  FBox Bounds = FBox(ForceInit);
};

/// A range of records picked for one view, in the compacted index space the
/// key shader works in. Mirrors uint4 in the shader.
struct FSplatRange
{
  uint32 First, Count, Prefix, Pad;
};

// Records per GPU buffer: 2^26 x 40 bytes = 2.7 GB, under the RHI's 4 GB.
static constexpr uint32 RecordsPerBuffer = 1u << 26;
static constexpr uint32 MaxSplatBuffers = 4;

struct FSplatResources
{
  uint32 Count = 0;      // records on the GPU
  uint32 Capacity = 0;   // sort buffers: max selected per view
  uint32 EnvFirst = 0, EnvCount = 0;
  TArray<FSplatNode> Nodes;
  uint32 TopCount = 0;
  TArray<FBufferRHIRef> SplatBuffers;
  TArray<FShaderResourceViewRHIRef> SplatSRVs;
  FBufferRHIRef Keys[2], Values[2];
  FShaderResourceViewRHIRef KeySRV[2], ValueSRV[2];
  FUnorderedAccessViewRHIRef KeyUAV[2], ValueUAV[2];
};

// ---------------------------------------------------------------- shaders

BEGIN_SHADER_PARAMETER_STRUCT(FSplatCommonParameters, )
  SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)
  SHADER_PARAMETER_SRV(StructuredBuffer<FSplat>, Splats0)
  SHADER_PARAMETER_SRV(StructuredBuffer<FSplat>, Splats1)
  SHADER_PARAMETER_SRV(StructuredBuffer<FSplat>, Splats2)
  SHADER_PARAMETER_SRV(StructuredBuffer<FSplat>, Splats3)
  SHADER_PARAMETER(uint32, NumSplats)
  SHADER_PARAMETER(FVector3f, PreViewTranslation)
END_SHADER_PARAMETER_STRUCT()

class FSplatKeyGenCS : public FGlobalShader
{
public:
  DECLARE_GLOBAL_SHADER(FSplatKeyGenCS);
  SHADER_USE_PARAMETER_STRUCT(FSplatKeyGenCS, FGlobalShader);
  BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
    SHADER_PARAMETER_STRUCT_INCLUDE(FSplatCommonParameters, Common)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint4>, Ranges)
    SHADER_PARAMETER(uint32, NumRanges)
    SHADER_PARAMETER_UAV(RWBuffer<uint>, OutKeys)
    SHADER_PARAMETER_UAV(RWBuffer<uint>, OutValues)
  END_SHADER_PARAMETER_STRUCT()

  static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters &Parameters)
  {
    return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
  }
};

class FSplatVS : public FGlobalShader
{
public:
  DECLARE_GLOBAL_SHADER(FSplatVS);
  SHADER_USE_PARAMETER_STRUCT(FSplatVS, FGlobalShader);
  BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
    SHADER_PARAMETER_STRUCT_INCLUDE(FSplatCommonParameters, Common)
    SHADER_PARAMETER_SRV(Buffer<uint>, SortedIds)
  END_SHADER_PARAMETER_STRUCT()

  static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters &Parameters)
  {
    return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
  }
};

class FSplatPS : public FGlobalShader
{
public:
  DECLARE_GLOBAL_SHADER(FSplatPS);
  SHADER_USE_PARAMETER_STRUCT(FSplatPS, FGlobalShader);
  BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
  END_SHADER_PARAMETER_STRUCT()

  static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters &Parameters)
  {
    return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
  }
};

class FSplatCompositePS : public FGlobalShader
{
public:
  DECLARE_GLOBAL_SHADER(FSplatCompositePS);
  SHADER_USE_PARAMETER_STRUCT(FSplatCompositePS, FGlobalShader);
  BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float4>, SplatAccum)
    SHADER_PARAMETER(float, ColorScale)
    RENDER_TARGET_BINDING_SLOTS()
  END_SHADER_PARAMETER_STRUCT()

  static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters &Parameters)
  {
    return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
  }
};

BEGIN_SHADER_PARAMETER_STRUCT(FSplatDrawPassParameters, )
  SHADER_PARAMETER_STRUCT_INCLUDE(FSplatVS::FParameters, VS)
  SHADER_PARAMETER_STRUCT_INCLUDE(FSplatPS::FParameters, PS)
  RENDER_TARGET_BINDING_SLOTS()
END_SHADER_PARAMETER_STRUCT()

} // namespace

IMPLEMENT_GLOBAL_SHADER(FSplatKeyGenCS, "/Plugin/AirGroundBridge/Private/GaussianSplat.usf", "KeyGenCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FSplatVS, "/Plugin/AirGroundBridge/Private/GaussianSplat.usf", "MainVS", SF_Vertex);
IMPLEMENT_GLOBAL_SHADER(FSplatPS, "/Plugin/AirGroundBridge/Private/GaussianSplat.usf", "MainPS", SF_Pixel);
IMPLEMENT_GLOBAL_SHADER(FSplatCompositePS, "/Plugin/AirGroundBridge/Private/GaussianSplat.usf", "CompositePS", SF_Pixel);

// -------------------------------------------------------------- extension

namespace
{

class FGaussianSplatSceneViewExtension final : public FSceneViewExtensionBase
{
public:
  explicit FGaussianSplatSceneViewExtension(const FAutoRegister &AutoRegister)
      : FSceneViewExtensionBase(AutoRegister)
  {
  }

  void SetupViewFamily(FSceneViewFamily &InViewFamily) override {}
  void SetupView(FSceneViewFamily &InViewFamily, FSceneView &InView) override {}
  void BeginRenderViewFamily(FSceneViewFamily &InViewFamily) override {}

  bool IsActiveThisFrame_Internal(const FSceneViewExtensionContext &Context) const override
  {
    return NumSplats.load() > 0;
  }

  void PrePostProcessPass_RenderThread(FRDGBuilder &GraphBuilder, const FSceneView &InView,
                                       const FPostProcessingInputs &Inputs) override;

  /// Render-thread owned; replaced through render commands only.
  TSharedPtr<FSplatResources, ESPMode::ThreadSafe> Resources;
  FPrimitiveComponentId MeshProbe;
  bool bMeshHiddenInMainView = false;
  std::atomic<int32> NumSplats{0};

  int32 LodLogsLeft = 6; // first picks are logged regardless of ag.Splat.LogLod

  // Sort verification, render thread only.
  TUniquePtr<FRHIGPUBufferReadback> SortReadback;
  TUniquePtr<FRHIGPUBufferReadback> KeyReadback; // before the sort
  uint32 SortReadbackCount = 0;
  FVector3f VerifyViewOrigin = FVector3f::ZeroVector;
};

TSharedPtr<FGaussianSplatSceneViewExtension, ESPMode::ThreadSafe> GSplatExtension;

void FGaussianSplatSceneViewExtension::PrePostProcessPass_RenderThread(
    FRDGBuilder &GraphBuilder, const FSceneView &InView, const FPostProcessingInputs &Inputs)
{
  const TSharedPtr<FSplatResources, ESPMode::ThreadSafe> Res = Resources;
  if (!Res.IsValid() || Res->Count == 0 || !InView.bIsViewInfo || !Inputs.SceneTextures ||
      CVarSplatEnable.GetValueOnRenderThread() == 0)
  {
    return;
  }
  const FViewInfo &View = static_cast<const FViewInfo &>(InView);
  const FSceneTextureUniformParameters *SceneTextures = Inputs.SceneTextures->GetParameters().GetContents();
  FRDGTextureRef SceneColor = SceneTextures->SceneColorTexture;
  FRDGTextureRef SceneDepth = SceneTextures->SceneDepthTexture;
  if (SceneColor == nullptr || SceneDepth == nullptr)
  {
    return;
  }

  // LOD pick for this view: walk the tree from the top, descend while a
  // node's typical splat would be wider than LodPixels on screen, drop nodes
  // outside the frustum, and raise the threshold until the pick fits the
  // sort buffers. The sky sphere is always in.
  const FVector CameraPos = View.ViewMatrices.GetViewOrigin();
  const double FocalPx = View.ViewMatrices.GetProjectionMatrix().M[0][0] * 0.5 * View.ViewRect.Width();
  const bool bNoEnv = Res->EnvCount == 0;
  TArray<FSplatRange> Ranges;
  TArray<uint32> Stack;
  uint32 Selected = 0, NodesPicked = 0;
  float Tau = FMath::Max(0.25f, CVarSplatLodPixels.GetValueOnRenderThread());
  for (int32 Attempt = 0; Attempt < 10; ++Attempt)
  {
    Ranges.Reset();
    Stack.Reset();
    Selected = 0;
    NodesPicked = 0;
    for (uint32 i = 0; i < Res->TopCount; ++i) Stack.Add(i);
    while (Stack.Num() > 0)
    {
      const FSplatNode &Node = Res->Nodes[Stack.Pop(EAllowShrinking::No)];
      if (!View.ViewFrustum.IntersectBox(Node.Bounds.GetCenter(), Node.Bounds.GetExtent()))
      {
        continue;
      }
      const double Dist = FMath::Max(100.0, FMath::Sqrt(Node.Bounds.ComputeSquaredDistanceToPoint(CameraPos)));
      const double Pixels = Node.SplatSizeCm * FocalPx / Dist;
      if (Pixels > Tau && Node.ChildCount > 0)
      {
        for (uint32 c = 0; c < Node.ChildCount; ++c) Stack.Add(Node.ChildFirst + c);
        continue;
      }
      if (Node.Count > 0)
      {
        Ranges.Add({Node.First, Node.Count, Selected, 0});
        Selected += Node.Count;
        ++NodesPicked;
      }
    }
    if (!bNoEnv)
    {
      Ranges.Add({Res->EnvFirst, Res->EnvCount, Selected, 0});
      Selected += Res->EnvCount;
    }
    if (Selected <= Res->Capacity) break;
    Tau *= 1.3f;
  }
  while (Selected > Res->Capacity && Ranges.Num() > 0)
  {
    // Still over after ten tries: shed whole ranges from the end.
    Selected = Ranges.Last().Prefix;
    Ranges.Pop(EAllowShrinking::No);
  }
  if (Selected == 0 || Ranges.Num() == 0)
  {
    return;
  }
  if (LodLogsLeft > 0 || CVarSplatLogLod.GetValueOnRenderThread() != 0)
  {
    --LodLogsLeft;
    UE_LOG(LogGaussianSplat, Display,
           TEXT("GaussianSplat: view %s at (%.0f %.0f %.0f) cm, focal %.0f px: %u nodes, %u splats (+%u sky) at %.2f px, budget %u"),
           View.bIsSceneCapture ? TEXT("capture") : TEXT("main"), CameraPos.X, CameraPos.Y, CameraPos.Z, FocalPx,
           NodesPicked, Selected - (bNoEnv ? 0u : Res->EnvCount), bNoEnv ? 0u : Res->EnvCount, Tau, Res->Capacity);
  }
  const uint32 Count = Selected;
  FRDGBufferRef RangesBuffer = CreateStructuredBuffer(
      GraphBuilder, TEXT("ReconTwinSplatRanges"), sizeof(FSplatRange), Ranges.Num(), Ranges.GetData(),
      Ranges.Num() * sizeof(FSplatRange));

  if (SortReadback.IsValid() && SortReadback->IsReady() && KeyReadback->IsReady())
  {
    const uint32 *Raw = static_cast<const uint32 *>(KeyReadback->Lock(SortReadbackCount * sizeof(uint32)));
    uint64 RawCulled = 0;
    uint32 RawMin = 0xFFFFFFFFu, RawMax = 0;
    for (uint32 i = 0; i < SortReadbackCount; ++i)
    {
      if (Raw[i] == 0xFFFFFFFFu) { ++RawCulled; continue; }
      RawMin = FMath::Min(RawMin, Raw[i]);
      RawMax = FMath::Max(RawMax, Raw[i]);
    }
    UE_LOG(LogGaussianSplat, Display,
           TEXT("GaussianSplat: key check: view origin (%.0f %.0f %.0f) cm, raw keys[0..3] %08x %08x %08x %08x, "
                "%llu of %u culled, depth range %.1f..%.1f cm"),
           VerifyViewOrigin.X, VerifyViewOrigin.Y, VerifyViewOrigin.Z, Raw[0], Raw[1], Raw[2], Raw[3],
           RawCulled, SortReadbackCount, *reinterpret_cast<float *>(&RawMin), *reinterpret_cast<float *>(&RawMax));
    KeyReadback->Unlock();
    KeyReadback.Reset();

    const uint32 *Keys = static_cast<const uint32 *>(SortReadback->Lock(SortReadbackCount * sizeof(uint32)));
    uint64 Inversions = 0, Culled = 0;
    uint32 FirstBad = 0;
    for (uint32 i = 1; i < SortReadbackCount; ++i)
    {
      if (Keys[i] == 0xFFFFFFFFu) ++Culled;
      if (Keys[i] < Keys[i - 1])
      {
        if (Inversions == 0) FirstBad = i;
        ++Inversions;
      }
    }
    UE_LOG(LogGaussianSplat, Display,
           TEXT("GaussianSplat: sort check over %u keys: %llu out-of-order neighbours (first at %u: %08x after %08x), "
                "%llu culled, sorted[0..3] %08x %08x %08x %08x"),
           SortReadbackCount, Inversions, FirstBad, FirstBad ? Keys[FirstBad] : 0u, FirstBad ? Keys[FirstBad - 1] : 0u,
           Culled, Keys[0], Keys[1], Keys[2], Keys[3]);
    SortReadback->Unlock();
    SortReadback.Reset();
  }
  const bool bVerify = CVarSplatVerifySort.GetValueOnRenderThread() != 0 && !SortReadback.IsValid();
  if (bVerify)
  {
    SortReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("ReconTwinSplatSortCheck"));
    KeyReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("ReconTwinSplatKeyCheck"));
    SortReadbackCount = Count;
    VerifyViewOrigin = FVector3f(View.ViewMatrices.GetViewOrigin());
    CVarSplatVerifySort->Set(0, ECVF_SetByCode);
  }
  FRHIGPUBufferReadback *Readback = bVerify ? SortReadback.Get() : nullptr;
  FRHIGPUBufferReadback *RawReadback = bVerify ? KeyReadback.Get() : nullptr;

  FSplatCommonParameters Common;
  Common.View = View.ViewUniformBuffer;
  // Unused slots get the first buffer: every slot needs a valid view.
  Common.Splats0 = Res->SplatSRVs[0];
  Common.Splats1 = Res->SplatSRVs.Num() > 1 ? Res->SplatSRVs[1] : Res->SplatSRVs[0];
  Common.Splats2 = Res->SplatSRVs.Num() > 2 ? Res->SplatSRVs[2] : Res->SplatSRVs[0];
  Common.Splats3 = Res->SplatSRVs.Num() > 3 ? Res->SplatSRVs[3] : Res->SplatSRVs[0];
  Common.NumSplats = Count;
  Common.PreViewTranslation = FVector3f(View.ViewMatrices.GetPreViewTranslation());

  // 1. Depth keys for this view, then the engine's radix sort. Everything
  //    here lives outside RDG, so the transitions are explicit. The sort
  //    ping-pongs between the two buffers once per 4-bit pass; with every
  //    bit in the mask that is 8 passes and the result is back in buffer 0.
  const uint32 KeyMask = 0xFFFFFFFFu;
  const int32 ResultIndex = GetGPUSortPassCount(KeyMask) & 1;
  // The key pass must carry a parameter struct: RDG binds the static
  // uniform buffers (View among them) from it before the lambda runs. A
  // parameterless pass leaves View unbound and the compute shader reads
  // zeros for every matrix, which culled every splat (2026-10-14).
  TShaderMapRef<FSplatKeyGenCS> KeyGenCS(View.ShaderMap);
  FSplatKeyGenCS::FParameters *KeyParams = GraphBuilder.AllocParameters<FSplatKeyGenCS::FParameters>();
  KeyParams->Common = Common;
  KeyParams->Ranges = GraphBuilder.CreateSRV(RangesBuffer);
  KeyParams->NumRanges = Ranges.Num();
  KeyParams->OutKeys = Res->KeyUAV[0];
  KeyParams->OutValues = Res->ValueUAV[0];
  const ERHIFeatureLevel::Type FeatureLevel = View.GetFeatureLevel();
  GraphBuilder.AddPass(
      RDG_EVENT_NAME("ReconTwinSplatSort %u", Count), KeyParams, ERDGPassFlags::Compute | ERDGPassFlags::NeverCull,
      [Res, KeyGenCS, KeyParams, Count, KeyMask, ResultIndex, FeatureLevel, Readback, RawReadback](FRHICommandList &RHICmdList) {
        RHICmdList.Transition({
            FRHITransitionInfo(Res->KeyUAV[0], ERHIAccess::Unknown, ERHIAccess::UAVCompute),
            FRHITransitionInfo(Res->ValueUAV[0], ERHIAccess::Unknown, ERHIAccess::UAVCompute),
        });
        FComputeShaderUtils::Dispatch(RHICmdList, KeyGenCS, *KeyParams,
                                      FIntVector(FMath::DivideAndRoundUp(Count, 256u), 1, 1));
        if (RawReadback != nullptr)
        {
          RHICmdList.Transition(FRHITransitionInfo(Res->Keys[0], ERHIAccess::UAVCompute, ERHIAccess::CopySrc));
          RawReadback->EnqueueCopy(RHICmdList, Res->Keys[0], Count * sizeof(uint32));
          RHICmdList.Transition(FRHITransitionInfo(Res->Keys[0], ERHIAccess::CopySrc, ERHIAccess::UAVCompute));
        }
        FGPUSortBuffers SortBuffers;
        for (int32 i = 0; i < 2; ++i)
        {
          SortBuffers.RemoteKeySRVs[i] = Res->KeySRV[i];
          SortBuffers.RemoteKeyUAVs[i] = Res->KeyUAV[i];
          SortBuffers.RemoteValueSRVs[i] = Res->ValueSRV[i];
          SortBuffers.RemoteValueUAVs[i] = Res->ValueUAV[i];
        }
        const int32 Sorted = SortGPUBuffers(RHICmdList, SortBuffers, 0, KeyMask, Count, FeatureLevel);
        ensureMsgf(Sorted == ResultIndex, TEXT("GPU sort landed in buffer %d, expected %d"), Sorted, ResultIndex);
        RHICmdList.Transition(FRHITransitionInfo(Res->ValueUAV[ResultIndex], ERHIAccess::Unknown,
                                                 ERHIAccess::SRVGraphics));
        if (Readback != nullptr)
        {
          RHICmdList.Transition(FRHITransitionInfo(Res->Keys[ResultIndex], ERHIAccess::Unknown, ERHIAccess::CopySrc));
          Readback->EnqueueCopy(RHICmdList, Res->Keys[ResultIndex], Count * sizeof(uint32));
          RHICmdList.Transition(FRHITransitionInfo(Res->Keys[ResultIndex], ERHIAccess::CopySrc, ERHIAccess::UAVCompute));
        }
      });

  // 2. Draw, far to near, blended over the lit scene, depth-tested against
  //    everything else in it (the drone, props, the ground). Where the mesh
  //    is hidden the splats are the surfaces and write depth too; where it
  //    is visible (encoding cameras) they write colour only.
  const bool bMeshHidden = CVarSplatWriteDepth.GetValueOnRenderThread() != 0 &&
                           (View.bIsSceneCapture
                                ? (MeshProbe.IsValid() && View.HiddenPrimitives.Contains(MeshProbe))
                                : bMeshHiddenInMainView);
  // Splats blend among themselves in display space (the scan's own), into a
  // transient target; one composite pass then converts to linear and lays
  // the result over the scene by coverage.
  FRDGTextureRef Accum = GraphBuilder.CreateTexture(
      FRDGTextureDesc::Create2D(SceneColor->Desc.Extent, PF_FloatRGBA, FClearValueBinding::Transparent,
                                TexCreate_RenderTargetable | TexCreate_ShaderResource),
      TEXT("ReconTwinSplatAccum"));
  FSplatDrawPassParameters *Pass = GraphBuilder.AllocParameters<FSplatDrawPassParameters>();
  Pass->VS.Common = Common;
  Pass->VS.SortedIds = Res->ValueSRV[ResultIndex];
  Pass->RenderTargets[0] = FRenderTargetBinding(Accum, ERenderTargetLoadAction::EClear);
  Pass->RenderTargets.DepthStencil = FDepthStencilBinding(
      SceneDepth, ERenderTargetLoadAction::ELoad, ERenderTargetLoadAction::ENoAction,
      bMeshHidden ? FExclusiveDepthStencil::DepthWrite_StencilNop : FExclusiveDepthStencil::DepthRead_StencilNop);
  TShaderMapRef<FSplatVS> VertexShader(View.ShaderMap);
  TShaderMapRef<FSplatPS> PixelShader(View.ShaderMap);
  const FIntRect ViewRect = View.ViewRect;
  GraphBuilder.AddPass(
      RDG_EVENT_NAME("ReconTwinSplatDraw %u", Count), Pass, ERDGPassFlags::Raster,
      [Pass, VertexShader, PixelShader, ViewRect, Count, bMeshHidden](FRHICommandList &RHICmdList) {
        RHICmdList.SetViewport(ViewRect.Min.X, ViewRect.Min.Y, 0.0f, ViewRect.Max.X, ViewRect.Max.Y, 1.0f);
        FGraphicsPipelineStateInitializer PSOInit;
        RHICmdList.ApplyCachedRenderTargets(PSOInit);
        // Premultiplied over, colour and coverage alike.
        PSOInit.BlendState = TStaticBlendState<CW_RGBA, BO_Add, BF_One, BF_InverseSourceAlpha,
                                               BO_Add, BF_One, BF_InverseSourceAlpha>::GetRHI();
        PSOInit.RasterizerState = TStaticRasterizerState<FM_Solid, CM_None>::GetRHI();
        PSOInit.DepthStencilState = bMeshHidden
                                        ? TStaticDepthStencilState<true, CF_DepthNearOrEqual>::GetRHI()
                                        : TStaticDepthStencilState<false, CF_DepthNearOrEqual>::GetRHI();
        PSOInit.BoundShaderState.VertexDeclarationRHI = GEmptyVertexDeclaration.VertexDeclarationRHI;
        PSOInit.BoundShaderState.VertexShaderRHI = VertexShader.GetVertexShader();
        PSOInit.BoundShaderState.PixelShaderRHI = PixelShader.GetPixelShader();
        PSOInit.PrimitiveType = PT_TriangleList;
        SetGraphicsPipelineState(RHICmdList, PSOInit, 0);
        SetShaderParameters(RHICmdList, VertexShader, VertexShader.GetVertexShader(), Pass->VS);
        SetShaderParameters(RHICmdList, PixelShader, PixelShader.GetPixelShader(), Pass->PS);
        RHICmdList.SetStreamSource(0, nullptr, 0);
        RHICmdList.DrawPrimitive(0, 2 * Count, 1);
      });

  // 3. Composite: linear(display) x white x pre-exposure, over the scene.
  FSplatCompositePS::FParameters *Composite = GraphBuilder.AllocParameters<FSplatCompositePS::FParameters>();
  Composite->SplatAccum = Accum;
  Composite->ColorScale = CVarSplatWhiteNits.GetValueOnRenderThread() *
                          CVarSplatIntensity.GetValueOnRenderThread() * View.PreExposure;
  Composite->RenderTargets[0] = FRenderTargetBinding(SceneColor, ERenderTargetLoadAction::ELoad);
  TShaderMapRef<FSplatCompositePS> CompositeShader(View.ShaderMap);
  FPixelShaderUtils::AddFullscreenPass(
      GraphBuilder, View.ShaderMap, RDG_EVENT_NAME("ReconTwinSplatComposite"), CompositeShader, Composite, ViewRect,
      TStaticBlendState<CW_RGB, BO_Add, BF_One, BF_InverseSourceAlpha, BO_Add, BF_Zero, BF_One>::GetRHI());
}

// ------------------------------------------------------------------ pack

#pragma pack(push, 1)
struct FPackHeader
{
  char Magic[4];
  uint32 Version;
  uint32 Count;
  uint32 Flags;
  float BoundsMin[3];
  float BoundsMax[3];
  uint32 EnvCount;  // sky-sphere records, kept last in the file
  uint32 NodeCount; // v2: LOD tree table follows the header
  uint32 TopCount;  // v2: the first TopCount nodes are the tree's top level
  uint32 Reserved[3];
};
struct FPackNode
{
  uint32 First, Count, ChildFirst, ChildCount, Level;
  float SplatSize; // metres
  float BoundsMin[3], BoundsMax[3]; // scan grid frame, metres
};
struct FPackRecord
{
  float Position[3];
  uint16 Quat[4]; // w x y z, half
  uint16 Scale[3]; // half, metres
  uint8 Color[4];
  uint8 Pad[2];
};
#pragma pack(pop)
static_assert(sizeof(FPackHeader) == 64, "pack header is 64 bytes");
static_assert(sizeof(FPackNode) == 48, "pack node is 48 bytes");
static_assert(sizeof(FPackRecord) == 32, "pack record is 32 bytes");

float Half(uint16 Encoded)
{
  FFloat16 H;
  H.Encoded = Encoded;
  return H.GetFloat();
}

void EnsureExtension()
{
  if (!GSplatExtension.IsValid())
  {
    GSplatExtension = FSceneViewExtensions::NewExtension<FGaussianSplatSceneViewExtension>();
    FWorldDelegates::OnWorldCleanup.AddLambda(
        [](UWorld *, bool, bool) { FGaussianSplatLayer::Unload(); });
  }
}

} // namespace

bool FGaussianSplatLayer::Load(const FString &PackPath, const FSplatFrame &Frame, const FVector &OriginCm,
                               FGaussianSplatSummary &Summary, FString &Error)
{
  const double Started = FPlatformTime::Seconds();
  if (GUsingNullRHI)
  {
    // Headless: nothing will draw, and a multi-level pack is gigabytes.
    UE_LOG(LogGaussianSplat, Display, TEXT("GaussianSplat: null RHI, %s not loaded"), *PackPath);
    return true;
  }
  // Streamed: a multi-level pack is several GB, past TArray's 2 GB reach and
  // not worth holding twice. Header, node table, then records in chunks that
  // are converted as they arrive.
  TUniquePtr<IFileHandle> File(FPlatformFileManager::Get().GetPlatformFile().OpenRead(*PackPath));
  if (!File.IsValid())
  {
    Error = TEXT("cannot open file");
    return false;
  }
  const int64 FileSize = File->Size();
  if (FileSize < (int64)sizeof(FPackHeader))
  {
    Error = TEXT("shorter than a header");
    return false;
  }
  FPackHeader Header;
  if (!File->Read(reinterpret_cast<uint8 *>(&Header), sizeof(Header)))
  {
    Error = TEXT("cannot read header");
    return false;
  }
  if (FMemory::Memcmp(Header.Magic, "AGSP", 4) != 0)
  {
    Error = TEXT("not an AGSP pack");
    return false;
  }
  if (Header.Version != 1 && Header.Version != 2)
  {
    Error = FString::Printf(TEXT("pack version %u, this build reads 1 and 2"), Header.Version);
    return false;
  }
  const uint32 NodeCount = Header.Version >= 2 ? Header.NodeCount : 0;
  const int64 Expected = (int64)sizeof(FPackHeader) + (int64)NodeCount * sizeof(FPackNode) +
                         (int64)Header.Count * sizeof(FPackRecord);
  if (FileSize != Expected)
  {
    Error = FString::Printf(TEXT("%lld bytes, header promises %lld"), FileSize, Expected);
    return false;
  }
  Summary.InFile = (int32)Header.Count;
  // The scan's sky sphere is the last EnvCount records. Dropping it leaves
  // the level's own sky and ground visible around the site.
  uint32 Count = Header.Count;
  uint32 EnvCount = Header.EnvCount;
  if (EnvCount > Count) EnvCount = 0;
  if (FParse::Param(FCommandLine::Get(), TEXT("ReconTwinSplatsNoEnv")))
  {
    Count -= EnvCount;
    EnvCount = 0;
  }
  const double (&M)[3][3] = Frame.M;
  auto ToUnreal = [&M, &OriginCm](double X, double Y, double Z) {
    return FVector(M[0][0] * X + M[0][1] * Y + M[0][2] * Z + OriginCm.X,
                   M[1][0] * X + M[1][1] * Y + M[1][2] * Z + OriginCm.Y,
                   M[2][0] * X + M[2][1] * Y + M[2][2] * Z + OriginCm.Z);
  };
  auto BoxToUnreal = [&ToUnreal](const float *Lo, const float *Hi) {
    FBox Box(ForceInit);
    for (int32 k = 0; k < 8; ++k)
    {
      Box += ToUnreal((k & 1) ? Hi[0] : Lo[0], (k & 2) ? Hi[1] : Lo[1], (k & 4) ? Hi[2] : Lo[2]);
    }
    return Box;
  };

  // The LOD tree; a v1 pack is one node holding everything.
  TArray<FSplatNode> Nodes;
  uint32 TopCount = 1;
  const uint32 SceneCount = Count - EnvCount;
  if (NodeCount > 0)
  {
    TArray<FPackNode> InNodes;
    InNodes.SetNumUninitialized(NodeCount);
    if (!File->Read(reinterpret_cast<uint8 *>(InNodes.GetData()), (int64)NodeCount * sizeof(FPackNode)))
    {
      Error = TEXT("cannot read the node table");
      return false;
    }
    Nodes.SetNum(NodeCount);
    for (uint32 i = 0; i < NodeCount; ++i)
    {
      const FPackNode &N = InNodes[i];
      if ((uint64)N.First + N.Count > SceneCount || (uint64)N.ChildFirst + N.ChildCount > NodeCount)
      {
        Error = FString::Printf(TEXT("node %u points outside the pack"), i);
        return false;
      }
      Nodes[i].First = N.First;
      Nodes[i].Count = N.Count;
      Nodes[i].ChildFirst = N.ChildFirst;
      Nodes[i].ChildCount = N.ChildCount;
      Nodes[i].Level = N.Level;
      // Grid metres -> cm; the frame's scale is within 0.1 % of 100.
      Nodes[i].SplatSizeCm = N.SplatSize * 100.0f;
      Nodes[i].Bounds = BoxToUnreal(N.BoundsMin, N.BoundsMax);
    }
    TopCount = FMath::Clamp(Header.TopCount, 1u, NodeCount);
  }
  else
  {
    FSplatNode &Root = Nodes.AddDefaulted_GetRef();
    Root.Count = SceneCount;
    Root.Bounds = BoxToUnreal(Header.BoundsMin, Header.BoundsMax);
  }
  Summary.Bounds = BoxToUnreal(Header.BoundsMin, Header.BoundsMax);

  // Scan frame -> UE: position p' = M p + origin; covariance S' = (M R S)(M R S)^T.
  TArray<FSplatRecordGPU> Records;
  Records.SetNumUninitialized(Count);
  TArray<FPackRecord> Chunk;
  const uint32 ChunkRecords = 1u << 20; // 32 MB of file per read
  Chunk.SetNumUninitialized(ChunkRecords);
  for (uint32 Done = 0; Done < Count; Done += ChunkRecords)
  {
    const uint32 Num = FMath::Min(ChunkRecords, Count - Done);
    if (!File->Read(reinterpret_cast<uint8 *>(Chunk.GetData()), (int64)Num * sizeof(FPackRecord)))
    {
      Error = FString::Printf(TEXT("read failed at record %u"), Done);
      return false;
    }
    const FPackRecord *In = Chunk.GetData();
    FSplatRecordGPU *OutBase = Records.GetData() + Done;
    ParallelFor(Num, [&](int32 Index) {
      const uint32 i = (uint32)Index;
      const FPackRecord &R = In[i];
    const double P[3] = {R.Position[0], R.Position[1], R.Position[2]};
    double Q[4] = {Half(R.Quat[0]), Half(R.Quat[1]), Half(R.Quat[2]), Half(R.Quat[3])};
    const double Norm = FMath::Sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
    if (Norm > 0.0)
    {
      for (double &V : Q) V /= Norm;
    }
    else
    {
      Q[0] = 1.0;
    }
    const double W = Q[0], X = Q[1], Y = Q[2], Z = Q[3];
    const double S[3] = {Half(R.Scale[0]), Half(R.Scale[1]), Half(R.Scale[2])};
    const double Rot[3][3] = {
        {1.0 - 2.0 * (Y * Y + Z * Z), 2.0 * (X * Y - W * Z), 2.0 * (X * Z + W * Y)},
        {2.0 * (X * Y + W * Z), 1.0 - 2.0 * (X * X + Z * Z), 2.0 * (Y * Z - W * X)},
        {2.0 * (X * Z - W * Y), 2.0 * (Y * Z + W * X), 1.0 - 2.0 * (X * X + Y * Y)}};
    // A = M * R * diag(S)
    double A[3][3];
    for (int32 r = 0; r < 3; ++r)
    {
      for (int32 c = 0; c < 3; ++c)
      {
        A[r][c] = (M[r][0] * Rot[0][c] + M[r][1] * Rot[1][c] + M[r][2] * Rot[2][c]) * S[c];
      }
    }
    auto Cov = [&A](int32 r, int32 c) { return A[r][0] * A[c][0] + A[r][1] * A[c][1] + A[r][2] * A[c][2]; };
    FSplatRecordGPU &Out = OutBase[i];
    Out.Position = FVector3f(
        (float)(M[0][0] * P[0] + M[0][1] * P[1] + M[0][2] * P[2] + OriginCm.X),
        (float)(M[1][0] * P[0] + M[1][1] * P[1] + M[1][2] * P[2] + OriginCm.Y),
        (float)(M[2][0] * P[0] + M[2][1] * P[1] + M[2][2] * P[2] + OriginCm.Z));
    Out.Color = (uint32)R.Color[0] | ((uint32)R.Color[1] << 8) | ((uint32)R.Color[2] << 16) |
                ((uint32)R.Color[3] << 24);
    Out.Cov[0] = (float)Cov(0, 0);
    Out.Cov[1] = (float)Cov(0, 1);
    Out.Cov[2] = (float)Cov(0, 2);
    Out.Cov[3] = (float)Cov(1, 1);
    Out.Cov[4] = (float)Cov(1, 2);
    Out.Cov[5] = (float)Cov(2, 2);
    });
  }
  File.Reset();
  Summary.Loaded = Records.Num();
  Summary.Nodes = Nodes.Num();
  if (Count > RecordsPerBuffer * MaxSplatBuffers)
  {
    Error = FString::Printf(TEXT("%u records, this build holds at most %u"), Count, RecordsPerBuffer * MaxSplatBuffers);
    return false;
  }

  // Sort buffers are sized for the per-view budget, never more than the pack.
  int32 Budget = CVarSplatBudget.GetValueOnGameThread();
  FParse::Value(FCommandLine::Get(), TEXT("ReconTwinSplatBudget="), Budget);
  const uint32 Capacity = FMath::Clamp((uint32)FMath::Max(Budget, 1024), 1024u, Count);
  Summary.Capacity = Capacity;

  EnsureExtension();
  TSharedPtr<FGaussianSplatSceneViewExtension, ESPMode::ThreadSafe> Ext = GSplatExtension;
  ENQUEUE_RENDER_COMMAND(ReconTwinSplatUpload)(
      [Ext, Records = MoveTemp(Records), Nodes = MoveTemp(Nodes), TopCount, EnvCount, Capacity](
          FRHICommandListImmediate &RHICmdList) mutable {
        TSharedPtr<FSplatResources, ESPMode::ThreadSafe> Res = MakeShared<FSplatResources, ESPMode::ThreadSafe>();
        Res->Count = Records.Num();
        Res->Capacity = Capacity;
        Res->EnvCount = EnvCount;
        Res->EnvFirst = Records.Num() - EnvCount;
        Res->Nodes = MoveTemp(Nodes);
        Res->TopCount = TopCount;
        for (uint32 First = 0; First < (uint32)Records.Num(); First += RecordsPerBuffer)
        {
          const uint32 Num = FMath::Min(RecordsPerBuffer, (uint32)Records.Num() - First);
          const uint32 SplatBytes = Num * sizeof(FSplatRecordGPU);
          const FRHIBufferCreateDesc Desc =
              FRHIBufferCreateDesc::CreateStructured(TEXT("ReconTwinSplats"), SplatBytes, sizeof(FSplatRecordGPU))
                  .AddUsage(EBufferUsageFlags::Static | EBufferUsageFlags::ShaderResource)
                  .SetInitActionInitializer()
                  .DetermineInitialState();
          FRHIBufferInitializer Initializer = RHICmdList.CreateBufferInitializer(Desc);
          Initializer.WriteData(Records.GetData() + First, SplatBytes);
          FBufferRHIRef Buffer = Initializer.Finalize();
          Res->SplatSRVs.Add(RHICmdList.CreateShaderResourceView(
              Buffer, FRHIViewDesc::CreateBufferSRV()
                          .SetType(FRHIViewDesc::EBufferType::Structured)
                          .SetStride(sizeof(FSplatRecordGPU))));
          Res->SplatBuffers.Add(MoveTemp(Buffer));
        }
        const uint32 KeyBytes = Capacity * sizeof(uint32);
        for (int32 i = 0; i < 2; ++i)
        {
          const TCHAR *Names[2][2] = {{TEXT("ReconTwinSplatKeys0"), TEXT("ReconTwinSplatKeys1")},
                                      {TEXT("ReconTwinSplatValues0"), TEXT("ReconTwinSplatValues1")}};
          FBufferRHIRef *Buffers[2] = {&Res->Keys[i], &Res->Values[i]};
          FShaderResourceViewRHIRef *SRVs[2] = {&Res->KeySRV[i], &Res->ValueSRV[i]};
          FUnorderedAccessViewRHIRef *UAVs[2] = {&Res->KeyUAV[i], &Res->ValueUAV[i]};
          for (int32 k = 0; k < 2; ++k)
          {
            const FRHIBufferCreateDesc Desc =
                FRHIBufferCreateDesc::CreateVertex(Names[k][i], KeyBytes)
                    .AddUsage(EBufferUsageFlags::Static | EBufferUsageFlags::ShaderResource |
                              EBufferUsageFlags::UnorderedAccess)
                    .SetInitActionNone()
                    .SetInitialState(ERHIAccess::UAVCompute);
            *Buffers[k] = RHICmdList.CreateBuffer(Desc);
            *SRVs[k] = RHICmdList.CreateShaderResourceView(
                *Buffers[k], FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R32_UINT));
            *UAVs[k] = RHICmdList.CreateUnorderedAccessView(
                *Buffers[k], FRHIViewDesc::CreateBufferUAV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R32_UINT));
          }
        }
        Ext->Resources = Res;
        Ext->NumSplats.store(Res->Count);
      });

  Summary.Seconds = FPlatformTime::Seconds() - Started;
  return true;
}

void FGaussianSplatLayer::Unload()
{
  if (!GSplatExtension.IsValid())
  {
    return;
  }
  TSharedPtr<FGaussianSplatSceneViewExtension, ESPMode::ThreadSafe> Ext = GSplatExtension;
  Ext->NumSplats.store(0);
  ENQUEUE_RENDER_COMMAND(ReconTwinSplatUnload)([Ext](FRHICommandListImmediate &) { Ext->Resources.Reset(); });
}

void FGaussianSplatLayer::SetMeshProbe(FPrimitiveComponentId TilePrimitive, bool bHiddenFromMainView)
{
  EnsureExtension();
  TSharedPtr<FGaussianSplatSceneViewExtension, ESPMode::ThreadSafe> Ext = GSplatExtension;
  ENQUEUE_RENDER_COMMAND(ReconTwinSplatProbe)([Ext, TilePrimitive, bHiddenFromMainView](FRHICommandListImmediate &) {
    Ext->MeshProbe = TilePrimitive;
    Ext->bMeshHiddenInMainView = bHiddenFromMainView;
  });
}

int32 FGaussianSplatLayer::NumLoaded()
{
  return GSplatExtension.IsValid() ? GSplatExtension->NumSplats.load() : 0;
}
