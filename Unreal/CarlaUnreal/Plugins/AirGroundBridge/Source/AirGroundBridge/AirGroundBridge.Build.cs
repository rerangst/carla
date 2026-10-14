// AirGroundBridge — combined CARLA + ProjectAirSim runtime module.
using UnrealBuildTool;
using System.IO;
using System.Collections.Generic;

public class AirGroundBridge : ModuleRules
{
    public AirGroundBridge(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        // PAS headers use std:: containers and throw; must match its module flags.
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(
            new string[] { "Core", "CoreUObject", "Engine" });

        PrivateDependencyModuleNames.AddRange(
            new string[] { "Carla", "ProjectAirSim", "RenderCore", "RHI",
                // ReconTwin: twin.jsonc manifest + runtime collision tiles.
                "Json", "ProceduralMeshComponent",
                // GaussianSplatLayer: a scene view extension with its own
                // global shaders (Renderer for FViewInfo and the post-process
                // inputs, Projects to find the plugin's Shaders/ directory).
                "Renderer", "Projects",
                // Carla keeps these private, but its public headers
                // (CarlaEpisode -> ActorData -> WheelPhysicsControl) need them.
                "Chaos", "ChaosVehicles", "PhysicsCore", "Foliage" });

        // ProjectAirSim only exports Source/ProjectAirSim/Public, but its public
        // UnrealSimLoader.h pulls headers from Private/ and from the SimLibs
        // packages, so consumers must mirror those include paths.
        string PasPlugin = Path.GetFullPath(
            Path.Combine(PluginDirectory, "..", "ProjectAirSim"));
        string PasModule = Path.Combine(PasPlugin, "Source", "ProjectAirSim");

        var SimLibs = new List<string> {
            "core_sim", "simserver", "physics", "rendering_scene",
            "multirotor_api", "rover_api", "mavlinkcom", "eigen",
            "assimp", "json", "nng"
        };

        var Includes = new List<string> { Path.Combine(PasModule, "Private") };
        foreach (var Lib in SimLibs)
            Includes.Add(Path.Combine(PasPlugin, "SimLibs", Lib, "include"));
        Includes.Add(Path.Combine(
            PasPlugin, "SimLibs", "shared_libs", "onnxruntime", "include"));

        string EngineDir = Path.GetFullPath(Target.RelativeEnginePath);
        Includes.Add(Path.Combine(EngineDir, "Source/Runtime/Renderer/Internal"));
        // FViewInfo (SceneRendering.h) is private to the renderer; the splat
        // pass needs its render-resolution rect and pre-exposure.
        Includes.Add(Path.Combine(EngineDir, "Source/Runtime/Renderer/Private"));

        PrivateIncludePaths.AddRange(Includes);
    }
}
