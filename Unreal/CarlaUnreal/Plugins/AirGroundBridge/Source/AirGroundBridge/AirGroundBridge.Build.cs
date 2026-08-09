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
                // Carla keeps these private, but its public headers
                // (CarlaEpisode -> ActorData -> WheelPhysicsControl) need them.
                "Chaos", "ChaosVehicles", "PhysicsCore" });

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

        PrivateIncludePaths.AddRange(Includes);
    }
}
