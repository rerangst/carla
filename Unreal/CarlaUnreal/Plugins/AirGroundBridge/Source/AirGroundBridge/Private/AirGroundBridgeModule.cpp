#include "AgExec.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/CoreDelegates.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "ShaderCore.h"

class FAirGroundBridgeModule : public IModuleInterface
{
public:
  virtual void StartupModule() override
  {
    // The Gaussian splat shaders live in the plugin; the module loads at
    // PostConfigInit so the mapping exists before the global shader map is
    // compiled.
    const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("AirGroundBridge"));
    if (Plugin.IsValid())
    {
      AddShaderSourceDirectoryMapping(TEXT("/Plugin/AirGroundBridge"),
                                      FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders")));
    }
    // -AgExecPort=N: console lines over loopback UDP, once the engine can run them.
    FCoreDelegates::GetOnPostEngineInit().AddStatic(&AgExec::Start);
  }

  virtual void ShutdownModule() override { AgExec::Stop(); }
};

IMPLEMENT_MODULE(FAirGroundBridgeModule, AirGroundBridge);
