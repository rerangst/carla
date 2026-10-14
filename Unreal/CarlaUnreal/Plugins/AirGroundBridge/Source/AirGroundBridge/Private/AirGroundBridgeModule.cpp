#include "Interfaces/IPluginManager.h"
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
  }
};

IMPLEMENT_MODULE(FAirGroundBridgeModule, AirGroundBridge);
