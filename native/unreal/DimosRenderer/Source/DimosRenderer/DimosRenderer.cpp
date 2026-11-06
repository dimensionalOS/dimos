#include "Modules/ModuleManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

class FDimosRendererModule : public FDefaultGameModuleImpl
{
public:
    virtual void StartupModule() override
    {
        AddShaderSourceDirectoryMapping(TEXT("/DimosRenderer"), FPaths::Combine(FPaths::ProjectDir(), TEXT("Shaders")));
    }
};

IMPLEMENT_PRIMARY_GAME_MODULE(FDimosRendererModule, DimosRenderer, "DimosRenderer");
