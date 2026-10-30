using UnrealBuildTool;

public class DimosRenderer : ModuleRules
{
    public DimosRenderer(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "InputCore", "Json", "JsonUtilities",
            "Sockets", "Networking", "ProceduralMeshComponent", "RenderCore", "RHI", "PhysicsCore"
        });
    }
}
