// Source/QuadSimToReality/QuadSimToReality.Build.cs
using UnrealBuildTool;

public class QuadSimToReality : ModuleRules
{
    public QuadSimToReality(ReadOnlyTargetRules Target) : base(Target)
    {
        // Use explicit/shared PCHs
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        // We only need the basic engine modules here; all your simulation
        // code now lives in the plugin.
        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "InputCore"
        });

        PrivateDependencyModuleNames.AddRange(new string[] { });
    }
}
