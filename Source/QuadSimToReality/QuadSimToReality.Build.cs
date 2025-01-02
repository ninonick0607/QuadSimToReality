using UnrealBuildTool;
using System.IO;

public class QuadSimToReality : ModuleRules
{
    public QuadSimToReality(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        // Add public dependency module names
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core",
            "CoreUObject",
            "Engine",
            "InputCore",
            "EnhancedInput",
            "ChaosVehicles",
            "PhysicsCore",
            "RenderCore",
            "RHI",
            "Sockets",
            "Networking",
            "ImGui",
            "Slate",
            "SlateCore",
            "UMG",
            "Json",
            "JsonUtilities",
            "WebBrowser",
        });

        PrivateDependencyModuleNames.AddRange(new string[] { "WebBrowserWidget", "WebBrowserWidget" });
        // Third-party library paths
        string ThirdPartyPath = Path.Combine(ModuleDirectory, "../../ThirdParty/");
        string ZeroMQPath = Path.Combine(ThirdPartyPath, "ZeroMQ");
        string ZeroMQLibPath = Path.Combine(ZeroMQPath, "lib");
        string ZeroMQIncludePath = Path.Combine(ZeroMQPath, "include");

        // Include paths
        PublicIncludePaths.AddRange(new string[] {
            ZeroMQIncludePath,
            Path.Combine(ThirdPartyPath, "cppzmq", "include"),
        });

        // Library paths and libraries
        PublicAdditionalLibraries.Add(Path.Combine(ZeroMQLibPath, "libzmq-v143-mt-4_3_6.lib"));

        // Delay-load the DLL
        PublicDelayLoadDLLs.Add("libzmq-v143-mt-4_3_6.dll");

        // Add the DLL as a runtime dependency
        RuntimeDependencies.Add("$(BinaryOutputDir)/libzmq-v143-mt-4_3_6.dll", Path.Combine(ZeroMQLibPath, "libzmq-v143-mt-4_3_6.dll"));
        bEnableExceptions = true;
        CppStandard = CppStandardVersion.Cpp20;    }
}