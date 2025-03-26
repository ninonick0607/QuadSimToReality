using UnrealBuildTool;
using System.IO;

public class QuadSimToReality : ModuleRules
{
    public QuadSimToReality(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput",
            "ChaosVehicles", "PhysicsCore", "RenderCore", "RHI",
            "Sockets", "Networking", "ImGui", "Slate", "SlateCore", 
            "UMG", "Json", "JsonUtilities",
            "rclUE"  // Added rclUE as a dependency
        });
        
        PrivateDependencyModuleNames.AddRange(new string[] { });
        
        // Third-party library paths
        string ThirdPartyPath = Path.Combine(ModuleDirectory, "../../ThirdParty/");
        string ZeroMQPath = Path.Combine(ThirdPartyPath, "ZeroMQ");
        string ZeroMQIncludePath = Path.Combine(ZeroMQPath, "include");
        string CPPZMQIncludePath = Path.Combine(ThirdPartyPath, "cppzmq/include");

        // Include paths for all platforms
        PublicIncludePaths.AddRange(new string[] {
            ZeroMQIncludePath,
            CPPZMQIncludePath,
        });
        
        // rclUE include paths
        // The PluginDirectory variable lets us locate the rclUE plugin regardless of its location
        string PluginDirectory = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../Plugins"));
        string RclUEPath = Path.Combine(PluginDirectory, "rclUE");
        string ROS2IncludePath = Path.Combine(RclUEPath, "ThirdParty/ros/include");
        
        PublicIncludePaths.AddRange(new string[] {
            Path.Combine(RclUEPath, "Source/rclUE/Public"),
            ROS2IncludePath,
            // Common ROS2 message types include paths
            Path.Combine(ROS2IncludePath, "rcl"),
            Path.Combine(ROS2IncludePath, "rcl_interfaces"),
            Path.Combine(ROS2IncludePath, "std_msgs")
        });

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // Windows-specific configuration
            string ZeroMQLibPath = Path.Combine(ZeroMQPath, "lib");
            PublicAdditionalLibraries.Add(Path.Combine(ZeroMQLibPath, "libzmq-v143-mt-4_3_6.lib"));
            PublicDelayLoadDLLs.Add("libzmq-v143-mt-4_3_6.dll");
            RuntimeDependencies.Add("$(BinaryOutputDir)/libzmq-v143-mt-4_3_6.dll", 
                Path.Combine(ZeroMQLibPath, "libzmq-v143-mt-4_3_6.dll"));
                
            // Note: rclUE doesn't officially support Windows yet, as mentioned in the documentation
            PublicDefinitions.Add("WITH_ROS=0");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            // Linux-specific configuration for ZeroMQ
            PublicSystemLibraryPaths.Add("/usr/lib/x86_64-linux-gnu");
            PublicSystemLibraries.Add("zmq");
            
            // ROS2 specific configuration for Linux
            PublicDefinitions.Add("WITH_ROS=1");
            
            // Make sure the required system libraries for ROS2 are available
            PublicSystemLibraries.AddRange(new string[] {
                "yaml",
                "spdlog"
            });
            
            // Set the ROS_VERSION definition
            PublicDefinitions.Add("ROS_VERSION=2");
            
            // If you need specific ROS2 Foxy version
            PublicDefinitions.Add("ROS_DISTRO=foxy");
        }

        bEnableExceptions = true;
        CppStandard = CppStandardVersion.Cpp20;
    }
}
