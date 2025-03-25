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
            "rclUE" 
        });
        
        PrivateDependencyModuleNames.AddRange(new string[] { });
        
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
            PublicDefinitions.Add("WITH_ROS=0");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            
            // ROS2 specific configuration for Linux
            PublicDefinitions.Add("WITH_ROS=1");
            
            // Set the ROS_VERSION definition
            PublicDefinitions.Add("ROS_VERSION=2");
            
            // If you need specific ROS2 Foxy version
            PublicDefinitions.Add("ROS_DISTRO=foxy");
        }

        bEnableExceptions = true;
        CppStandard = CppStandardVersion.Cpp20;
    }
}
