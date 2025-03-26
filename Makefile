# Makefile generated by MakefileGenerator.cs
# *DO NOT EDIT*

UNREALROOTPATH = /home/ninonick0426/Unreal

TARGETS = \
	QuadSimToReality-Android-DebugGame  \
	QuadSimToReality-Android-Development  \
	QuadSimToReality-Android-Shipping  \
	QuadSimToReality-Linux-DebugGame  \
	QuadSimToReality-Linux-Development  \
	QuadSimToReality-Linux-Shipping  \
	QuadSimToReality-LinuxArm64-DebugGame  \
	QuadSimToReality-LinuxArm64-Development  \
	QuadSimToReality-LinuxArm64-Shipping  \
	QuadSimToReality \
	QuadSimToRealityEditor-Linux-DebugGame  \
	QuadSimToRealityEditor-Linux-Development  \
	QuadSimToRealityEditor \
	LiveLinkHub-Linux-DebugGame  \
	LiveLinkHub-Linux-Development  \
	LiveLinkHub \
	DotNetPerforceLib \
	EventLoopUnitTests \
	UnrealEditor-Linux-DebugGame  \
	UnrealEditor-Linux-Development  \
	UnrealEditor \
	UnrealGame-Android-DebugGame  \
	UnrealGame-Android-Development  \
	UnrealGame-Android-Shipping  \
	UnrealGame-Linux-DebugGame  \
	UnrealGame-Linux-Development  \
	UnrealGame-Linux-Shipping  \
	UnrealGame-LinuxArm64-DebugGame  \
	UnrealGame-LinuxArm64-Development  \
	UnrealGame-LinuxArm64-Shipping  \
	UnrealGame\
	configure

BUILD = "$(UNREALROOTPATH)/Engine/Build/BatchFiles/RunUBT.sh"

all: StandardSet

RequiredTools: CrashReportClient-Linux-Shipping CrashReportClientEditor-Linux-Shipping ShaderCompileWorker UnrealLightmass EpicWebHelper-Linux-Shipping

StandardSet: RequiredTools UnrealFrontend QuadSimToRealityEditor UnrealInsights

DebugSet: RequiredTools UnrealFrontend-Linux-Debug QuadSimToRealityEditor-Linux-Debug


QuadSimToReality-Android-DebugGame:
	 $(BUILD) QuadSimToReality Android DebugGame  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality-Android-Development:
	 $(BUILD) QuadSimToReality Android Development  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality-Android-Shipping:
	 $(BUILD) QuadSimToReality Android Shipping  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality-Linux-DebugGame:
	 $(BUILD) QuadSimToReality Linux DebugGame  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality-Linux-Development:
	 $(BUILD) QuadSimToReality Linux Development  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality-Linux-Shipping:
	 $(BUILD) QuadSimToReality Linux Shipping  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality-LinuxArm64-DebugGame:
	 $(BUILD) QuadSimToReality LinuxArm64 DebugGame  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality-LinuxArm64-Development:
	 $(BUILD) QuadSimToReality LinuxArm64 Development  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality-LinuxArm64-Shipping:
	 $(BUILD) QuadSimToReality LinuxArm64 Shipping  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToReality: QuadSimToReality-Linux-Development

QuadSimToRealityEditor-Linux-DebugGame:
	 $(BUILD) QuadSimToRealityEditor Linux DebugGame  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToRealityEditor-Linux-Development:
	 $(BUILD) QuadSimToRealityEditor Linux Development  -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" $(ARGS)

QuadSimToRealityEditor: QuadSimToRealityEditor-Linux-Development

LiveLinkHub-Linux-DebugGame:
	 $(BUILD) LiveLinkHub Linux DebugGame  $(ARGS)

LiveLinkHub-Linux-Development:
	 $(BUILD) LiveLinkHub Linux Development  $(ARGS)

LiveLinkHub: LiveLinkHub-Linux-Development

DotNetPerforceLib: DotNetPerforceLib-Linux-Development

EventLoopUnitTests: EventLoopUnitTests-Linux-Development

UnrealEditor-Linux-DebugGame:
	 $(BUILD) UnrealEditor Linux DebugGame  $(ARGS)

UnrealEditor-Linux-Development:
	 $(BUILD) UnrealEditor Linux Development  $(ARGS)

UnrealEditor: UnrealEditor-Linux-Development

UnrealGame-Android-DebugGame:
	 $(BUILD) UnrealGame Android DebugGame  $(ARGS)

UnrealGame-Android-Development:
	 $(BUILD) UnrealGame Android Development  $(ARGS)

UnrealGame-Android-Shipping:
	 $(BUILD) UnrealGame Android Shipping  $(ARGS)

UnrealGame-Linux-DebugGame:
	 $(BUILD) UnrealGame Linux DebugGame  $(ARGS)

UnrealGame-Linux-Development:
	 $(BUILD) UnrealGame Linux Development  $(ARGS)

UnrealGame-Linux-Shipping:
	 $(BUILD) UnrealGame Linux Shipping  $(ARGS)

UnrealGame-LinuxArm64-DebugGame:
	 $(BUILD) UnrealGame LinuxArm64 DebugGame  $(ARGS)

UnrealGame-LinuxArm64-Development:
	 $(BUILD) UnrealGame LinuxArm64 Development  $(ARGS)

UnrealGame-LinuxArm64-Shipping:
	 $(BUILD) UnrealGame LinuxArm64 Shipping  $(ARGS)

UnrealGame: UnrealGame-Linux-Development

configure:
	$(BUILD) -ProjectFiles -Project="/home/ninonick0426/Desktop/QuadSimToReality/QuadSimToReality.uproject" -Game 

.PHONY: $(TARGETS)
