#include "UI/ImGuiUtil.h"
#include "imgui.h"
#include "implot.h"
#include "Pawns/QuadPawn.h"
#include "string"
#include "Controllers/QuadDroneController.h"
#include "Kismet/GameplayStatics.h"
#include "Core/DroneJSONConfig.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include <string>
#include "Misc/DateTime.h"
#include "Core/DroneManager.h"

UImGuiUtil::UImGuiUtil()
	: DronePawn(nullptr)
	, Controller(nullptr)
	, CumulativeTime(0.0f)
	, MaxPlotTime(10.0f)
	, bShowSettingsUI(false)
{
	const auto& Config = UDroneJSONConfig::Get().Config;
	PrimaryComponentTick.bCanEverTick = true;
	maxVelocityBound = Config.FlightParams.MaxVelocityBound;
	maxThrust = Config.FlightParams.MaxThrust;
	plotSwitch = false;

}

void UImGuiUtil::Initialize(AQuadPawn* InPawn, UQuadDroneController* InController)
{ 
	DronePawn = InPawn;
	Controller = InController;
}

void UImGuiUtil::BeginPlay()
{
    Super::BeginPlay();
}

void UImGuiUtil::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}


void UImGuiUtil::ImGuiHud(EFlightMode CurrentMode, TArray<float>& ThrustsVal,
                          float desiredRollAngle, float desiredPitchAngle,
                          const FRotator& currentRotation,
                          const FVector& waypoint, const FVector& currLoc,
                          const FVector& error,
                          const FVector& currentVelocity,
                          float maxVelocity,
                          float maxAngle,
                          float xOutput, float yOutput, float zOutput, float deltaTime)
{
    // Window setup
    ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 600), ImGuiCond_FirstUseEver);
	// Set up window position and size
	ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
 
    // Use Pawn's DroneID as identifier
    FString droneID = DronePawn ? DronePawn->DroneID : FString(TEXT("Unknown"));
 
	FString WindowName = FString::Printf(TEXT("Drone Controller##%s"), *droneID);
    ImGui::Begin(TCHAR_TO_UTF8(*WindowName), nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    // Swarm mode broadcast helper
    ADroneManager* Manager = ADroneManager::Get(GetWorld());
    bool bSwarm = Manager && Manager->IsSwarmMode();
    auto applyToControllers = [&](auto&& Func) {
        if (bSwarm && Manager) {
            TArray<AQuadPawn*> drones = Manager->GetDroneList();
            for (AQuadPawn* pawn : drones) {
                if (pawn && pawn->QuadController) {
                    Func(pawn->QuadController);
                }
            }
        } else {
            if (Controller) Func(Controller);
        }
    };


    if (ImGui::Button("Settings")) {
        bShowSettingsUI = !bShowSettingsUI;
    }
    if (bShowSettingsUI) {
        ImGui::End();
        // Settings Window
        if (ImGui::Begin("Settings", &bShowSettingsUI, ImGuiWindowFlags_AlwaysAutoResize)) {
            auto& Cfg = UDroneJSONConfig::Get().Config;
            // Flight Parameters
            if (ImGui::CollapsingHeader("Flight Parameters")) {
                ImGui::InputFloat("Max Velocity Bound", &Cfg.FlightParams.MaxVelocityBound);
                ImGui::InputFloat("Max Velocity", &Cfg.FlightParams.MaxVelocity);
                ImGui::InputFloat("Max Angle", &Cfg.FlightParams.MaxAngle);
                ImGui::InputFloat("Max PID Output", &Cfg.FlightParams.MaxPIDOutput);
                ImGui::InputFloat("Altitude Threshold", &Cfg.FlightParams.AltitudeThreshold);
                ImGui::InputFloat("Min Altitude Local", &Cfg.FlightParams.MinAltitudeLocal);
                ImGui::InputFloat("Acceptable Distance", &Cfg.FlightParams.AcceptableDistance);
            }
            // Controller Parameters
            if (ImGui::CollapsingHeader("Controller Parameters")) {
                ImGui::InputFloat("Altitude Rate", &Cfg.ControllerParams.AltitudeRate);
                ImGui::InputFloat("Yaw Rate", &Cfg.ControllerParams.YawRate);
                ImGui::InputFloat("Min Velocity For Yaw", &Cfg.ControllerParams.MinVelocityForYaw);
            }
            // Obstacle Parameters
            if (ImGui::CollapsingHeader("Obstacle Parameters")) {
                ImGui::InputFloat("Outer Boundary Size", &Cfg.ObstacleParams.OuterBoundarySize);
                ImGui::InputFloat("Inner Boundary Size", &Cfg.ObstacleParams.InnerBoundarySize);
                ImGui::InputFloat("Spawn Height", &Cfg.ObstacleParams.SpawnHeight);
            }
            if (ImGui::Button("Save Settings")) {
                if (UDroneJSONConfig::Get().SaveConfig()) {
                    UE_LOG(LogTemp, Log, TEXT("Config saved successfully."));
                } else {
                    UE_LOG(LogTemp, Error, TEXT("Failed to save config."));
                }
            }
        }
        ImGui::End();
        return;
    }
    ImGui::Text("Drone ID: %s", TCHAR_TO_UTF8(*droneID));

	FVector currentDesiredVelocity = Controller ? Controller->GetDesiredVelocity() : FVector::ZeroVector;

    // Top controls
    static bool bLocalManualMode = false;
    if (ImGui::Checkbox("Manual Thrust Mode", &bLocalManualMode))
        applyToControllers([&](UQuadDroneController* C){ C->SetManualThrustMode(bLocalManualMode); });
    ImGui::SameLine(200);
    static bool bDebugVis = false;
    if (ImGui::Checkbox("Debug Visuals", &bDebugVis))
        applyToControllers([&](UQuadDroneController* C){ C->SetDebugVisualsEnabled(bDebugVis); });
    ImGui::Separator();

    // Global sliders
    ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, maxVelocityBound);
    ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);
    applyToControllers([&](UQuadDroneController* C){ C->SetMaxVelocity(maxVelocity); });
    applyToControllers([&](UQuadDroneController* C){ C->SetDesiredAngle(maxAngle); });
    ImGui::Separator();

    // Main content depending on mode
    switch (CurrentMode)
    {
    case EFlightMode::None: break;
    case EFlightMode::AutoWaypoint:
        DisplayDesiredPositions();
        break;
    case EFlightMode::VelocityControl:
        DisplayDesiredVelocities(maxVelocity);
        break;
    case EFlightMode::JoyStickControl:
        // joy-stick UI if needed
        break;
    }

    // Thruster & state info
    DisplayThrust(ThrustsVal);
    ImGui::Separator();
    ImGui::Text("===== Current State & Feedback =====");
	ImGui::Text("======= Current State & Feedback =======");
	if (DronePawn && DronePawn->DroneBody)
	{
		float droneMass = DronePawn->GetMass();
		ImGui::Text("Drone Mass: %.2f kg", droneMass);
	}
	else
	{
		ImGui::Text("Drone Pawn or Drone Body is null!");
	}
	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Text("==== Attitude ====");
	ImGui::Text("Current: Roll: %.2f || Pitch: %.2f", currentRotation.Roll, currentRotation.Pitch);
	ImGui::Text("Desired: Roll: %.2f || Pitch: %.2f ", desiredRollAngle, desiredPitchAngle);
	ImGui::Text("==== Position ====");
	ImGui::Text("Current: %.1f, %.1f, %.1f ", currLoc.X, currLoc.Y, currLoc.Z);
	ImGui::Text("==== Velocity ====");
	ImGui::Text("Current: %.1f, %.1f, %.1f", currentVelocity.X, currentVelocity.Y, currentVelocity.Z);
	ImGui::Text("Desired: %.1f, %.1f, %.1f", currentDesiredVelocity.X, currentDesiredVelocity.Y, currentDesiredVelocity.Z);
       
    ImGui::Separator();

    // PID settings
    static bool syncXY = false, syncRP = false;
    DisplayPIDSettings(CurrentMode, "PID Settings", syncXY, syncRP);
    ImGui::Separator();

    // Actions
    ImGui::Checkbox("Enable Plots", &plotSwitch);
    ImGui::SameLine(); DisplayButtons();
    ImGui::Separator();

    ImGui::End();

    if (plotSwitch)
        RenderControlPlots(deltaTime, currentRotation, desiredRollAngle, desiredPitchAngle, maxAngle);

    DisplayPIDHistoryWindow();
}

void UImGuiUtil::RenderControlPlots(float deltaTime, const FRotator& currentRotation, float desiredRoll, float desiredPitch,float maxAngle)
{
    if (!Controller) return;

    // Get velocities using getters
    FVector currentLocalVelocity = Controller->GetCurrentLocalVelocity();
    FVector desiredVelocity = Controller->GetDesiredVelocity();

    CumulativeTime += deltaTime;
    TimeData.Add(CumulativeTime);

    // Add velocity data
    CurrentVelocityXData.Add(currentLocalVelocity.X);
    CurrentVelocityYData.Add(currentLocalVelocity.Y);
    DesiredVelocityXData.Add(desiredVelocity.X);
    DesiredVelocityYData.Add(desiredVelocity.Y);

    // Add angle data
    CurrentRollData.Add(currentRotation.Roll);
    DesiredRollData.Add(desiredRoll); // Use the passed desired angle
    CurrentPitchData.Add(currentRotation.Pitch);
    DesiredPitchData.Add(desiredPitch); // Use the passed desired angle


    // Pruning logic for ALL history arrays
    while (TimeData.Num() > 0 && (CumulativeTime - TimeData[0] > MaxPlotTime))
    {
        TimeData.RemoveAt(0);
        // Prune existing plots' data if RenderImPlot is still used elsewhere
        if (Thrust0Data.Num() > 0) Thrust0Data.RemoveAt(0);
        if (Thrust1Data.Num() > 0) Thrust1Data.RemoveAt(0);
        if (Thrust2Data.Num() > 0) Thrust2Data.RemoveAt(0);
        if (Thrust3Data.Num() > 0) Thrust3Data.RemoveAt(0);
        if (DesiredHeadingData.Num() > 0) DesiredHeadingData.RemoveAt(0);
        if (CurrentHeadingData.Num() > 0) CurrentHeadingData.RemoveAt(0);
        if (VectorErrorData.Num() > 0) VectorErrorData.RemoveAt(0);
        // Prune new plots' data
        if (CurrentVelocityXData.Num() > 0) CurrentVelocityXData.RemoveAt(0);
        if (CurrentVelocityYData.Num() > 0) CurrentVelocityYData.RemoveAt(0);
        if (CurrentVelocityZData.Num() > 0) CurrentVelocityZData.RemoveAt(0);
        if (DesiredVelocityXData.Num() > 0) DesiredVelocityXData.RemoveAt(0);
        if (DesiredVelocityYData.Num() > 0) DesiredVelocityYData.RemoveAt(0);
        if (DesiredVelocityZData.Num() > 0) DesiredVelocityZData.RemoveAt(0);
        if (CurrentRollData.Num() > 0) CurrentRollData.RemoveAt(0);
        if (DesiredRollData.Num() > 0) DesiredRollData.RemoveAt(0);
        if (CurrentPitchData.Num() > 0) CurrentPitchData.RemoveAt(0);
        if (DesiredPitchData.Num() > 0) DesiredPitchData.RemoveAt(0);
    }

     // Limit max data points (simpler than time-based for consistent array sizes)
    while (TimeData.Num() > MaxDataPoints)
    {
        TimeData.RemoveAt(0);
        if (Thrust0Data.Num() > 0) Thrust0Data.RemoveAt(0);
        if (Thrust1Data.Num() > 0) Thrust1Data.RemoveAt(0);
        if (Thrust2Data.Num() > 0) Thrust2Data.RemoveAt(0);
        if (Thrust3Data.Num() > 0) Thrust3Data.RemoveAt(0);
        if (DesiredHeadingData.Num() > 0) DesiredHeadingData.RemoveAt(0);
        if (CurrentHeadingData.Num() > 0) CurrentHeadingData.RemoveAt(0);
        if (VectorErrorData.Num() > 0) VectorErrorData.RemoveAt(0);
        if (CurrentVelocityXData.Num() > 0) CurrentVelocityXData.RemoveAt(0);
        if (CurrentVelocityYData.Num() > 0) CurrentVelocityYData.RemoveAt(0);
        if (DesiredVelocityXData.Num() > 0) DesiredVelocityXData.RemoveAt(0);
        if (DesiredVelocityYData.Num() > 0) DesiredVelocityYData.RemoveAt(0);
        if (CurrentRollData.Num() > 0) CurrentRollData.RemoveAt(0);
        if (DesiredRollData.Num() > 0) DesiredRollData.RemoveAt(0);
        if (CurrentPitchData.Num() > 0) CurrentPitchData.RemoveAt(0);
        if (DesiredPitchData.Num() > 0) DesiredPitchData.RemoveAt(0);
    }


    // Start a new ImGui window for these plots
    ImGui::SetNextWindowSize(ImVec2(600, 700), ImGuiCond_FirstUseEver); // Adjusted size for 3 plots
    ImGui::SetNextWindowPos(ImVec2(950, 10), ImGuiCond_FirstUseEver); // Position next to controller window

    ImGui::Begin("Control Plots");

    ImVec2 windowSize = ImGui::GetContentRegionAvail();
    // Allocate roughly equal height for three plots
    float plotHeight = (windowSize.y / 3.0f) - (ImGui::GetStyle().ItemSpacing.y * 2); // Account for spacing
    ImVec2 plotSize(windowSize.x, plotHeight);
    ImPlotFlags plotFlags = ImPlotFlags_None; // Or ImPlotFlags_NoLegend if preferred
    ImPlotAxisFlags axisFlags = ImPlotAxisFlags_None; // Or customize as needed

    int dataCount = TimeData.Num(); // Use the count from TimeData


    // Velocity Plot
    if (ImPlot::BeginPlot("Velocity (Local Frame)", plotSize, plotFlags))
    {
        ImPlot::SetupAxes("Time (s)", "Velocity (cm/s)", axisFlags, axisFlags);
        ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - MaxPlotTime, CumulativeTime, ImGuiCond_Always); // Keep X axis scrolling

        if (dataCount > 0)
        {
            // Current Velocities
            ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.f, 0.f, 1.f), 1.5f); // Red
            ImPlot::PlotLine("Current Vel X", TimeData.GetData(), CurrentVelocityXData.GetData(), dataCount);
            ImPlot::SetNextLineStyle(ImVec4(0.f, 1.0f, 0.f, 1.f), 1.5f); // Green
            ImPlot::PlotLine("Current Vel Y", TimeData.GetData(), CurrentVelocityYData.GetData(), dataCount);

            // Desired Velocities (dashed or different color)
             ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 1.0f); // Thinner lines for desired
            // ImPlot::PushStyleVar(ImPlotStyleVar_DashPatterns, { 10.f, 5.f }); // Example dash pattern

            ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.6f, 0.6f, 1.f)); // Light Red
            ImPlot::PlotLine("Desired Vel X", TimeData.GetData(), DesiredVelocityXData.GetData(), dataCount);
            ImPlot::SetNextLineStyle(ImVec4(0.6f, 1.0f, 0.6f, 1.f)); // Light Green
            ImPlot::PlotLine("Desired Vel Y", TimeData.GetData(), DesiredVelocityYData.GetData(), dataCount);

            //ImPlot::PopStyleVar(); // Pop dash pattern if used
             ImPlot::PopStyleVar(); // Pop line weight
        }

        ImPlot::EndPlot();
    }

    ImGui::Spacing(); // Add space between plots

    // Roll Plot
    if (ImPlot::BeginPlot("Roll Angle", plotSize, plotFlags))
    {
        ImPlot::SetupAxes("Time (s)", "Angle (degrees)", axisFlags, axisFlags);
        ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - MaxPlotTime, CumulativeTime, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -maxAngle-10, maxAngle+10, ImPlotCond_Once); // Set Y limits based on maxAngle

        if (dataCount > 0)
        {
            ImPlot::SetNextLineStyle(ImVec4(1.f, 0.f, 0.f, 1.f), 1.5f); // Red for current
            ImPlot::PlotLine("Current Roll", TimeData.GetData(), CurrentRollData.GetData(), dataCount);

            ImPlot::SetNextLineStyle(ImVec4(1.f, 0.6f, 0.6f, 1.f), 1.0f); // Lighter Red for desired
            ImPlot::PlotLine("Desired Roll", TimeData.GetData(), DesiredRollData.GetData(), dataCount);
        }
        ImPlot::EndPlot();
    }

    ImGui::Spacing();

    // Pitch Plot
    if (ImPlot::BeginPlot("Pitch Angle", plotSize, plotFlags))
    {
        ImPlot::SetupAxes("Time (s)", "Angle (degrees)", axisFlags, axisFlags);
        ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - MaxPlotTime, CumulativeTime, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -maxAngle-10, maxAngle+10, ImPlotCond_Once);

        if (dataCount > 0)
        {
            ImPlot::SetNextLineStyle(ImVec4(0.f, 1.f, 0.f, 1.f), 1.5f); // Green for current
            ImPlot::PlotLine("Current Pitch", TimeData.GetData(), CurrentPitchData.GetData(), dataCount);

            ImPlot::SetNextLineStyle(ImVec4(0.6f, 1.f, 0.6f, 1.f), 1.0f); // Lighter Green for desired
            ImPlot::PlotLine("Desired Pitch", TimeData.GetData(), DesiredPitchData.GetData(), dataCount);
        }
        ImPlot::EndPlot();
    }


    ImGui::End(); // End Control Plots window
}

void UImGuiUtil::DisplayThrust(TArray<float>& ThrustsNum) {

	ImGui::Separator();
	ImGui::Text("Thruster Power");

	static float AllThrustValue = 0.0f;
	if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, 0, maxThrust))
	{
		for (int i = 0; i < ThrustsNum.Num(); i++)
		{
			ThrustsNum[i] = AllThrustValue;
		}
	}

	ImGui::Separator();
	ImGui::Text("Thruster Power");

	static bool synchronizeDiagonal1 = false;
	static bool synchronizeDiagonal2 = false;
	ImGui::Checkbox("Synchronize Diagonal Motors FL & BR", &synchronizeDiagonal1);
	ImGui::Checkbox("Synchronize Diagonal Motors FR & BL", &synchronizeDiagonal2);

	ImGui::Separator();

	if (ThrustsNum.Num() >= 4)
	{
		ImGui::Text("Diagonal 1 Motors");

		// Push a unique ID to differentiate these widgets
		ImGui::PushID("Diag1");
		ImGui::Indent();
		if (synchronizeDiagonal1)
		{
			if (ImGui::SliderFloat("FL & BR Thrust", &ThrustsNum[0], 0, maxThrust))
			{
				ThrustsNum[3] = ThrustsNum[0];
			}
			ImGui::Text("Back Right (Synchronized): %.2f", ThrustsNum[3]);
		}
		else
		{
			// "Front Left" is effectively "Diag1/Front Left"
			ImGui::SliderFloat("Front Left", &ThrustsNum[0], 0, maxThrust);
			ImGui::SliderFloat("Back Right", &ThrustsNum[3], 0, maxThrust);
		}
		ImGui::Unindent();
		ImGui::PopID();  // pop "Diag1"

		// -------------------- Diagonal 2 --------------------
		ImGui::Text("Diagonal 2 Motors");

		ImGui::PushID("Diag2");
		ImGui::Indent();
		if (synchronizeDiagonal2)
		{
			if (ImGui::SliderFloat("FR & BL Thrust", &ThrustsNum[1], 0, maxThrust))
			{
				ThrustsNum[2] = ThrustsNum[1];
			}
			ImGui::Text("Back Left (Synchronized): %.2f", ThrustsNum[2]);
		}
		else
		{
			ImGui::SliderFloat("Front Right", &ThrustsNum[1], 0, maxThrust);
			ImGui::SliderFloat("Back Left", &ThrustsNum[2], 0, maxThrust);
		}
		ImGui::Unindent();
		ImGui::PopID(); // pop "Diag2"
	}

}

void UImGuiUtil::DisplayPIDSettings(EFlightMode Mode, const char* headerLabel, bool& synchronizeXYGains, bool& synchronizeGains)
{
	FFullPIDSet* PIDSet = Controller ? Controller->GetPIDSet(Mode) : nullptr;
	if (!PIDSet)
	{
		ImGui::Text("No PID Set found for this mode.");
		return;
	}

	// Helper lambda remains the same - used for non-synced controls and other axes
	auto DrawPIDGainControl = [](const char* label, float* value, float minValue, float maxValue)
		{
			float totalWidth = ImGui::GetContentRegionAvail().x;
			float inputWidth = 80.0f;
			// Adjust slider width calculation slightly if necessary
			float sliderWidth = totalWidth > (inputWidth + 20.0f) ? totalWidth - inputWidth - 20.0f : 100.0f;

			ImGui::PushItemWidth(sliderWidth);
			bool changed = ImGui::SliderFloat(label, value, minValue, maxValue);
			ImGui::PopItemWidth();

			ImGui::SameLine();

			ImGui::PushItemWidth(inputWidth);
			ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1));
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));
			std::string inputLabel = std::string("##Input_") + label; // Use unique ID prefix
			changed |= ImGui::InputFloat(inputLabel.c_str(), value, 0.0f, 0.0f, "%.3f");
			ImGui::PopStyleColor(2);
			ImGui::PopItemWidth();
			return changed; // Return true if value was changed by either widget
		};


	if (ImGui::CollapsingHeader(headerLabel, ImGuiTreeNodeFlags_DefaultOpen))
	{
		// --- Position PID ---
		ImGui::Text("Position PID Gains");
		ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);
		ImGui::Indent(); // Indent Position PID section

		// Define common layout widths (calculate once)
		float totalWidth = ImGui::GetContentRegionAvail().x;
		float inputWidth = 80.0f;
		float sliderWidth = totalWidth > (inputWidth + 20.0f) ? totalWidth - inputWidth - 20.0f : 100.0f;

		// Define gain limits based on request
		const float minXGain = 0.0f;
		const float maxXGain = -0.8f;
		const float minYGain = 0.0f;
		const float maxYGain = 0.8f;

		// --- X Axis ---
		ImGui::Text("X Axis");
		ImGui::Indent(); // Indent X controls
		if (PIDSet->XPID)
		{
			// Temporary variables to hold current values for direct ImGui interaction
			float xP = PIDSet->XPID->ProportionalGain;
			float xI = PIDSet->XPID->IntegralGain;
			float xD = PIDSet->XPID->DerivativeGain;
			bool x_changed = false;

			// X Proportional
			ImGui::PushItemWidth(sliderWidth);
			if (ImGui::SliderFloat("X P", &xP, minXGain, maxXGain)) x_changed = true;
			ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
			if (ImGui::InputFloat("##XP_Input", &xP, 0.0f, 0.0f, "%.3f")) x_changed = true;
			ImGui::PopItemWidth();
			if (x_changed)
			{
				PIDSet->XPID->ProportionalGain = xP;
				if (synchronizeXYGains && PIDSet->YPID) { PIDSet->YPID->ProportionalGain = -xP; } // Mirror Y = -X
				x_changed = false; // Reset flag for next control
			}

			// X Integral
			ImGui::PushItemWidth(sliderWidth);
			if (ImGui::SliderFloat("X I", &xI, minXGain, maxXGain)) x_changed = true;
			ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
			if (ImGui::InputFloat("##XI_Input", &xI, 0.0f, 0.0f, "%.3f")) x_changed = true;
			ImGui::PopItemWidth();
			if (x_changed)
			{
				PIDSet->XPID->IntegralGain = xI;
				if (synchronizeXYGains && PIDSet->YPID) { PIDSet->YPID->IntegralGain = -xI; } // Mirror Y = -X
				x_changed = false;
			}

			// X Derivative
			ImGui::PushItemWidth(sliderWidth);
			if (ImGui::SliderFloat("X D", &xD, minXGain, maxXGain)) x_changed = true;
			ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
			if (ImGui::InputFloat("##XD_Input", &xD, 0.0f, 0.0f, "%.3f")) x_changed = true;
			ImGui::PopItemWidth();
			if (x_changed)
			{
				PIDSet->XPID->DerivativeGain = xD;
				if (synchronizeXYGains && PIDSet->YPID) { PIDSet->YPID->DerivativeGain = -xD; } // Mirror Y = -X
				// No need to reset x_changed here
			}
		}
		else { ImGui::TextDisabled("X PID Controller Unavailable"); }
		ImGui::Unindent(); // Unindent X controls


		// --- Y Axis ---
		ImGui::Text("Y Axis");
		ImGui::Indent(); // Indent Y controls
		if (PIDSet->YPID)
		{
			// Temporary variables to hold current values
			float yP = PIDSet->YPID->ProportionalGain;
			float yI = PIDSet->YPID->IntegralGain;
			float yD = PIDSet->YPID->DerivativeGain;
			bool y_changed = false;

			// Y Proportional
			ImGui::PushItemWidth(sliderWidth);
			if (ImGui::SliderFloat("Y P", &yP, minYGain, maxYGain)) y_changed = true;
			ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
			if (ImGui::InputFloat("##YP_Input", &yP, 0.0f, 0.0f, "%.3f")) y_changed = true;
			ImGui::PopItemWidth();
			if (y_changed)
			{
				PIDSet->YPID->ProportionalGain = yP;
				if (synchronizeXYGains && PIDSet->XPID) { PIDSet->XPID->ProportionalGain = -yP; } // Mirror X = -Y
				y_changed = false; // Reset flag
			}

			// Y Integral
			ImGui::PushItemWidth(sliderWidth);
			if (ImGui::SliderFloat("Y I", &yI, minYGain, maxYGain)) y_changed = true;
			ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
			if (ImGui::InputFloat("##YI_Input", &yI, 0.0f, 0.0f, "%.3f")) y_changed = true;
			ImGui::PopItemWidth();
			if (y_changed)
			{
				PIDSet->YPID->IntegralGain = yI;
				if (synchronizeXYGains && PIDSet->XPID) { PIDSet->XPID->IntegralGain = -yI; } // Mirror X = -Y
				y_changed = false;
			}

			// Y Derivative
			ImGui::PushItemWidth(sliderWidth);
			if (ImGui::SliderFloat("Y D", &yD, minYGain, maxYGain)) y_changed = true;
			ImGui::PopItemWidth(); ImGui::SameLine(); ImGui::PushItemWidth(inputWidth);
			if (ImGui::InputFloat("##YD_Input", &yD, 0.0f, 0.0f, "%.3f")) y_changed = true;
			ImGui::PopItemWidth();
			if (y_changed)
			{
				PIDSet->YPID->DerivativeGain = yD;
				if (synchronizeXYGains && PIDSet->XPID) { PIDSet->XPID->DerivativeGain = -yD; } // Mirror X = -Y
				// No need to reset y_changed here
			}
		}
		else { ImGui::TextDisabled("Y PID Controller Unavailable"); }
		ImGui::Unindent(); // Unindent Y controls


		// --- Z Axis ---
		// Z axis uses the helper function as it's not synchronized with X/Y mirror logic
		ImGui::Text("Z Axis");
		ImGui::Indent(); // Indent Z controls
		if (PIDSet->ZPID)
		{
			// Using the original range [0, 10] for Z, adjust if needed
			DrawPIDGainControl("Z P", &PIDSet->ZPID->ProportionalGain, 0.0f, 10.0f);
			DrawPIDGainControl("Z I", &PIDSet->ZPID->IntegralGain, 0.0f, 10.0f);
			DrawPIDGainControl("Z D", &PIDSet->ZPID->DerivativeGain, 0.0f, 10.0f);
		}
		else { ImGui::TextDisabled("Z PID Controller Unavailable"); }
		ImGui::Unindent(); // Unindent Z controls

		ImGui::Unindent(); // Unindent Position PID section
		ImGui::Separator();

		// --- Attitude PID ---
		// Attitude PID logic remains unchanged (using synchronizeGains for Roll/Pitch)
		ImGui::Text("Attitude PID Gains");
		ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeGains);
		ImGui::Indent(); // Indent Attitude PID section

		// Roll
		ImGui::Text("Roll");
		ImGui::Indent();
		if (PIDSet->RollPID)
		{
			if (synchronizeGains && PIDSet->PitchPID)
			{
				if (DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, 0.0f, 5.0f))
					PIDSet->PitchPID->ProportionalGain = PIDSet->RollPID->ProportionalGain;
				if (DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, 0.0f, 5.0f))
					PIDSet->PitchPID->IntegralGain = PIDSet->RollPID->IntegralGain;
				if (DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, 0.0f, 5.0f))
					PIDSet->PitchPID->DerivativeGain = PIDSet->RollPID->DerivativeGain;
			}
			else // Not synchronizing or PitchPID is null
			{
				DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, 0.0f, 5.0f);
				DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, 0.0f, 5.0f);
				DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, 0.0f, 5.0f);
			}
		}
		else { ImGui::TextDisabled("Roll PID Unavailable"); }
		ImGui::Unindent();

		// Pitch
		ImGui::Text("Pitch");
		ImGui::Indent();
		if (PIDSet->PitchPID)
		{
			if (synchronizeGains && PIDSet->RollPID)
			{
				if (DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, 0.0f, 5.0f))
					PIDSet->RollPID->ProportionalGain = PIDSet->PitchPID->ProportionalGain;
				if (DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, 0.0f, 5.0f))
					PIDSet->RollPID->IntegralGain = PIDSet->PitchPID->IntegralGain;
				if (DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, 0.0f, 5.0f))
					PIDSet->RollPID->DerivativeGain = PIDSet->PitchPID->DerivativeGain;
			}
			else // Not synchronizing or RollPID is null
			{
				DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, 0.0f, 5.0f);
				DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, 0.0f, 5.0f);
				DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, 0.0f, 5.0f);
			}
		}
		else { ImGui::TextDisabled("Pitch PID Unavailable"); }
		ImGui::Unindent();

		// Yaw
		ImGui::Text("Yaw");
		ImGui::Indent();
		if (PIDSet->YawPID)
		{
			// Using original range [0, 2] for Yaw, adjust if needed
			DrawPIDGainControl("Yaw P", &PIDSet->YawPID->ProportionalGain, 0.0f, 2.0f);
			DrawPIDGainControl("Yaw I", &PIDSet->YawPID->IntegralGain, 0.0f, 2.0f);
			DrawPIDGainControl("Yaw D", &PIDSet->YawPID->DerivativeGain, 0.0f, 2.0f);
		}
		else { ImGui::TextDisabled("Yaw PID Unavailable"); }
		ImGui::Unindent();

		ImGui::Unindent(); // Unindent Attitude PID section
		ImGui::Separator();

		// --- Save Button ---
		// Save logic remains the same
		if (ImGui::Button("Save PID Gains", ImVec2(200, 50)))
		{
			FString FilePath = FPaths::ProjectDir() + "PIDGains.csv";
			IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
			bool bFileExists = PlatformFile.FileExists(*FilePath);
			FString Header = TEXT("Timestamp,xP,xI,xD,yP,yI,yD,zP,zI,zD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD\n");
			if (!bFileExists)
			{
				FFileHelper::SaveStringToFile(Header, *FilePath);
			}
			FString GainData;
			GainData = FDateTime::Now().ToString() + TEXT(",");
			if (PIDSet->XPID) GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->XPID->ProportionalGain, PIDSet->XPID->IntegralGain, PIDSet->XPID->DerivativeGain); else GainData += TEXT("0,0,0,");
			if (PIDSet->YPID) GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->YPID->ProportionalGain, PIDSet->YPID->IntegralGain, PIDSet->YPID->DerivativeGain); else GainData += TEXT("0,0,0,");
			if (PIDSet->ZPID) GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->ZPID->ProportionalGain, PIDSet->ZPID->IntegralGain, PIDSet->ZPID->DerivativeGain); else GainData += TEXT("0,0,0,");
			if (PIDSet->RollPID) GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->RollPID->ProportionalGain, PIDSet->RollPID->IntegralGain, PIDSet->RollPID->DerivativeGain); else GainData += TEXT("0,0,0,");
			if (PIDSet->PitchPID) GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->PitchPID->ProportionalGain, PIDSet->PitchPID->IntegralGain, PIDSet->PitchPID->DerivativeGain); else GainData += TEXT("0,0,0,");
			if (PIDSet->YawPID) GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f"), PIDSet->YawPID->ProportionalGain, PIDSet->YawPID->IntegralGain, PIDSet->YawPID->DerivativeGain); else GainData += TEXT("0,0,0");

			FFileHelper::SaveStringToFile(GainData + TEXT("\n"), *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
		}
	}
}

void UImGuiUtil::DisplayButtons()
{
	if (ImGui::Button("Switch Camera Mode", ImVec2(100, 50)))
	{
		if (DronePawn)
		{
			DronePawn->SwitchCamera();
		}
	}
	ImGui::SameLine(0, 10);

	if (ImGui::Button("Release Input", ImVec2(100, 50)))
	{
		if (DronePawn)
		{
			DronePawn->ToggleImguiInput();
		}
	}
	ImGui::SameLine(0, 10);

   if (ImGui::Button("Reset Drone up high", ImVec2(100, 50)))
   {
       // Reset high for one or all controllers
       ADroneManager* Manager = ADroneManager::Get(GetWorld());
       if (Manager && Manager->IsSwarmMode())
       {
           for (AQuadPawn* pawn : Manager->GetDroneList())
           {
               if (pawn && pawn->QuadController)
                   pawn->QuadController->ResetDroneHigh();
           }
       }
       else if (Controller)
       {
           Controller->ResetDroneHigh();
       }
   }
	ImGui::SameLine(0, 10);

   if (ImGui::Button("Reset Drone 0 point", ImVec2(100, 50)))
   {
       // Reset origin for one or all controllers
       ADroneManager* Manager = ADroneManager::Get(GetWorld());
       if (Manager && Manager->IsSwarmMode())
       {
           for (AQuadPawn* pawn : Manager->GetDroneList())
           {
               if (pawn && pawn->QuadController)
                   pawn->QuadController->ResetDroneOrigin();
           }
       }
       else if (Controller)
       {
           Controller->ResetDroneOrigin();
       }
   }
	
}

void UImGuiUtil::DisplayDesiredVelocities(float maxVelocity)
{
	ImGui::Text("Desired Velocities");

	static float prevVx = 0.0f;
    static float prevVy = 0.0f;
    static float prevVz = 0.0f;
    static float prevYr = 0.0f;
    static float desiredHoverAltitude; 
    static bool firstRun = true;

    FVector currentDesiredVelocity = Controller->GetDesiredVelocity();
    float currentYawRate = Controller->GetDesiredYawRate();
    bool hoverModeActive = Controller->IsHoverModeActive();

    float tempVx = currentDesiredVelocity.X;
    float tempVy = currentDesiredVelocity.Y;
    float tempVz = currentDesiredVelocity.Z;
    float tempYr = currentYawRate;
    bool velocityChanged = false; // Flag to force update if buttons are pressed

    // --- Hover Mode Control ---
    ImGui::PushStyleColor(ImGuiCol_Button, hoverModeActive ? ImVec4(0.1f, 0.8f, 0.6f, 1.0f) : ImVec4(0.1f, 0.6f, 0.8f, 1.0f)); // Green when ON, Blue when OFF
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hoverModeActive ? ImVec4(0.2f, 0.9f, 0.7f, 1.0f) : ImVec4(0.2f, 0.7f, 0.9f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, hoverModeActive ? ImVec4(0.0f, 0.7f, 0.5f, 1.0f) : ImVec4(0.0f, 0.5f, 0.7f, 1.0f));

    if (ImGui::Button(hoverModeActive ? "HOVER MODE ACTIVE" : "ACTIVATE HOVER MODE", ImVec2(200, 35)))
    {
        bool activateHover = !hoverModeActive;
        // Apply hover mode to swarm or single
        ADroneManager* Manager = ADroneManager::Get(GetWorld());
        if (Manager && Manager->IsSwarmMode())
        {
            for (AQuadPawn* pawn : Manager->GetDroneList())
            {
                if (pawn && pawn->QuadController)
                {
                    pawn->QuadController->SetHoverMode(activateHover, activateHover ? desiredHoverAltitude : 0.0f);
                }
            }
        }
        else if (Controller)
        {
            Controller->SetHoverMode(activateHover, activateHover ? desiredHoverAltitude : 0.0f);
        }
        hoverModeActive = activateHover;
        if (hoverModeActive)
        {
            tempVz = 0.0f;
        }
        velocityChanged = true;
    }
    ImGui::PopStyleColor(3);
	
    ImGui::SliderFloat("Desired Hover Altitude (cm)", &desiredHoverAltitude, 50.0f, 1000.0f, "%.0f cm");
    if (hoverModeActive)
    {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.1f, 0.8f, 0.6f, 1.0f), "Target Altitude: %.0f cm", desiredHoverAltitude);
        static float lastSentAltitude = -1.0f; // Track last sent value
        if (fabs(desiredHoverAltitude - lastSentAltitude) > 1.0f) { // Add deadzone/check
            // Re-send hover command for swarm or single
            ADroneManager* Manager = ADroneManager::Get(GetWorld());
            if (Manager && Manager->IsSwarmMode()) {
                for (AQuadPawn* pawn : Manager->GetDroneList()) {
                    if (pawn && pawn->QuadController) {
                        pawn->QuadController->SetHoverMode(true, desiredHoverAltitude);
                    }
                }
            } else if (Controller) {
                Controller->SetHoverMode(true, desiredHoverAltitude);
            }
            lastSentAltitude = desiredHoverAltitude;
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Desired Velocities & Yaw Rate");
    ImGui::Spacing();

    ImGui::SliderFloat("Desired Velocity X", &tempVx, -maxVelocity, maxVelocity, "%.1f cm/s");
    ImGui::SliderFloat("Desired Velocity Y", &tempVy, -maxVelocity, maxVelocity, "%.1f cm/s");

    if (hoverModeActive)
    {
        tempVz = 0.0f; // Explicitly keep desired Z velocity at 0 when hover is active
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f); // Dim the slider
        ImGui::SliderFloat("Desired Velocity Z (Hover)", &tempVz, -maxVelocity, maxVelocity, "%.1f cm/s");
        ImGui::PopStyleVar();
    }
    else
    {
        ImGui::SliderFloat("Desired Velocity Z", &tempVz, -maxVelocity, maxVelocity, "%.1f cm/s");
    }

    ImGui::SliderFloat("Desired Yaw Rate", &tempYr, -50.f, 50.f, "%.1f deg/s");

    // ImGui::PopItemWidth(); // If PushItemWidth was used

    ImGui::Separator();

    ImGui::Text("Reset to 0:");
    ImGui::SameLine();
    if (ImGui::Button("X"))
    {
        tempVx = 0.0f;
        velocityChanged = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Y"))
    {
        tempVy = 0.0f;
        velocityChanged = true;
    }
    ImGui::SameLine();
    if (!hoverModeActive)
    {
        if (ImGui::Button("Z"))
        {
           tempVz = 0.0f;
           velocityChanged = true;
        }
    } else {
       // Optionally show a disabled Z reset button
       ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
       ImGui::Button("Z"); // Does nothing
       ImGui::PopStyleVar();
    }
    ImGui::SameLine();
    if (ImGui::Button("Yaw"))
    {
        tempYr = 0.0f;
        velocityChanged = true;
    }

    ImGui::Separator();

    if (firstRun)
    {
        prevVx = tempVx;
        prevVy = tempVy;
        prevVz = tempVz;
        prevYr = tempYr;
        firstRun = false;
    }

    const float threshold = 0.01f; 
    bool significantChange = (FMath::Abs(tempVx - prevVx) > threshold) ||
                             (FMath::Abs(tempVy - prevVy) > threshold) ||
                             (FMath::Abs(tempVz - prevVz) > threshold) ||
                             (FMath::Abs(tempYr - prevYr) > threshold);
	
    if (velocityChanged || significantChange)
    {
        // Ensure Z velocity is zero if hover mode is active, regardless of slider state
        if (hoverModeActive) tempVz = 0.0f;

        FVector desiredNewVelocity = FVector(tempVx, tempVy, tempVz);
        // Apply velocity and yaw commands to swarm or single
        ADroneManager* Manager = ADroneManager::Get(GetWorld());
        if (Manager && Manager->IsSwarmMode())
        {
            for (AQuadPawn* pawn : Manager->GetDroneList())
            {
                if (pawn && pawn->QuadController)
                {
                    pawn->QuadController->SetDesiredVelocity(desiredNewVelocity);
                    pawn->QuadController->SetDesiredYawRate(tempYr);
                }
            }
        }
        else if (Controller)
        {
            Controller->SetDesiredVelocity(desiredNewVelocity);
            Controller->SetDesiredYawRate(tempYr);
        }

        // Update previous values so that subsequent small changes within the threshold are ignored
        prevVx = tempVx;
        prevVy = tempVy;
        prevVz = tempVz;
        prevYr = tempYr;
    }
}


void UImGuiUtil::DisplayDesiredPositions()
{
	if (!Controller)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Controller not available!");
		return;
	}

	ImGui::Text("Desired Position Setpoint (m)");

	static float prevPx = 0.0f;
	static float prevPy = 0.0f;
	static float prevPz = 0.0f;
	static bool firstRun = true;

	static bool resetXChecked = false;
	static bool resetYChecked = false;
	static bool resetZChecked = false;

	FVector currentSetpoint_cm = Controller->GetCurrentSetPoint();

	float tempPx_cm = currentSetpoint_cm.X;
	float tempPy_cm = currentSetpoint_cm.Y;
	float tempPz_cm = currentSetpoint_cm.Z;

	float displayPx_m = tempPx_cm / 100.0f;
	float displayPy_m = tempPy_cm / 100.0f;
	float displayPz_m = tempPz_cm / 100.0f;

	bool positionChanged = false;

	const float maxPositionBound_m = 2000.0f;
	const float minPositionBound_m = -2000.0f;
	const float maxPositionBoundZ_m = 1000.0f;
	const float minPositionBoundZ_m = 0.0f;

	ImGui::Spacing();

	if (ImGui::SliderFloat("Desired Position X (m)", &displayPx_m, minPositionBound_m, maxPositionBound_m))
	{
		tempPx_cm = displayPx_m * 100.0f;
		positionChanged = true;
	}
	ImGui::SameLine();
	if (ImGui::Checkbox("Reset X to 0##Pos", &resetXChecked))
	{
		if (resetXChecked)
		{
			tempPx_cm = 0.0f;
			displayPx_m = 0.0f;
			positionChanged = true;
		}
		resetXChecked = false;
	}

	if (ImGui::SliderFloat("Desired Position Y (m)", &displayPy_m, minPositionBound_m, maxPositionBound_m))
	{
		tempPy_cm = displayPy_m * 100.0f;
		positionChanged = true;
	}
	ImGui::SameLine();
	if (ImGui::Checkbox("Reset Y to 0##Pos", &resetYChecked))
	{
		if (resetYChecked)
		{
			tempPy_cm = 0.0f;
			displayPy_m = 0.0f;
			positionChanged = true;
		}
		resetYChecked = false;
	}

	if (ImGui::SliderFloat("Desired Position Z (m)", &displayPz_m, minPositionBoundZ_m, maxPositionBoundZ_m))
	{
		tempPz_cm = displayPz_m * 100.0f;
		positionChanged = true;
	}
	ImGui::SameLine();
	if (ImGui::Checkbox("Reset Z to 0##Pos", &resetZChecked))
	{
		if (resetZChecked)
		{
			tempPz_cm = 0.0f;
			displayPz_m = 0.0f;
			positionChanged = true;
		}
		resetZChecked = false;
	}

	if (firstRun)
	{
		prevPx = tempPx_cm;
		prevPy = tempPy_cm;
		prevPz = tempPz_cm;
		firstRun = false;
	}

	const float threshold_cm = 0.1f;
	bool significantChange = (FMath::Abs(tempPx_cm - prevPx) > threshold_cm) ||
		(FMath::Abs(tempPy_cm - prevPy) > threshold_cm) ||
		(FMath::Abs(tempPz_cm - prevPz) > threshold_cm);


	if (significantChange || positionChanged)
	{
		FVector desiredNewPosition_cm = FVector(tempPx_cm, tempPy_cm, tempPz_cm);
		// Apply new destination to swarm or single controller
		ADroneManager* Manager = ADroneManager::Get(GetWorld());
		if (Manager && Manager->IsSwarmMode())
		{
			for (AQuadPawn* pawn : Manager->GetDroneList())
			{
				if (pawn && pawn->QuadController)
				{
					pawn->QuadController->SetDestination(desiredNewPosition_cm);
				}
			}
		}
		else if (Controller)
		{
			Controller->SetDestination(desiredNewPosition_cm);
		}

		prevPx = tempPx_cm;
		prevPy = tempPy_cm;
		prevPz = tempPz_cm;
	}

	ImGui::Separator();
}

void UImGuiUtil::DisplayPIDHistoryWindow()
{
	ImGui::SetNextWindowPos(ImVec2(420, 520), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(800, 400), ImGuiCond_FirstUseEver);

	if (!ImGui::Begin("PID Configurations History"))
	{
		ImGui::End();
		return;
	}

	// Path to the CSV file
	FString FilePath = FPaths::ProjectDir() + "PIDGains.csv";

	// Check if file exists
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!PlatformFile.FileExists(*FilePath))
	{
		ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "PID history file not found: %s", TCHAR_TO_UTF8(*FilePath));
		ImGui::End();
		return;
	}

	// Read the CSV file
	FString FileContent;
	if (!FFileHelper::LoadFileToString(FileContent, *FilePath))
	{
		ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Failed to read PID history file");
		ImGui::End();
		return;
	}

	// Parse the CSV content
	TArray<FString> Lines;
	FileContent.ParseIntoArrayLines(Lines, false);

	if (Lines.Num() < 2) // Need at least header and one data row
	{
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "PID history file is empty or invalid");
		ImGui::End();
		return;
	}

	// Set up table
	static ImGuiTableFlags TableFlags =
		ImGuiTableFlags_Borders |
		ImGuiTableFlags_RowBg |
		ImGuiTableFlags_ScrollY |
		ImGuiTableFlags_SizingFixedFit;

	if (ImGui::BeginTable("PIDHistoryTable", 19, TableFlags, ImVec2(0, 0), 0.0f))
	{
		// Parse header
		TArray<FString> Headers;
		Lines[0].ParseIntoArray(Headers, TEXT(","), true);

		// Add headers to table
		ImGui::TableSetupScrollFreeze(1, 1); // Freeze header row
		for (int32 ColIdx = 0; ColIdx < Headers.Num(); ColIdx++)
		{
			ImGui::TableSetupColumn(TCHAR_TO_UTF8(*Headers[ColIdx]), ImGuiTableColumnFlags_WidthFixed);
		}
		ImGui::TableHeadersRow();

		// Process data rows
		for (int32 RowIdx = 1; RowIdx < Lines.Num(); RowIdx++)
		{
			// Skip empty lines
			if (Lines[RowIdx].IsEmpty())
				continue;

			ImGui::TableNextRow();

			TArray<FString> Values;
			Lines[RowIdx].ParseIntoArray(Values, TEXT(","), true);

			// Fill in row data
			for (int32 ColIdx = 0; ColIdx < Values.Num() && ColIdx < Headers.Num(); ColIdx++)
			{
				ImGui::TableSetColumnIndex(ColIdx);

				// For timestamp column (0), just display the value
				if (ColIdx == 0)
				{
					ImGui::TextUnformatted(TCHAR_TO_UTF8(*Values[ColIdx]));

					// Add Load button in the first column
					ImGui::SameLine();
					FString ButtonLabel = "Load##" + FString::FromInt(RowIdx);
					if (ImGui::SmallButton(TCHAR_TO_UTF8(*ButtonLabel)))
					{
						// When clicked, load these PID values
						//LoadPIDValues(DroneMode, Values);
					}
				}
				else
				{
					// For numeric columns, align right and convert to float for display
					float Value = FCString::Atof(*Values[ColIdx]);
					ImGui::Text("%6.3f", Value);
				}
			}
		}

		ImGui::EndTable();
	}

	ImGui::End();
}

// Helper method to load PID values from a row
void UImGuiUtil::LoadPIDValues(EFlightMode Mode, const TArray<FString>& Values)
{
	if (!Controller || Values.Num() < 19) // Ensure we have all values (timestamp + 18 PID values)
		return;

	FFullPIDSet* PIDSet = Controller->GetPIDSet(Mode);
	if (!PIDSet)
		return;

	// Values order: timestamp, xP, xI, xD, yP, yI, yD, zP, zI, zD, rollP, rollI, rollD, pitchP, pitchI, pitchD, yawP, yawI, yawD

	// Load X PID
	if (PIDSet->XPID)
	{
		PIDSet->XPID->ProportionalGain = FCString::Atof(*Values[1]);
		PIDSet->XPID->IntegralGain = FCString::Atof(*Values[2]);
		PIDSet->XPID->DerivativeGain = FCString::Atof(*Values[3]);
	}

	// Load Y PID
	if (PIDSet->YPID)
	{
		PIDSet->YPID->ProportionalGain = FCString::Atof(*Values[4]);
		PIDSet->YPID->IntegralGain = FCString::Atof(*Values[5]);
		PIDSet->YPID->DerivativeGain = FCString::Atof(*Values[6]);
	}

	// Load Z PID
	if (PIDSet->ZPID)
	{
		PIDSet->ZPID->ProportionalGain = FCString::Atof(*Values[7]);
		PIDSet->ZPID->IntegralGain = FCString::Atof(*Values[8]);       
		PIDSet->ZPID->DerivativeGain = FCString::Atof(*Values[9]);
	}

	// Load Roll PID
	if (PIDSet->RollPID)
	{
		PIDSet->RollPID->ProportionalGain = FCString::Atof(*Values[10]);
		PIDSet->RollPID->IntegralGain = FCString::Atof(*Values[11]);
		PIDSet->RollPID->DerivativeGain = FCString::Atof(*Values[12]);
	}

	// Load Pitch PID
	if (PIDSet->PitchPID)
	{
		PIDSet->PitchPID->ProportionalGain = FCString::Atof(*Values[13]);
		PIDSet->PitchPID->IntegralGain = FCString::Atof(*Values[14]);
		PIDSet->PitchPID->DerivativeGain = FCString::Atof(*Values[15]);
	}

	// Load Yaw PID
	if (PIDSet->YawPID)
	{
		PIDSet->YawPID->ProportionalGain = FCString::Atof(*Values[16]);
		PIDSet->YawPID->IntegralGain = FCString::Atof(*Values[17]);
		PIDSet->YawPID->DerivativeGain = FCString::Atof(*Values[18]);
	}

	// Notify of successful load
	UE_LOG(LogTemp, Display, TEXT("Loaded PID configuration from %s"), *Values[0]);
}