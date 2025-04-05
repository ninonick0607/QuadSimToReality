#include "UI/ImGuiUtil.h"
#include "imgui.h"
#include "implot.h"
#include "Pawns/QuadPawn.h"
#include "string"
#include "Controllers/QuadDroneController.h"
#include "Core/DroneJSONConfig.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "Kismet/GameplayStatics.h"
#include <string>
#include "Misc/DateTime.h"

UImGuiUtil::UImGuiUtil()
	: DronePawn(nullptr)
	, Controller(nullptr)
	, CumulativeTime(0.0f)
	, MaxPlotTime(10.0f)
{
	const auto& Config = UDroneJSONConfig::Get().Config;
	PrimaryComponentTick.bCanEverTick = true;
	maxVelocity = Config.FlightParams.MaxVelocity;
	maxAngle = Config.FlightParams.MaxAngle;

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

void UImGuiUtil::VelocityHud(TArray<float>& ThrustsVal,
                                  float rollError, float pitchError,
                                  const FRotator& currentRotation,
                                  const FVector& waypoint, const FVector& currLoc,
                                  const FVector& error,
                                  const FVector& currentVelocity,
                                  float xOutput, float yOutput, float zOutput, float deltaTime)
{
	// Set up window position and size
	ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);

	// Instead of looking for a component on the pawn, we search the world for our dedicated actor.
	AZMQController* zmqControllerCurrent = nullptr;
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AZMQController::StaticClass(), FoundActors);
	if (FoundActors.Num() > 0)
	{
		zmqControllerCurrent = Cast<AZMQController>(FoundActors[0]);
	}

	FVector currentGoalState = FVector::ZeroVector;
	FString droneID = FString("Unknown");
	if (zmqControllerCurrent && zmqControllerCurrent->IsValidLowLevel())
	{
		currentGoalState = zmqControllerCurrent->GetCurrentGoalPosition();
		droneID = zmqControllerCurrent->GetConfiguration().DroneID;
	}

	// Use ## to give ImGui a hidden unique identifier
	FString WindowName = FString::Printf(TEXT("Drone Controller##%s"), *droneID);

	ImGui::Begin(TCHAR_TO_UTF8(*WindowName), nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar);
	
	ImGui::Text("Drone ID: %s", TCHAR_TO_UTF8(*droneID));
	static bool bLocalManualMode = false;
	if (ImGui::Checkbox("Manual Thrust Mode", &bLocalManualMode))
	{
		if (Controller)
		{
			Controller->SetManualThrustMode(bLocalManualMode);
		}
	}
	// Display drone info and various UI elements
	DisplayDroneInfo();
	ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, 600.0f);
	ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);
	Controller->SetDesiredAngle(maxAngle);
	DisplayDesiredVelocities();

	ImGui::Separator();
	ImGui::Text("Thruster Power");

	static float AllThrustValue = 0.0f;
	if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, 0, 700.0f))
	{
		for (int i = 0; i < ThrustsVal.Num(); i++)
		{
			ThrustsVal[i] = AllThrustValue;
		}
	}

	ImGui::Separator();
	ImGui::Text("Thruster Power");

	static bool synchronizeDiagonal1 = false;
	static bool synchronizeDiagonal2 = false;
	ImGui::Checkbox("Synchronize Diagonal Motors FL & BR", &synchronizeDiagonal1);
	ImGui::Checkbox("Synchronize Diagonal Motors FR & BL", &synchronizeDiagonal2);

	ImGui::Separator();

	if (ThrustsVal.Num() >= 4)
	{
		ImGui::Text("Diagonal 1 Motors");
    
		// Push a unique ID to differentiate these widgets
		ImGui::PushID("Diag1");
		ImGui::Indent();
		if (synchronizeDiagonal1)
		{
			if (ImGui::SliderFloat("FL & BR Thrust", &ThrustsVal[0], 0, 700.0f))
			{
				ThrustsVal[3] = ThrustsVal[0];
			}
			ImGui::Text("Back Right (Synchronized): %.2f", ThrustsVal[3]);
		}
		else
		{
			// "Front Left" is effectively "Diag1/Front Left"
			ImGui::SliderFloat("Front Left", &ThrustsVal[0], 0, 700.0f);
			ImGui::SliderFloat("Back Right", &ThrustsVal[3], 0, 700.0f);
		}
		ImGui::Unindent();
		ImGui::PopID();  // pop "Diag1"

		// -------------------- Diagonal 2 --------------------
		ImGui::Text("Diagonal 2 Motors");
    
		ImGui::PushID("Diag2");
		ImGui::Indent();
		if (synchronizeDiagonal2)
		{
			if (ImGui::SliderFloat("FR & BL Thrust", &ThrustsVal[1], 0, 700.0f))
			{
				ThrustsVal[2] = ThrustsVal[1];
			}
			ImGui::Text("Back Left (Synchronized): %.2f", ThrustsVal[2]);
		}
		else
		{
			ImGui::SliderFloat("Front Right", &ThrustsVal[1], 0, 700.0f);
			ImGui::SliderFloat("Back Left", &ThrustsVal[2], 0, 700.0f);
		}
		ImGui::Unindent();
		ImGui::PopID(); // pop "Diag2"
	}
    FVector currentDesiredVelocity = Controller->GetDesiredVelocity();

	ImGui::Separator();
	ImGui::Text("Desired Roll: %.2f", rollError);
	ImGui::SameLine();
	ImGui::Text("Current Roll: %.2f", currentRotation.Roll);
	ImGui::Text("Desired Pitch: %.2f", pitchError);
	ImGui::SameLine();
	ImGui::Text("Current Pitch: %.2f", currentRotation.Pitch);

	ImGui::Separator();
	ImGui::Text("Current Position X, Y, Z: %.2f, %.2f, %.2f", currLoc.X, currLoc.Y, currLoc.Z);
	ImGui::Text("Desired Position X, Y, Z: %.2f, %.2f, %.2f", waypoint.X, waypoint.Y, waypoint.Z);
	ImGui::Text("Position Error X, Y, Z: %.2f, %.2f, %.2f", error.X, error.Y, error.Z);
	ImGui::Spacing();
	ImGui::Text("Velocity Command Received X, Y, Z: %.2f, %.2f, %.2f", currentDesiredVelocity.X, currentDesiredVelocity.Y, currentDesiredVelocity.Z);
	ImGui::Text("Current Velocity X, Y, Z: %.2f, %.2f, %.2f", currentVelocity.X, currentVelocity.Y, currentVelocity.Z);
	ImGui::Text("Current Goal State X, Y, Z: %.2f, %.2f, %.2f", currentGoalState.X, currentGoalState.Y, currentGoalState.Z);
	ImGui::Spacing();

	static bool syncXY = false;
	static bool syncRP = false;
	DisplayPIDSettings("PID Settings", syncXY, syncRP);

	ImGui::Separator();
	ImGui::Text("Position PID Outputs");
	ImGui::Text("X Output: %.2f", xOutput);
	ImGui::Text("Y Output: %.2f", yOutput);
	ImGui::Text("Z Output: %.2f", zOutput);
	ImGui::Separator();

	DisplayCameraControls();
	DisplayResetDroneButtons();
	DisplayPIDHistoryWindow();

	ImGui::End();

}

void UImGuiUtil::RenderImPlot(const TArray<float>& ThrustsVal, const FVector& desiredForwardVector, const FVector& currentForwardVector, float deltaTime)
{
    if (ThrustsVal.Num() < 4)
    {
        return;
    }

    CumulativeTime += deltaTime;
    TimeData.Add(CumulativeTime);
    
    // Add thruster data
    Thrust0Data.Add(ThrustsVal[0]);
    Thrust1Data.Add(ThrustsVal[1]);
    Thrust2Data.Add(ThrustsVal[2]);
    Thrust3Data.Add(ThrustsVal[3]);
    
    // Extract heading angles from forward vectors (ignoring Z component)
    // Using atan2 to get the yaw angle in degrees
    FVector desiredFlat = desiredForwardVector;
    desiredFlat.Z = 0;
    desiredFlat.Normalize();
    
    FVector currentFlat = currentForwardVector;
    currentFlat.Z = 0;
    currentFlat.Normalize();
    
    // Calculate heading angles (converts from -180 to 180 range)
    float desiredHeading = FMath::RadiansToDegrees(FMath::Atan2(desiredFlat.Y, desiredFlat.X));
    float currentHeading = FMath::RadiansToDegrees(FMath::Atan2(currentFlat.Y, currentFlat.X));
    
    // For consistency when crossing the 180/-180 boundary
    float rawErrorAngle = desiredHeading - currentHeading;
    // Normalize to -180 to 180 range
    while (rawErrorAngle > 180.0f) rawErrorAngle -= 360.0f;
    while (rawErrorAngle < -180.0f) rawErrorAngle += 360.0f;
    
    // Add the heading data to our arrays
    DesiredHeadingData.Add(desiredHeading);
    CurrentHeadingData.Add(currentHeading);
    VectorErrorData.Add(rawErrorAngle);

    // Clean up old data points for all arrays
    while (TimeData.Num() > 0 && (CumulativeTime - TimeData[0] > MaxPlotTime))
    {
        TimeData.RemoveAt(0);
        Thrust0Data.RemoveAt(0);
        Thrust1Data.RemoveAt(0);
        Thrust2Data.RemoveAt(0);
        Thrust3Data.RemoveAt(0);
        DesiredHeadingData.RemoveAt(0);
        CurrentHeadingData.RemoveAt(0);
        VectorErrorData.RemoveAt(0);
    }

    while (TimeData.Num() > MaxDataPoints)
    {
        TimeData.RemoveAt(0);
        Thrust0Data.RemoveAt(0);
        Thrust1Data.RemoveAt(0);
        Thrust2Data.RemoveAt(0);
        Thrust3Data.RemoveAt(0);
        DesiredHeadingData.RemoveAt(0);
        CurrentHeadingData.RemoveAt(0);
        VectorErrorData.RemoveAt(0);
    }

    ImGui::SetNextWindowPos(ImVec2(850, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(600, 600), ImGuiCond_FirstUseEver);

    ImGui::Begin("Drone Analysis", nullptr, ImGuiWindowFlags_NoCollapse);

    ImVec2 windowSize = ImGui::GetContentRegionAvail();
    float plotHeight = windowSize.y * 0.5f;  // Split window for two plots
    ImVec2 plotSize(windowSize.x, plotHeight - 10);
    ImPlotFlags plotFlags = ImPlotFlags_None;
    ImPlotAxisFlags axisFlags = ImPlotAxisFlags_None;

    // First plot for thrust values
    if (ImPlot::BeginPlot("Thrust Values Over Time", plotSize, plotFlags))
    {
        ImPlot::SetupAxes("Time (s)", "Thrust (N)", axisFlags, axisFlags);

        const ImVec4 FL_COLOR(1.0f, 0.2f, 0.2f, 1.0f);  // Red
        const ImVec4 FR_COLOR(0.2f, 1.0f, 0.2f, 1.0f);  // Green
        const ImVec4 BL_COLOR(0.2f, 0.6f, 1.0f, 1.0f);  // Blue
        const ImVec4 BR_COLOR(1.0f, 0.8f, 0.0f, 1.0f);  // Yellow

        ImPlot::SetNextLineStyle(FL_COLOR, 2.0f);
        ImPlot::PlotLine("Front Left", TimeData.GetData(), Thrust0Data.GetData(), TimeData.Num());

        ImPlot::SetNextLineStyle(FR_COLOR, 2.0f);
        ImPlot::PlotLine("Front Right", TimeData.GetData(), Thrust1Data.GetData(), TimeData.Num());

        ImPlot::SetNextLineStyle(BL_COLOR, 2.0f);
        ImPlot::PlotLine("Back Left", TimeData.GetData(), Thrust2Data.GetData(), TimeData.Num());

        ImPlot::SetNextLineStyle(BR_COLOR, 2.0f);
        ImPlot::PlotLine("Back Right", TimeData.GetData(), Thrust3Data.GetData(), TimeData.Num());

        ImPlot::EndPlot();
    }

    ImGui::Spacing();

    // Second plot for heading comparison and error
    if (ImPlot::BeginPlot("Heading Comparison", plotSize, plotFlags))
    {
        ImPlot::SetupAxes("Time (s)", "Heading (degrees)", axisFlags, axisFlags);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -180, 180, ImPlotCond_Once);

        const ImVec4 DESIRED_COLOR(0.2f, 0.7f, 0.9f, 1.0f);  // Blue for desired
        const ImVec4 CURRENT_COLOR(1.0f, 0.4f, 0.4f, 1.0f);  // Red for current
        const ImVec4 ERROR_COLOR(0.8f, 0.4f, 0.9f, 1.0f);    // Purple for error

        ImPlot::SetNextLineStyle(DESIRED_COLOR, 2.0f);
        ImPlot::PlotLine("Desired Heading", TimeData.GetData(), DesiredHeadingData.GetData(), TimeData.Num());

        ImPlot::SetNextLineStyle(CURRENT_COLOR, 2.0f);
        ImPlot::PlotLine("Current Heading", TimeData.GetData(), CurrentHeadingData.GetData(), TimeData.Num());

        ImPlot::SetNextLineStyle(ERROR_COLOR, 1.5f);
        ImPlot::PlotLine("Heading Error", TimeData.GetData(), VectorErrorData.GetData(), TimeData.Num());

        ImPlot::EndPlot();
    }

    ImGui::End();
}

void UImGuiUtil::DisplayDroneInfo()
{
	ImGui::Text("Drone Model Feedback");
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
}

void UImGuiUtil::DisplayPIDSettings(const char* headerLabel, bool& synchronizeXYGains, bool& synchronizeGains)
{
	FFullPIDSet* PIDSet = Controller ? Controller->GetPIDSet() : nullptr;
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
		const float maxXGain = -0.1f;
		const float minYGain = 0.0f;
		const float maxYGain = 0.1f;

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
				if (DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, 0.0f, 20.0f))
					PIDSet->PitchPID->ProportionalGain = PIDSet->RollPID->ProportionalGain;
				if (DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, 0.0f, 20.0f))
					PIDSet->PitchPID->IntegralGain = PIDSet->RollPID->IntegralGain;
				if (DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, 0.0f, 20.0f))
					PIDSet->PitchPID->DerivativeGain = PIDSet->RollPID->DerivativeGain;
			}
			else // Not synchronizing or PitchPID is null
			{
				DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, 0.0f, 20.0f);
				DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, 0.0f, 20.0f);
				DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, 0.0f, 20.0f);
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
				if (DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, 0.0f, 20.0f))
					PIDSet->RollPID->ProportionalGain = PIDSet->PitchPID->ProportionalGain;
				if (DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, 0.0f, 20.0f))
					PIDSet->RollPID->IntegralGain = PIDSet->PitchPID->IntegralGain;
				if (DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, 0.0f, 20.0f))
					PIDSet->RollPID->DerivativeGain = PIDSet->PitchPID->DerivativeGain;
			}
			else // Not synchronizing or RollPID is null
			{
				DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, 0.0f, 20.0f);
				DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, 0.0f, 20.0f);
				DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, 0.0f, 20.0f);
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

void UImGuiUtil::DisplayCameraControls()
{
	ImGui::Separator();
	ImGui::Spacing();

	if (ImGui::Button("Switch Camera Mode", ImVec2(200, 50)))
	{
		if (DronePawn)
		{
			DronePawn->SwitchCamera();
		}
	}
}

void UImGuiUtil::DisplayResetDroneButtons()
{
	if (ImGui::Button("Release Input", ImVec2(200, 100)))
	{
		if (DronePawn)
		{
			DronePawn->ToggleImguiInput();
		}
	}

	if (ImGui::Button("Reset Drone up high", ImVec2(200, 100)))
	{
		if (Controller)
		{
			Controller->ResetDroneHigh();
		}
	}

	if (ImGui::Button("Reset Drone 0 point", ImVec2(200, 100)))
	{
		if (Controller)
		{
			Controller->ResetDroneOrigin();
		}
	}
}

void UImGuiUtil::DisplayDesiredVelocities()
{
    ImGui::Text("Desired Velocities");

    // Static variables to hold previous slider values
    static float prevVx = 0.0f;
    static float prevVy = 0.0f;
    static float prevVz = 0.0f;
	static float prevYr = 0.0f;
    static bool firstRun = true;
    
    // Reset checkboxes states (we need separate variables for these)
    static bool resetXChecked = false;
    static bool resetYChecked = false;
    static bool resetZChecked = false;
	static bool resetYrChecked = false;
    
    FVector currentDesiredVelocity = Controller->GetDesiredVelocity();
	float currentYawRate = Controller->GetDesiredYawRate();
    bool hoverModeActive = Controller->IsHoverModeActive();  // Get hover mode state from controller

    float tempVx = currentDesiredVelocity.X;
    float tempVy = currentDesiredVelocity.Y;
    float tempVz = currentDesiredVelocity.Z;
	float tempYr = currentYawRate;
    bool velocityChanged = false;

    // Add hover mode button with distinctive styling
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.6f, 0.8f, 1.0f)); // Blue button
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.7f, 0.9f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.5f, 0.7f, 1.0f));

    if (ImGui::Button(hoverModeActive ? "HOVER MODE ACTIVE" : "ACTIVATE HOVER MODE", ImVec2(200, 35)))
    {
        // Toggle hover mode through the controller
        Controller->SetHoverMode(!hoverModeActive);
        
        // Update local values to match the new state
        if (!hoverModeActive)  // It's about to be activated
            tempVz = 28.0f;
            
        velocityChanged = true;
    }
    ImGui::PopStyleColor(3);

    if (hoverModeActive)
    {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.1f, 0.6f, 0.8f, 1.0f), "Z-velocity locked at 28.0");
    }

    ImGui::Spacing();

    // X velocity slider with reset checkbox
    velocityChanged |= ImGui::SliderFloat("Desired Velocity X", &tempVx, -maxVelocity, maxVelocity);
    ImGui::SameLine();
    if (ImGui::Checkbox("Reset X to 0", &resetXChecked))
    {
        if (resetXChecked)
        {
            tempVx = 0.0f;
            velocityChanged = true;
        }
        // Auto-uncheck after resetting
        resetXChecked = false;
    }

    // Y velocity slider with reset checkbox
    velocityChanged |= ImGui::SliderFloat("Desired Velocity Y", &tempVy, -maxVelocity, maxVelocity);
    ImGui::SameLine();
    if (ImGui::Checkbox("Reset Y to 0", &resetYChecked))
    {
        if (resetYChecked)
        {
            tempVy = 0.0f;
            velocityChanged = true;
        }
        // Auto-uncheck after resetting
        resetYChecked = false;
    }

    // Only show Z slider control if hover mode is not active
    if (!hoverModeActive)
    {
        velocityChanged |= ImGui::SliderFloat("Desired Velocity Z", &tempVz, -maxVelocity, maxVelocity);
        ImGui::SameLine();
        if (ImGui::Checkbox("Reset Z to 0", &resetZChecked))
        {
            if (resetZChecked)
            {
                tempVz = 0.0f;
                velocityChanged = true;
            }
            // Auto-uncheck after resetting
            resetZChecked = false;
        }
    }
    else
    {
        // Display a disabled slider for Z when in hover mode
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(0.1f, 0.6f, 0.8f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.2f, 0.2f, 0.2f, 0.5f));
        ImGui::SliderFloat("Desired Velocity Z (Locked)", &tempVz, -maxVelocity, maxVelocity);
        ImGui::PopStyleColor(2);

        // In hover mode, Z velocity is always 0.0
        tempVz = 0.0f;
    }

	// Yaw rate velocity slider with reset checkbox
    velocityChanged |= ImGui::SliderFloat("Desired Yaw Rate", &tempYr, -50.f, 50.f);
    ImGui::SameLine();
    if (ImGui::Checkbox("Reset Yr to 0", &resetYrChecked))
    {
        if (resetYrChecked)
        {
            tempYr = 0.0f;
            velocityChanged = true;
        }
        // Auto-uncheck after resetting
        resetYrChecked = false;
    }

    // On first run, initialize previous values
    if (firstRun)
    {
        prevVx = tempVx;
        prevVy = tempVy;
        prevVz = tempVz;
		prevYr = tempYr;
        firstRun = false;
    }

    // Set a deadzone threshold (adjust as needed)
    const float threshold = 0.01f;
    bool significantChange = (FMath::Abs(tempVx - prevVx) > threshold) ||
        (FMath::Abs(tempVy - prevVy) > threshold) ||
        (FMath::Abs(tempVz - prevVz) > threshold) ||
		(FMath::Abs(tempYr - prevYr) > threshold);

    // Only update the desired velocity if there's a significant change or if we just entered hover mode
    if (significantChange || velocityChanged)
    {
        FVector desiredNewVelocity = FVector(tempVx, tempVy, tempVz);
        if (Controller)
        {
            Controller->SetDesiredVelocity(desiredNewVelocity);
			Controller->SetDesiredYawRate(tempYr);
        }
        // Update previous values so that subsequent small changes are ignored
        prevVx = tempVx;
        prevVy = tempVy;
        prevVz = tempVz;
		prevYr = tempYr;
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
						LoadPIDValues(Values);
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
void UImGuiUtil::LoadPIDValues(const TArray<FString>& Values)
{
	if (!Controller || Values.Num() < 19) // Ensure we have all values (timestamp + 18 PID values)
		return;

	FFullPIDSet* PIDSet = Controller->GetPIDSet();
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