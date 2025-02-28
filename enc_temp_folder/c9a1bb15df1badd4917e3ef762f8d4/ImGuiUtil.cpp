#include "UI/ImGuiUtil.h"
#include "imgui.h"
#include "implot.h"
#include "Pawns/QuadPawn.h"
#include "string"
#include "Controllers/QuadDroneController.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/DateTime.h"

UImGuiUtil::UImGuiUtil()
	: DronePawn(nullptr)
	, Controller(nullptr)
	, Debug_DrawDroneCollisionSphere(true)
	, Debug_DrawDroneWaypoint(true)
	, maxPIDOutput(700.0f)
	, maxVelocity(600.0f)
	, maxAngle(45.0f)
	, desiredNewVelocity(FVector::ZeroVector)
	, CumulativeTime(0.0f)
	, MaxPlotTime(10.0f)
	, LastDesiredYaw(0.0f)
	, LastCurrentYaw(0.0f)
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UImGuiUtil::Initialize(AQuadPawn* InPawn, UQuadDroneController* InController, const FVector& InDesiredNewVelocity,
								 bool InDebug_DrawDroneCollisionSphere, bool InDebug_DrawDroneWaypoint,
								 float InMaxPIDOutput, float InMaxVelocity, float InMaxAngle)
{
	DronePawn = InPawn;
	Controller = InController;
	desiredNewVelocity = InDesiredNewVelocity;
	Debug_DrawDroneCollisionSphere = InDebug_DrawDroneCollisionSphere;
	Debug_DrawDroneWaypoint = InDebug_DrawDroneWaypoint;
	maxPIDOutput = InMaxPIDOutput;
	maxVelocity = InMaxVelocity;
	maxAngle = InMaxAngle;
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

	DisplayDesiredVelocities();
	DisplayDebugOptions();

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
	ImGui::Text("Velocity Command Received X, Y, Z: %.2f, %.2f, %.2f", desiredNewVelocity.X, desiredNewVelocity.Y, desiredNewVelocity.Z);
	ImGui::Text("Current Goal State X, Y, Z: %.2f, %.2f, %.2f", currentGoalState.X, currentGoalState.Y, currentGoalState.Z);
	ImGui::Spacing();

	static bool syncXY = false;
	static bool syncRP = false;
	DisplayPIDSettings(EFlightMode::VelocityControl, "PID Settings", syncXY, syncRP);

	ImGui::Separator();
	ImGui::Text("Position PID Outputs");
	ImGui::Text("X Output: %.2f", xOutput);
	ImGui::Text("Y Output: %.2f", yOutput);
	ImGui::Text("Z Output: %.2f", zOutput);
	ImGui::Separator();

	DisplayCameraControls();
	DisplayResetDroneButtons();
	DisplayPIDHistoryWindow();

	ImGui::Separator();

	ImGui::End();

}

void UImGuiUtil::RenderImPlot(const TArray<float>& ThrustsVal, float deltaTime)
{
	// Entry log
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: RenderImPlot started"));

	// Only proceed if we have valid data to plot
	if (!IsValid(DronePawn) || ThrustsVal.Num() < 4)
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Invalid DronePawn or ThrustsVal size (%d)"), ThrustsVal.Num());
		return;
	}

	// Track time for all plots - limit to small increments to avoid floating point issues
	float previousTime = CumulativeTime;
	CumulativeTime += FMath::Min(deltaTime, 0.1f);

	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Time updated: prev=%.3f, delta=%.3f, current=%.3f"),
		previousTime, deltaTime, CumulativeTime);

	// Log array sizes before adding data
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Before adding data - TimeData: %d, Thrust data: %d/%d/%d/%d"),
		TimeData.Num(), Thrust0Data.Num(), Thrust1Data.Num(), Thrust2Data.Num(), Thrust3Data.Num());

	// Store thrust data with safety checks
	TimeData.Add(CumulativeTime);
	Thrust0Data.Add(ThrustsVal[0]);
	Thrust1Data.Add(ThrustsVal[1]);
	Thrust2Data.Add(ThrustsVal[2]);
	Thrust3Data.Add(ThrustsVal[3]);

	UE_LOG(LogTemp, Warning, TEXT("IMGUI: After adding data - TimeData size: %d"), TimeData.Num());

	// Clean up old data based on time window
	int32 removedCount = 0;
	while (TimeData.Num() > 1 && (CumulativeTime - TimeData[0] > MaxPlotTime))
	{
		TimeData.RemoveAt(0);
		Thrust0Data.RemoveAt(0);
		Thrust1Data.RemoveAt(0);
		Thrust2Data.RemoveAt(0);
		Thrust3Data.RemoveAt(0);
		removedCount++;
	}

	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Removed %d old thrust data points"), removedCount);

	// Limit data points to prevent memory issues
	if (TimeData.Num() > MaxDataPoints)
	{
		int32 NumToRemove = TimeData.Num() / 2;
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Removing %d thrust data points (above limit)"), NumToRemove);

		TimeData.RemoveAt(0, NumToRemove);
		Thrust0Data.RemoveAt(0, NumToRemove);
		Thrust1Data.RemoveAt(0, NumToRemove);
		Thrust2Data.RemoveAt(0, NumToRemove);
		Thrust3Data.RemoveAt(0, NumToRemove);
	}

	// --------------------------------
	// Process yaw data
	// --------------------------------
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Starting yaw data processing"));

	bool hasYawData = false;
	float currentYaw = 0.0f;
	float desiredYaw = 0.0f;

	// Check if controller and pawn are valid before processing yaw
	if (!IsValid(Controller))
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Controller is invalid"));
	}

	if (!IsValid(DronePawn))
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: DronePawn is invalid"));
	}

	// Log previous yaw data sizes
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Before adding yaw data - YawTimeData: %d, DesiredYawData: %d, CurrentYawData: %d"),
		YawTimeData.Num(), DesiredYawData.Num(), CurrentYawData.Num());

	if (IsValid(Controller) && IsValid(DronePawn))
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Controller and DronePawn are valid"));

		hasYawData = true;

		// Get yaw values
		desiredYaw = Controller->GetDesiredYaw();
		currentYaw = DronePawn->GetActorRotation().Yaw;

		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Raw yaw values - desired: %.2f, current: %.2f"),
			desiredYaw, currentYaw);

		// Normalize yaw values
		float normalizedDesiredYaw = FRotator::NormalizeAxis(desiredYaw);
		float normalizedCurrentYaw = FRotator::NormalizeAxis(currentYaw);

		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Normalized yaw values - desired: %.2f, current: %.2f"),
			normalizedDesiredYaw, normalizedCurrentYaw);

		// Store yaw data
		YawTimeData.Add(CumulativeTime);
		DesiredYawData.Add(normalizedDesiredYaw);
		CurrentYawData.Add(normalizedCurrentYaw);

		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Added new yaw data point, YawTimeData size now: %d"),
			YawTimeData.Num());

		// Store last values
		LastDesiredYaw = normalizedDesiredYaw;
		LastCurrentYaw = normalizedCurrentYaw;

		// Clean up old yaw data
		int32 yawRemovedCount = 0;
		while (YawTimeData.Num() > 1 && (CumulativeTime - YawTimeData[0] > MaxPlotTime))
		{
			YawTimeData.RemoveAt(0);
			DesiredYawData.RemoveAt(0);
			CurrentYawData.RemoveAt(0);
			yawRemovedCount++;
		}

		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Removed %d old yaw data points"), yawRemovedCount);

		// Limit yaw data points
		if (YawTimeData.Num() > MaxYawDataPoints)
		{
			int32 NumToRemove = YawTimeData.Num() / 2;
			UE_LOG(LogTemp, Warning, TEXT("IMGUI: Removing %d yaw data points (above limit)"), NumToRemove);

			YawTimeData.RemoveAt(0, NumToRemove);
			DesiredYawData.RemoveAt(0, NumToRemove);
			CurrentYawData.RemoveAt(0, NumToRemove);
		}
	}

	// --------------------------------
	// Create ImGui window
	// --------------------------------
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Setting up plot window"));

	ImGui::SetNextWindowPos(ImVec2(850, 10), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(600, 600), ImGuiCond_FirstUseEver);

	// Begin window with flags to handle resizing safely
	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoCollapse;

	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Beginning window"));

	bool windowOpen = ImGui::Begin("Drone Flight Analysis", nullptr, windowFlags);
	if (!windowOpen)
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Window not open, returning"));
		ImGui::End();
		return;
	}

	// Calculate plot sizes
	ImVec2 availableSize = ImGui::GetContentRegionAvail();

	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Available window size: %.1f x %.1f"),
		availableSize.x, availableSize.y);

	float plotHeight = FMath::Max(150.0f, FMath::Min(200.0f, availableSize.y / 3.0f - 10.0f));
	float plotWidth = FMath::Max(200.0f, availableSize.x);
	ImVec2 plotSize(plotWidth, plotHeight);

	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Plot size calculated: %.1f x %.1f"),
		plotSize.x, plotSize.y);

	// Common plot flags
	ImPlotFlags plotFlags = ImPlotFlags_NoBoxSelect | ImPlotFlags_NoMenus;
	ImPlotAxisFlags axisFlags = ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoSideSwitch;

	// --------------------------------
	// Thrust Plot
	// --------------------------------
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Processing thrust plot"));

	if (ImGui::CollapsingHeader("Thrust Values", ImGuiTreeNodeFlags_DefaultOpen))
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Beginning thrust plot"));

		if (ImPlot::BeginPlot("##Thrust Values", plotSize, plotFlags))
		{
			UE_LOG(LogTemp, Warning, TEXT("IMGUI: Thrust plot setup"));

			ImPlot::SetupAxes("Time (s)", "Thrust (N)", axisFlags, axisFlags);
			ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - MaxPlotTime, CumulativeTime, ImGuiCond_Always);
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 700.0f);

			if (CumulativeTime > MaxPlotTime)
			{
				ImPlot::PlotText("Thrust Values Over Time", CumulativeTime - MaxPlotTime + 0.2f, 660.0f);
			}

			UE_LOG(LogTemp, Warning, TEXT("IMGUI: Plotting thrust lines, data size: %d"), TimeData.Num());

			if (TimeData.Num() > 0)
			{
				const ImVec4 FL_COLOR(1.0f, 0.2f, 0.2f, 1.0f);
				const ImVec4 FR_COLOR(0.2f, 1.0f, 0.2f, 1.0f);
				const ImVec4 BL_COLOR(0.2f, 0.6f, 1.0f, 1.0f);
				const ImVec4 BR_COLOR(1.0f, 0.8f, 0.0f, 1.0f);

				ImPlot::SetNextLineStyle(FL_COLOR, 2.0f);
				ImPlot::PlotLine("Front Left", TimeData.GetData(), Thrust0Data.GetData(), TimeData.Num());

				ImPlot::SetNextLineStyle(FR_COLOR, 2.0f);
				ImPlot::PlotLine("Front Right", TimeData.GetData(), Thrust1Data.GetData(), TimeData.Num());

				ImPlot::SetNextLineStyle(BL_COLOR, 2.0f);
				ImPlot::PlotLine("Back Left", TimeData.GetData(), Thrust2Data.GetData(), TimeData.Num());

				ImPlot::SetNextLineStyle(BR_COLOR, 2.0f);
				ImPlot::PlotLine("Back Right", TimeData.GetData(), Thrust3Data.GetData(), TimeData.Num());
			}

			UE_LOG(LogTemp, Warning, TEXT("IMGUI: Ending thrust plot"));
			ImPlot::EndPlot();
		}
	}

	ImGui::Spacing();

	// --------------------------------
	// PID Integral Sums Plot
	// --------------------------------
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Processing PID integral sums plot"));

	if (ImGui::CollapsingHeader("PID Integral Sums", ImGuiTreeNodeFlags_DefaultOpen))
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Beginning PID integral plot"));

		if (ImPlot::BeginPlot("##PID Sums", plotSize, plotFlags))
		{
			ImPlot::SetupAxes("Time (s)", "Integral Sum", axisFlags, axisFlags);
			ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - MaxPlotTime, CumulativeTime, ImGuiCond_Always);

			if (CumulativeTime > MaxPlotTime)
			{
				ImPlot::PlotText("PID Integral Sums", CumulativeTime - MaxPlotTime + 0.2f, 0.0f);
			}

			// PID data processing
			static TArray<float> PIDTimes;
			static TArray<float> XSums;
			static TArray<float> YSums;
			static TArray<float> ZSums;

			// Similar PID data processing code as before
			// ...

			UE_LOG(LogTemp, Warning, TEXT("IMGUI: Ending PID integral plot"));
			ImPlot::EndPlot();
		}
	}

	ImGui::Spacing();

	// --------------------------------
	// Yaw Tracking Plot
	// --------------------------------
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Processing yaw tracking plot"));

	// Check if we have yaw data
	if (!hasYawData)
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: No yaw data available"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Yaw data sizes - Time: %d, Desired: %d, Current: %d"),
			YawTimeData.Num(), DesiredYawData.Num(), CurrentYawData.Num());
	}

	// Only display the collapsing header if we have yaw data
	if (hasYawData && ImGui::CollapsingHeader("Yaw Tracking", ImGuiTreeNodeFlags_DefaultOpen))
	{
		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Yaw header opened"));

		// Verify array sizes match before plotting
		bool validYawArrays = (YawTimeData.Num() > 0 &&
			YawTimeData.Num() == DesiredYawData.Num() &&
			YawTimeData.Num() == CurrentYawData.Num());

		UE_LOG(LogTemp, Warning, TEXT("IMGUI: Yaw arrays valid? %s"),
			validYawArrays ? TEXT("Yes") : TEXT("No"));

		if (!validYawArrays)
		{
			ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f),
				"Yaw data arrays mismatch: Time=%d, Desired=%d, Current=%d",
				YawTimeData.Num(), DesiredYawData.Num(), CurrentYawData.Num());
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("IMGUI: Beginning yaw plot"));

			// Attempt to begin the yaw plot
			if (ImPlot::BeginPlot("##Yaw Tracking", plotSize, plotFlags))
			{
				UE_LOG(LogTemp, Warning, TEXT("IMGUI: Yaw plot setup"));

				ImPlot::SetupAxes("Time (s)", "Yaw (degrees)", axisFlags, axisFlags);
				ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - MaxPlotTime, CumulativeTime, ImGuiCond_Always);
				ImPlot::SetupAxisLimits(ImAxis_Y1, -180, 180);

				// Add a plot title if we have enough data
				if (CumulativeTime > MaxPlotTime)
				{
					UE_LOG(LogTemp, Warning, TEXT("IMGUI: Adding yaw plot title"));
					ImPlot::PlotText("Yaw Tracking Over Time", CumulativeTime - MaxPlotTime + 0.2f, 160.0f);
				}

				UE_LOG(LogTemp, Warning, TEXT("IMGUI: Plotting desired yaw line, data size: %d"), YawTimeData.Num());

				// Plot desired yaw line (orange)
				ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), 2.0f);
				ImPlot::PlotLine("Desired Yaw", YawTimeData.GetData(), DesiredYawData.GetData(), YawTimeData.Num());

				UE_LOG(LogTemp, Warning, TEXT("IMGUI: Plotting current yaw line"));

				// Plot current yaw line (cyan)
				ImPlot::SetNextLineStyle(ImVec4(0.0f, 0.8f, 0.8f, 1.0f), 2.0f);
				ImPlot::PlotLine("Current Yaw", YawTimeData.GetData(), CurrentYawData.GetData(), YawTimeData.Num());

				// Calculate and display current error
				float currentError = FMath::Abs(FMath::FindDeltaAngleDegrees(LastCurrentYaw, LastDesiredYaw));

				UE_LOG(LogTemp, Warning, TEXT("IMGUI: Current yaw error: %.2f"), currentError);

				// Add text annotation showing current error
				if (CumulativeTime > 0.5f)
				{
					UE_LOG(LogTemp, Warning, TEXT("IMGUI: Adding error annotation"));

					char errorText[32];
					snprintf(errorText, sizeof(errorText), "Error: %.2f°", currentError);

					// Safely clamp the Y position to avoid rendering out of bounds
					float annotationY = FMath::Clamp(LastCurrentYaw + 10.0f, -170.0f, 170.0f);

					UE_LOG(LogTemp, Warning, TEXT("IMGUI: Error annotation position: time=%.2f, y=%.2f"),
						CumulativeTime - 0.5f, annotationY);

					ImPlot::PlotText(errorText, CumulativeTime - 0.5f, annotationY);
				}

				UE_LOG(LogTemp, Warning, TEXT("IMGUI: Ending yaw plot"));
				ImPlot::EndPlot();
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("IMGUI: Failed to begin yaw plot"));
				ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Failed to create yaw plot");
			}

			// Display error statistics outside of the plot
			UE_LOG(LogTemp, Warning, TEXT("IMGUI: Displaying yaw statistics"));

			float currentError = FMath::Abs(FMath::FindDeltaAngleDegrees(LastCurrentYaw, LastDesiredYaw));
			ImGui::Text("Current Yaw Error: %.2f°", currentError);

			if (YawTimeData.Num() > 1)
			{
				UE_LOG(LogTemp, Warning, TEXT("IMGUI: Calculating average yaw error"));

				float totalError = 0.0f;
				for (int32 i = 0; i < DesiredYawData.Num(); i++)
				{
					totalError += FMath::Abs(FMath::FindDeltaAngleDegrees(CurrentYawData[i], DesiredYawData[i]));
				}
				float avgError = totalError / DesiredYawData.Num();
				ImGui::Text("Average Yaw Error: %.2f°", avgError);
			}
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("IMGUI: Ending render window"));
	ImGui::End();
	UE_LOG(LogTemp, Warning, TEXT("IMGUI: RenderImPlot completed"));
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

void UImGuiUtil::DisplayDebugOptions()
{
    ImGui::Separator();
    ImGui::Text("Debug Draw");
    ImGui::Checkbox("Drone Collision Sphere", &Debug_DrawDroneCollisionSphere);
    ImGui::Checkbox("Drone Waypoint", &Debug_DrawDroneWaypoint);
    ImGui::Separator();
}

void UImGuiUtil::DisplayPIDSettings(EFlightMode Mode, const char* headerLabel, bool& synchronizeXYGains, bool& synchronizeGains)
{
	FFullPIDSet* PIDSet = Controller ? Controller->GetPIDSet(Mode) : nullptr;
	if (!PIDSet)
	{
		ImGui::Text("No PID Set found for this mode.");
		return;
	}
	if (ImGui::CollapsingHeader(headerLabel, ImGuiTreeNodeFlags_DefaultOpen))
	{
		auto DrawPIDGainControl = [](const char* label, float* value, float minValue, float maxValue)
		{
			float totalWidth = ImGui::GetContentRegionAvail().x;
			float inputWidth = 80.0f;
			float sliderWidth = totalWidth - inputWidth - 20.0f;

			ImGui::PushItemWidth(sliderWidth);
			ImGui::SliderFloat(label, value, minValue, maxValue);
			ImGui::PopItemWidth();

			ImGui::SameLine();

			ImGui::PushItemWidth(inputWidth);
			ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1));
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));
			std::string inputLabel = std::string("##") + label;
			ImGui::InputFloat(inputLabel.c_str(), value, 0.0f, 0.0f, "%.3f");
			ImGui::PopStyleColor(2);
			ImGui::PopItemWidth();
		};

		ImGui::Text("Position PID Gains");
		ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);
		ImGui::Indent();
		ImGui::Text("X Axis");
		if (synchronizeXYGains && PIDSet->XPID && PIDSet->YPID)
		{
			DrawPIDGainControl("X P", &PIDSet->XPID->ProportionalGain, 0.0f, 10.0f);
			PIDSet->YPID->ProportionalGain = PIDSet->XPID->ProportionalGain;
			DrawPIDGainControl("X I", &PIDSet->XPID->IntegralGain, 0.0f, 10.0f);
			PIDSet->YPID->IntegralGain = PIDSet->XPID->IntegralGain;
			DrawPIDGainControl("X D", &PIDSet->XPID->DerivativeGain, 0.0f, 10.0f);
			PIDSet->YPID->DerivativeGain = PIDSet->XPID->DerivativeGain;
		}
		else if (PIDSet->XPID)
		{
			DrawPIDGainControl("X P", &PIDSet->XPID->ProportionalGain, 0.0f, 10.0f);
			DrawPIDGainControl("X I", &PIDSet->XPID->IntegralGain, 0.0f, 10.0f);
			DrawPIDGainControl("X D", &PIDSet->XPID->DerivativeGain, 0.0f, 10.0f);
		}
		ImGui::Unindent();

		ImGui::Indent();
		ImGui::Text("Y Axis");
		if (synchronizeXYGains && PIDSet->YPID && PIDSet->XPID)
		{
			DrawPIDGainControl("Y P", &PIDSet->YPID->ProportionalGain, 0.0f, 10.0f);
			PIDSet->XPID->ProportionalGain = PIDSet->YPID->ProportionalGain;
			DrawPIDGainControl("Y I", &PIDSet->YPID->IntegralGain, 0.0f, 10.0f);
			PIDSet->XPID->IntegralGain = PIDSet->YPID->IntegralGain;
			DrawPIDGainControl("Y D", &PIDSet->YPID->DerivativeGain, 0.0f, 10.0f);
			PIDSet->XPID->DerivativeGain = PIDSet->YPID->DerivativeGain;
		}
		else if (PIDSet->YPID)
		{
			DrawPIDGainControl("Y P", &PIDSet->YPID->ProportionalGain, 0.0f, 10.0f);
			DrawPIDGainControl("Y I", &PIDSet->YPID->IntegralGain, 0.0f, 10.0f);
			DrawPIDGainControl("Y D", &PIDSet->YPID->DerivativeGain, 0.0f, 10.0f);
		}
		ImGui::Unindent();

		ImGui::Indent();
		ImGui::Text("Z Axis");
		if (PIDSet->ZPID)
		{
			DrawPIDGainControl("Z P", &PIDSet->ZPID->ProportionalGain, 0.0f, 10.0f);
			DrawPIDGainControl("Z I", &PIDSet->ZPID->IntegralGain, 0.0f, 10.0f);
			DrawPIDGainControl("Z D", &PIDSet->ZPID->DerivativeGain, 0.0f, 10.0f);
		}
		ImGui::Unindent();

		ImGui::Separator();

		ImGui::Text("Attitude PID Gains");
		ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeGains);
		ImGui::Indent();
		ImGui::Text("Roll");
		if (synchronizeGains && PIDSet->RollPID && PIDSet->PitchPID)
		{
			DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, 0.0f, 20.0f);
			PIDSet->PitchPID->ProportionalGain = PIDSet->RollPID->ProportionalGain;
			DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, 0.0f, 20.0f);
			PIDSet->PitchPID->IntegralGain = PIDSet->RollPID->IntegralGain;
			DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, 0.0f, 20.0f);
			PIDSet->PitchPID->DerivativeGain = PIDSet->RollPID->DerivativeGain;
		}
		else if (PIDSet->RollPID)
		{
			DrawPIDGainControl("Roll P", &PIDSet->RollPID->ProportionalGain, 0.0f, 20.0f);
			DrawPIDGainControl("Roll I", &PIDSet->RollPID->IntegralGain, 0.0f, 20.0f);
			DrawPIDGainControl("Roll D", &PIDSet->RollPID->DerivativeGain, 0.0f, 20.0f);
		}
		ImGui::Unindent();

		ImGui::Indent();
		ImGui::Text("Pitch");
		if (synchronizeGains && PIDSet->PitchPID && PIDSet->RollPID)
		{
			DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, 0.0f, 20.0f);
			PIDSet->RollPID->ProportionalGain = PIDSet->PitchPID->ProportionalGain;
			DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, 0.0f, 20.0f);
			PIDSet->RollPID->IntegralGain = PIDSet->PitchPID->IntegralGain;
			DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, 0.0f, 20.0f);
			PIDSet->RollPID->DerivativeGain = PIDSet->PitchPID->DerivativeGain;
		}
		else if (PIDSet->PitchPID)
		{
			DrawPIDGainControl("Pitch P", &PIDSet->PitchPID->ProportionalGain, 0.0f, 20.0f);
			DrawPIDGainControl("Pitch I", &PIDSet->PitchPID->IntegralGain, 0.0f, 20.0f);
			DrawPIDGainControl("Pitch D", &PIDSet->PitchPID->DerivativeGain, 0.0f, 20.0f);
		}
		ImGui::Unindent();

		ImGui::Indent();
		ImGui::Text("Yaw");
		if (PIDSet->YawPID)
		{
			DrawPIDGainControl("Yaw P", &PIDSet->YawPID->ProportionalGain, 0.0f, 2.0f);
			DrawPIDGainControl("Yaw I", &PIDSet->YawPID->IntegralGain, 0.0f, 2.0f);
			DrawPIDGainControl("Yaw D", &PIDSet->YawPID->DerivativeGain, 0.0f, 2.0f);
		}
		ImGui::Unindent();

		ImGui::Separator();

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
			if (PIDSet->XPID)
			{
				GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->XPID->ProportionalGain, PIDSet->XPID->IntegralGain, PIDSet->XPID->DerivativeGain);
			}
			if (PIDSet->YPID)
			{
				GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->YPID->ProportionalGain, PIDSet->YPID->IntegralGain, PIDSet->YPID->DerivativeGain);
			}
			if (PIDSet->ZPID)
			{
				GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->ZPID->ProportionalGain, PIDSet->ZPID->IntegralGain, PIDSet->ZPID->DerivativeGain);
			}
			if (PIDSet->RollPID)
			{
				GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->RollPID->ProportionalGain, PIDSet->RollPID->IntegralGain, PIDSet->RollPID->DerivativeGain);
			}
			if (PIDSet->PitchPID)
			{
				GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), PIDSet->PitchPID->ProportionalGain, PIDSet->PitchPID->IntegralGain, PIDSet->PitchPID->DerivativeGain);
			}
			if (PIDSet->YawPID)
			{
				GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f"), PIDSet->YawPID->ProportionalGain, PIDSet->YawPID->IntegralGain, PIDSet->YawPID->DerivativeGain);
			}
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

	// Static variables to hold previous slider values.
	static float prevVx = 0.0f;
	static float prevVy = 0.0f;
	static float prevVz = 0.0f;
	static bool firstRun = true;
	static bool hoverModeActive = false;  // Hover mode flag

	// Reset checkboxes states (we need separate variables for these)
	static bool resetXChecked = false;
	static bool resetYChecked = false;
	static bool resetZChecked = false;

	float tempVx = desiredNewVelocity.X;
	float tempVy = desiredNewVelocity.Y;
	float tempVz = desiredNewVelocity.Z;
	bool velocityChanged = false;

	// Add hover mode button with distinctive styling
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.6f, 0.8f, 1.0f)); // Blue button
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.7f, 0.9f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.5f, 0.7f, 1.0f));

	if (ImGui::Button(hoverModeActive ? "HOVER MODE ACTIVE" : "ACTIVATE HOVER MODE", ImVec2(200, 35)))
	{
		hoverModeActive = !hoverModeActive;
		if (hoverModeActive)
		{
			// Set Z velocity to 70 when activating hover mode
			tempVz = 70.0f;
			velocityChanged = true;
		}
	}
	ImGui::PopStyleColor(3);

	if (hoverModeActive)
	{
		ImGui::SameLine();
		ImGui::TextColored(ImVec4(0.1f, 0.6f, 0.8f, 1.0f), "Z-velocity locked at 70.0");
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

		// In hover mode, Z velocity is always 70
		tempVz = 70.0f;
	}

	// On first run, initialize previous values.
	if (firstRun)
	{
		prevVx = tempVx;
		prevVy = tempVy;
		prevVz = tempVz;
		firstRun = false;
	}

	// Set a deadzone threshold (adjust as needed)
	const float threshold = 0.01f;
	bool significantChange = (FMath::Abs(tempVx - prevVx) > threshold) ||
		(FMath::Abs(tempVy - prevVy) > threshold) ||
		(FMath::Abs(tempVz - prevVz) > threshold);

	// Only update the desired velocity if there's a significant change or if we just entered hover mode
	if (significantChange || velocityChanged)
	{
		desiredNewVelocity = FVector(tempVx, tempVy, tempVz);
		if (Controller)
		{
			Controller->SetDesiredVelocity(desiredNewVelocity);
		}
		// Update previous values so that subsequent small changes are ignored.
		prevVx = tempVx;
		prevVy = tempVy;
		prevVz = tempVz;
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

	FFullPIDSet* PIDSet = Controller->GetPIDSet(Controller->GetFlightMode());
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