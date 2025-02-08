#include "UI/ImGuiUtil.h"
#include "imgui.h"
#include "implot.h"
#include "Pawns/QuadPawn.h"
#include "string"
#include "Controllers/QuadDroneController.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/DateTime.h"

UImGuiUtil::UImGuiUtil()
    : DronePawn(nullptr)
    , Controller(nullptr)
    , Debug_DrawDroneCollisionSphere(true)
    , Debug_DrawDroneWaypoint(true)
    , maxPIDOutput(100.0f)
    , maxVelocity(600.0f)
    , maxAngle(45.0f)
    , desiredNewVelocity(FVector::ZeroVector)
    , CumulativeTime(0.0f)
    , MaxPlotTime(10.0f)
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

	UZMQController* zmqControllerCurrent = DronePawn ? DronePawn->FindComponentByClass<UZMQController>() : nullptr;
	FVector currentGoalState = FVector::ZeroVector;
	if (zmqControllerCurrent && zmqControllerCurrent->IsValidLowLevel())
	{
		currentGoalState = zmqControllerCurrent->GetCurrentGoalPosition();
	}

	ImGui::Begin("Drone Controller", nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar);

	// Display drone info and various UI elements
	DisplayDroneInfo();
	ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, 600.0f);
	ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);

	DisplayDesiredVelocities();
	DisplayDebugOptions();

	static float AllThrustValue = 0.0f;
	if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, 0, maxPIDOutput))
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
		ImGui::Indent();
		if (synchronizeDiagonal1)
		{
			if (ImGui::SliderFloat("FL & BR Thrust", &ThrustsVal[0], 0, maxPIDOutput))
			{
				ThrustsVal[3] = ThrustsVal[0];
			}
			ImGui::Text("Back Right (Synchronized): %.2f", ThrustsVal[3]);
		}
		else
		{
			ImGui::SliderFloat("Front Left", &ThrustsVal[0], 0, maxPIDOutput);
			ImGui::SliderFloat("Back Right", &ThrustsVal[3], 0, maxPIDOutput);
		}
		ImGui::Unindent();

		ImGui::Text("Diagonal 2 Motors");
		ImGui::Indent();
		if (synchronizeDiagonal2)
		{
			if (ImGui::SliderFloat("FR & BL Thrust", &ThrustsVal[1], 0, maxPIDOutput))
			{
				ThrustsVal[2] = ThrustsVal[1];
			}
			ImGui::Text("Back Left (Synchronized): %.2f", ThrustsVal[2]);
		}
		else
		{
			ImGui::SliderFloat("Front Right", &ThrustsVal[1], 0, maxPIDOutput);
			ImGui::SliderFloat("Back Left", &ThrustsVal[2], 0, maxPIDOutput);
		}
		ImGui::Unindent();
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

	ImGui::End();
}


void UImGuiUtil::RenderImPlot(const TArray<float>& ThrustsVal, float deltaTime)
{
	if (ThrustsVal.Num() < 4)
	{
		return;
	}

	CumulativeTime += deltaTime;
	TimeData.Add(CumulativeTime);
	Thrust0Data.Add(ThrustsVal[0]);
	Thrust1Data.Add(ThrustsVal[1]);
	Thrust2Data.Add(ThrustsVal[2]);
	Thrust3Data.Add(ThrustsVal[3]);

	while (TimeData.Num() > 0 && (CumulativeTime - TimeData[0] > MaxPlotTime))
	{
		TimeData.RemoveAt(0);
		Thrust0Data.RemoveAt(0);
		Thrust1Data.RemoveAt(0);
		Thrust2Data.RemoveAt(0);
		Thrust3Data.RemoveAt(0);
	}

	while (TimeData.Num() > MaxDataPoints)
	{
		TimeData.RemoveAt(0);
		Thrust0Data.RemoveAt(0);
		Thrust1Data.RemoveAt(0);
		Thrust2Data.RemoveAt(0);
		Thrust3Data.RemoveAt(0);
	}

	ImGui::SetNextWindowPos(ImVec2(850, 10), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);

	ImGui::Begin("Drone Thrust Analysis", nullptr, ImGuiWindowFlags_NoCollapse);

	ImVec2 windowSize = ImGui::GetContentRegionAvail();
	ImPlotFlags plotFlags = ImPlotFlags_None;
	ImPlotAxisFlags axisFlags = ImPlotAxisFlags_None;

	if (ImPlot::BeginPlot("Thrust Values Over Time", windowSize, plotFlags))
	{
		ImPlot::SetupAxes("Time (s)", "Thrust (N)", axisFlags, axisFlags);

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

		ImPlot::EndPlot();
	}

	if (ImPlot::BeginPlot("PID Integral Sums"))
	{
		ImPlot::SetupAxes("Time (s)", "Integral Sum");

		FFullPIDSet* CurrentSet = Controller ? Controller->GetPIDSet(Controller->GetFlightMode()) : nullptr;
		if (CurrentSet)
		{
			static TArray<float> Times;
			static TArray<float> XSums;
			static TArray<float> YSums;
			static TArray<float> ZSums;

			Times.Add(CumulativeTime);
			XSums.Add(CurrentSet->XPID->GetCurrentBufferSum());
			YSums.Add(CurrentSet->YPID->GetCurrentBufferSum());
			ZSums.Add(CurrentSet->ZPID->GetCurrentBufferSum());

			const int32 MaxPoints = 1000;
			if (Times.Num() > MaxPoints)
			{
				Times.RemoveAt(0);
				XSums.RemoveAt(0);
				YSums.RemoveAt(0);
				ZSums.RemoveAt(0);
			}

			ImPlot::PlotLine("X Integral", Times.GetData(), XSums.GetData(), Times.Num());
			ImPlot::PlotLine("Y Integral", Times.GetData(), YSums.GetData(), Times.Num());
			ImPlot::PlotLine("Z Integral", Times.GetData(), ZSums.GetData(), Times.Num());
		}

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
	float tempVx = desiredNewVelocity.X;
	float tempVy = desiredNewVelocity.Y;
	float tempVz = desiredNewVelocity.Z;
	bool velocityChanged = false;
	velocityChanged |= ImGui::SliderFloat("Desired Velocity X", &tempVx, -maxVelocity, maxVelocity);
	velocityChanged |= ImGui::SliderFloat("Desired Velocity Y", &tempVy, -maxVelocity, maxVelocity);
	velocityChanged |= ImGui::SliderFloat("Desired Velocity Z", &tempVz, -maxVelocity, maxVelocity);
	if (velocityChanged)
	{
		desiredNewVelocity.X = tempVx;
		desiredNewVelocity.Y = tempVy;
		desiredNewVelocity.Z = tempVz;
		if (Controller)
		{
			Controller->SetDesiredVelocity(desiredNewVelocity);
		}
	}
	ImGui::Separator();
}
