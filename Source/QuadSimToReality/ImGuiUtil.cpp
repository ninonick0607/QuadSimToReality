#include "ImGuiUtil.h"
#include "imgui.h"
#include "implot.h"
#include "QuadPawn.h"
#include "string"
#include "QuadDroneController.h" 

ImGuiUtil::ImGuiUtil(
    AQuadPawn* InPawn,
    QuadDroneController* InController,
    FVector IndesiredNewVelocity,
    bool& InInitialTakeoff,
    bool& InAltitudeReached,
    bool& InDebug_DrawDroneCollisionSphere,
    bool& InDebug_DrawDroneWaypoint,
    float InMaxPIDOutput,
    float InAltitudeThresh,
    float InMinAltitudeLocal,
    float& InMaxVelocity,
    float& InMaxAngle,
    QuadPIDController* InXPID,
    QuadPIDController* InYPID,
    QuadPIDController* InZPID,
    QuadPIDController* InRollAttitudePID,
    QuadPIDController* InPitchAttitudePID,
    QuadPIDController* InYawAttitudePID,

    QuadPIDController* InxPIDVelocity,
    QuadPIDController* InyPIDVelocity,
    QuadPIDController* InzPIDVelocity,
    QuadPIDController* InrollAttitudePIDVelocity,
    QuadPIDController* InpitchAttitudePIDVelocity,
    QuadPIDController* InyawAttitudePIDVelocity,

    QuadPIDController* InxPIDJoyStick,
    QuadPIDController* InyPIDJoyStick,
    QuadPIDController* InzPIDJoyStick,
    QuadPIDController* InrollAttitudePIDJoyStick,
    QuadPIDController* InpitchAttitudePIDJoyStick,
    QuadPIDController* InyawAttitudePIDJoyStick
)
    : dronePawn(InPawn)
    , controller(InController)
    , desiredNewVelocity(IndesiredNewVelocity)
    , initialTakeoff(InInitialTakeoff)
    , altitudeReached(InAltitudeReached)
    , Debug_DrawDroneCollisionSphere(InDebug_DrawDroneCollisionSphere)
    , Debug_DrawDroneWaypoint(InDebug_DrawDroneWaypoint)
    , maxPIDOutput(InMaxPIDOutput)
    , altitudeThresh(InAltitudeThresh)
    , minAltitudeLocal(InMinAltitudeLocal)
    , maxVelocity(InMaxVelocity)
    , maxAngle(InMaxAngle)
    , xPID(InXPID)
    , yPID(InYPID)
    , zPID(InZPID)
    , rollAttitudePID(InRollAttitudePID)
    , pitchAttitudePID(InPitchAttitudePID)
    , yawAttitudePID(InYawAttitudePID)
    , xPIDVelocity(InxPIDVelocity)
    , yPIDVelocity(InyPIDVelocity)
    , zPIDVelocity(InzPIDVelocity)
    , rollAttitudePIDVelocity(InrollAttitudePIDVelocity)
    , pitchAttitudePIDVelocity(InpitchAttitudePIDVelocity)
    , yawAttitudePIDVelocity(InyawAttitudePIDVelocity)
    , xPIDJoyStick(InxPIDJoyStick)
    , yPIDJoyStick(InyPIDJoyStick)
    , zPIDJoyStick(InzPIDJoyStick)
    , rollAttitudePIDJoyStick(InrollAttitudePIDJoyStick)
    , pitchAttitudePIDJoyStick(InpitchAttitudePIDJoyStick)
    , yawAttitudePIDJoyStick(InyawAttitudePIDJoyStick)
{
}

ImGuiUtil::~ImGuiUtil()
{
}


void ImGuiUtil::RenderImPlot(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error, const FVector& desiredVelocity,
    const FVector& currentVelocity,
    float xOutput, float yOutput, float zOutput,
    float rollOutput, float pitchOutput, float yawOutput,
    float deltaTime)
{
    // Add time data
    CumulativeTime += deltaTime;
    TimeData.Add(CumulativeTime);

    // Add PID outputs to history
    xPIDOutputHistory.Add(xOutput);
    yPIDOutputHistory.Add(yOutput);
    zPIDOutputHistory.Add(zOutput);
    rollPIDOutputHistory.Add(rollOutput);
    pitchPIDOutputHistory.Add(pitchOutput);
    yawPIDOutputHistory.Add(yawOutput);

    // Add error calculations
    float positionError = error.Size();  // Magnitude of position error
    float velocityError = (desiredVelocity - currentVelocity).Size();  // Magnitude of velocity error

    positionErrorHistory.Add(positionError);
    velocityErrorHistory.Add(velocityError);

    // Keep arrays within the MaxSize
    const int MaxSize = 1000; // Reduced from 5000 to prevent excessive memory usage

    auto TrimArrayToSize = [MaxSize](TArray<float>& array) {
        if (array.Num() > MaxSize) {
            array.RemoveAt(0, array.Num() - MaxSize);
        }
        };

    TrimArrayToSize(TimeData);
    TrimArrayToSize(xPIDOutputHistory);
    TrimArrayToSize(yPIDOutputHistory);
    TrimArrayToSize(zPIDOutputHistory);
    TrimArrayToSize(rollPIDOutputHistory);
    TrimArrayToSize(pitchPIDOutputHistory);
    TrimArrayToSize(yawPIDOutputHistory);
    TrimArrayToSize(positionErrorHistory);
    TrimArrayToSize(velocityErrorHistory);

    // Begin ImGui window
    ImGui::Begin("Drone PID Analysis");

    // Position Control Plot
    if (ImPlot::BeginPlot("Position Control", ImVec2(600, 300))) {
        ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - 10.0f, CumulativeTime, ImGuiCond_Always);
        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
        ImPlot::SetupAxis(ImAxis_Y1, "Output");

        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
        ImPlot::PlotLine("X PID", TimeData.GetData(), xPIDOutputHistory.GetData(), TimeData.Num());
        ImPlot::PopStyleColor();

        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
        ImPlot::PlotLine("Y PID", TimeData.GetData(), yPIDOutputHistory.GetData(), TimeData.Num());
        ImPlot::PopStyleColor();

        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
        ImPlot::PlotLine("Z PID", TimeData.GetData(), zPIDOutputHistory.GetData(), TimeData.Num());
        ImPlot::PopStyleColor();

        ImPlot::EndPlot();
    }

    // Attitude Control Plot
    if (ImPlot::BeginPlot("Attitude Control", ImVec2(600, 300))) {
        ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - 10.0f, CumulativeTime, ImGuiCond_Always);
        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
        ImPlot::SetupAxis(ImAxis_Y1, "Output");

        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.5f, 0.0f, 1.0f));
        ImPlot::PlotLine("Roll PID", TimeData.GetData(), rollPIDOutputHistory.GetData(), TimeData.Num());
        ImPlot::PopStyleColor();

        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.5f, 0.0f, 1.0f, 1.0f));
        ImPlot::PlotLine("Pitch PID", TimeData.GetData(), pitchPIDOutputHistory.GetData(), TimeData.Num());
        ImPlot::PopStyleColor();

        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
        ImPlot::PlotLine("Yaw PID", TimeData.GetData(), yawPIDOutputHistory.GetData(), TimeData.Num());
        ImPlot::PopStyleColor();

        ImPlot::EndPlot();
    }

    // Error Plot
    if (ImPlot::BeginPlot("Error Analysis", ImVec2(600, 300))) {
        ImPlot::SetupAxisLimits(ImAxis_X1, CumulativeTime - 10.0f, CumulativeTime, ImGuiCond_Always);
        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
        ImPlot::SetupAxis(ImAxis_Y1, "Error Magnitude");

        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.0f, 1.0f, 1.0f));
        ImPlot::PlotLine("Position Error", TimeData.GetData(), positionErrorHistory.GetData(), TimeData.Num());
        ImPlot::PopStyleColor();

        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.0f, 1.0f, 1.0f, 1.0f));
        ImPlot::PlotLine("Velocity Error", TimeData.GetData(), velocityErrorHistory.GetData(), TimeData.Num());
        ImPlot::PopStyleColor();

        ImPlot::EndPlot();
    }

    ImGui::End();
}

void ImGuiUtil::AutoWaypointHud(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error, const FVector& desiredVelocity,
    const FVector& currentVelocity,
    float xOutput, float yOutput, float zOutput, float deltaTime)
{
    ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver); // Reduced window size

    ImGui::Begin("Drone Controller", nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar); // Enable vertical scroll

    // Basic Model Feedback
    ImGui::Text("Drone Model Feedback");
    float droneMass = dronePawn ? dronePawn->DroneBody->GetMass() : 0.0f;
    ImGui::Text("Drone Mass: %.2f kg", droneMass);
    // Add more model feedback as needed
    ImGui::Separator();

    ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, 600.0f);
    ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);

    ImGui::Separator();
    ImGui::Text("Debug Draw");
    ImGui::Checkbox("Drone Collision Sphere", &Debug_DrawDroneCollisionSphere);
    ImGui::Checkbox("Drone Waypoint", &Debug_DrawDroneWaypoint);
    ImGui::Separator();

    ImGui::Separator();
    ImGui::Text("Control All Thrusts");

    static float AllThrustValue = 0.0f; // Initial thrust value

    if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, 0, maxPIDOutput))
    {
        // Set all thrusts to the value from the slider
        for (int i = 0; i < dronePawn->Thrusters.Num(); ++i)
        {
            dronePawn->Thrusters[i]->ThrustStrength = AllThrustValue;
        }
    }

    // Thruster Power
    ImGui::Separator();
    ImGui::Text("Thruster Power");

    // Add synchronization checkboxes for diagonals
    static bool synchronizeDiagonal1 = false;
    static bool synchronizeDiagonal2 = false;

    ImGui::Checkbox("Synchronize Diagonal Motors FL & BR", &synchronizeDiagonal1);
    ImGui::Checkbox("Synchronize Diagonal Motors FR & BL", &synchronizeDiagonal2);

    ImGui::Separator();

    if (ThrustsVal.Num() >= 4)
    {
        // Diagonal 1: Front Left (FL - index 0) and Back Right (BR - index 3)
        ImGui::Text("Diagonal 1 Motors");
        ImGui::Indent();
        if (synchronizeDiagonal1)
        {
            // Use a single slider to control both motors
            if (ImGui::SliderFloat("FL & BR Thrust", &ThrustsVal[0], 0, maxPIDOutput))
            {
                // Set both FL and BR to the same value
                ThrustsVal[3] = ThrustsVal[0];
            }
            // Display the synchronized value for BR
            ImGui::Text("Back Right (Synchronized): %.2f", ThrustsVal[3]);
        }
        else
        {
            // Individual control for FL and BR
            ImGui::SliderFloat("Front Left", &ThrustsVal[0], 0, maxPIDOutput);
            ImGui::SliderFloat("Back Right", &ThrustsVal[3], 0, maxPIDOutput);
        }
        ImGui::Unindent();

        // Diagonal 2: Front Right (FR - index 1) and Back Left (BL - index 2)
        ImGui::Text("Diagonal 2 Motors");
        ImGui::Indent();
        if (synchronizeDiagonal2)
        {
            // Use a single slider to control both motors
            if (ImGui::SliderFloat("FR & BL Thrust", &ThrustsVal[1], 0, maxPIDOutput))
            {
                // Set both FR and BL to the same value
                ThrustsVal[2] = ThrustsVal[1];
            }
            // Display the synchronized value for BL
            ImGui::Text("Back Left (Synchronized): %.2f", ThrustsVal[2]);
        }
        else
        {
            // Individual control for FR and BL
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
    ImGui::Text("Desired Position X, Y, Z: %.2f, %.2f, %.2f", waypoint.X, waypoint.Y, waypoint.Z);
    ImGui::Text("Current Position X, Y, Z: %.2f, %.2f, %.2f", currLoc.X, currLoc.Y, currLoc.Z);
    ImGui::Text("Position Error X, Y, Z: %.2f, %.2f, %.2f", error.X, error.Y, error.Z);
    ImGui::Spacing();
    ImGui::Text("Desired Velocity X, Y, Z: %.2f, %.2f, %.2f", desiredVelocity.X, desiredVelocity.Y, desiredVelocity.Z);
    ImGui::Text("Current Velocity X, Y, Z: %.2f, %.2f, %.2f", currentVelocity.X, currentVelocity.Y, currentVelocity.Z);
    ImGui::Spacing();

    // PID Settings Section
    if (ImGui::CollapsingHeader("PID Settings", ImGuiTreeNodeFlags_DefaultOpen))
    {
        bool tunerActive = controller->bZNTuningInProgress;
        if (tunerActive) {
            ImGui::BeginDisabled(); // Make PID sliders read-only
        }

        auto DrawPIDGainControl = [](const char* label, float* value, float minValue, float maxValue)
            {
                float totalWidth = ImGui::GetContentRegionAvail().x;
                float inputWidth = 80.0f;          // Width of the InputFloat
                float sliderWidth = totalWidth - inputWidth - 20.0f; // Adjust 20.0f for padding

                // Set width for SliderFloat
                ImGui::PushItemWidth(sliderWidth);
                ImGui::SliderFloat(label, value, minValue, maxValue);
                ImGui::PopItemWidth();

                // Place InputFloat next to SliderFloat
                ImGui::SameLine();

                // Set width for InputFloat
                ImGui::PushItemWidth(inputWidth);

                // Set background color to black and text color to white
                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1)); // Black background
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));    // White text

                // Use a hidden label for InputFloat to avoid label collision
                std::string inputLabel = std::string("##") + label;
                ImGui::InputFloat(inputLabel.c_str(), value, 0.0f, 0.0f, "%.3f");

                // Restore styles and widths
                ImGui::PopStyleColor(2);
                ImGui::PopItemWidth();
            };
        // Position PID Gains
        ImGui::Text("Position PID Gains");

        // Add a checkbox for synchronized control for X and Y axes
        static bool synchronizeXYGains = false;
        ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);

        // X Axis
        ImGui::Indent();
        ImGui::Text("X Axis");

        if (synchronizeXYGains) {
            // Synchronized control: adjusting X also adjusts Y
            DrawPIDGainControl("X P", &xPID->ProportionalGain, 0.0f, 10.0f);
            yPID->ProportionalGain = xPID->ProportionalGain;

            DrawPIDGainControl("X I", &xPID->IntegralGain, 0.0f, 10.0f);
            yPID->IntegralGain = xPID->IntegralGain;

            DrawPIDGainControl("X D", &xPID->DerivativeGain, 0.0f, 10.0f);
            yPID->DerivativeGain = xPID->DerivativeGain;
        }
        else {
            // Individual X control
            DrawPIDGainControl("X P", &xPID->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("X I", &xPID->IntegralGain, 0.0f, 10.0f); //*****
            DrawPIDGainControl("X D", &xPID->DerivativeGain, 0.0f, 10.0f);
        }
        ImGui::Unindent();

        // Y Axis
        ImGui::Indent();
        ImGui::Text("Y Axis");

        if (synchronizeXYGains) {
            // Display Y sliders, synchronized with X values
            DrawPIDGainControl("Y P", &yPID->ProportionalGain, 0.0f, 10.0f);
            xPID->ProportionalGain = yPID->ProportionalGain;

            DrawPIDGainControl("Y I", &yPID->IntegralGain, 0.0f, 10.0f);
            xPID->IntegralGain = yPID->IntegralGain;

            DrawPIDGainControl("Y D", &yPID->DerivativeGain, 0.0f, 10.0f);
            xPID->DerivativeGain = yPID->DerivativeGain;
        }
        else {
            // Individual Y control
            DrawPIDGainControl("Y P", &yPID->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("Y I", &yPID->IntegralGain, 0.0f, 10.0f);
            DrawPIDGainControl("Y D", &yPID->DerivativeGain, 0.0f, 10.0f);
        }
        ImGui::Unindent();

        // Z Axis
        ImGui::Indent();
        ImGui::Text("Z Axis");
        DrawPIDGainControl("Z P", &zPID->ProportionalGain, 0.0f, 10.0f);
        DrawPIDGainControl("Z I", &zPID->IntegralGain, 0.0f, 10.0f);
        DrawPIDGainControl("Z D", &zPID->DerivativeGain, 0.0f, 10.0f);
        ImGui::Unindent();

        ImGui::Separator();

        // Attitude PID Gains
        ImGui::Text("Attitude PID Gains");

        static bool synchronizeGains = false;
        ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeGains);

        // Roll
        ImGui::Indent();
        ImGui::Text("Roll");

        if (synchronizeGains) {
            // Synchronized control: adjusting Roll also adjusts Pitch
            DrawPIDGainControl("Roll P", &rollAttitudePID->ProportionalGain, 0.0f, 20.0f);
            pitchAttitudePID->ProportionalGain = rollAttitudePID->ProportionalGain;

            DrawPIDGainControl("Roll I", &rollAttitudePID->IntegralGain, 0.0f, 20.0f);
            pitchAttitudePID->IntegralGain = rollAttitudePID->IntegralGain;

            DrawPIDGainControl("Roll D", &rollAttitudePID->DerivativeGain, 0.0f, 20.0f);
            pitchAttitudePID->DerivativeGain = rollAttitudePID->DerivativeGain;
        }
        else {
            // Individual Roll control
            DrawPIDGainControl("Roll P", &rollAttitudePID->ProportionalGain, 0.0f, 20.0f);
            DrawPIDGainControl("Roll I", &rollAttitudePID->IntegralGain, 0.0f, 20.0f);
            DrawPIDGainControl("Roll D", &rollAttitudePID->DerivativeGain, 0.0f, 20.0f);
        }
        ImGui::Unindent();

        // Pitch
        ImGui::Indent();
        ImGui::Text("Pitch");

        if (synchronizeGains) {
            // Display Pitch sliders, synchronized with Roll values
            DrawPIDGainControl("Pitch P", &pitchAttitudePID->ProportionalGain, 0.0f, 20.0f);
            rollAttitudePID->ProportionalGain = pitchAttitudePID->ProportionalGain;

            DrawPIDGainControl("Pitch I", &pitchAttitudePID->IntegralGain, 0.0f, 20.0f);
            rollAttitudePID->IntegralGain = pitchAttitudePID->IntegralGain;

            DrawPIDGainControl("Pitch D", &pitchAttitudePID->DerivativeGain, 0.0f, 20.0f);
            rollAttitudePID->DerivativeGain = pitchAttitudePID->DerivativeGain;
        }
        else {
            // Individual Pitch control
            DrawPIDGainControl("Pitch P", &pitchAttitudePID->ProportionalGain, 0.0f, 20.0f);
            DrawPIDGainControl("Pitch I", &pitchAttitudePID->IntegralGain, 0.0f, 20.0f);
            DrawPIDGainControl("Pitch D", &pitchAttitudePID->DerivativeGain, 0.0f, 20.0f);
        }
        ImGui::Unindent();

        // Yaw
        ImGui::Indent();
        ImGui::Text("Yaw");
        DrawPIDGainControl("Yaw P", &yawAttitudePID->ProportionalGain, 0.0f, 20.0f);
        DrawPIDGainControl("Yaw I", &yawAttitudePID->IntegralGain, 0.0f, 20.0f);
        DrawPIDGainControl("Yaw D", &yawAttitudePID->DerivativeGain, 0.0f, 20.0f);
        ImGui::Unindent();

        ImGui::Separator();

        if (tunerActive) {
            ImGui::EndDisabled(); // Re-enable after PID gain controls if tuner was active
        }
        // **Save PID Gains Button**
        if (ImGui::Button("Save PID Gains", ImVec2(200, 50)))
        {
            // Include necessary Unreal Engine headers for file operations
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/DateTime.h"

// File path
            FString FilePath = FPaths::ProjectDir() + "PIDGains.csv";

            IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

            bool bFileExists = PlatformFile.FileExists(*FilePath);

            // Prepare header
            FString Header = TEXT("Timestamp,xP,xI,xD,yP,yI,yD,zP,zI,zD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD\n");

            // If file doesn't exist, write the header
            if (!bFileExists)
            {
                FFileHelper::SaveStringToFile(Header, *FilePath);
            }

            // Collect gains
            FString GainData;
            GainData = FDateTime::Now().ToString() + TEXT(","); // Add timestamp
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), xPID->ProportionalGain, xPID->IntegralGain, xPID->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), yPID->ProportionalGain, yPID->IntegralGain, yPID->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), zPID->ProportionalGain, zPID->IntegralGain, zPID->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), rollAttitudePID->ProportionalGain, rollAttitudePID->IntegralGain, rollAttitudePID->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), pitchAttitudePID->ProportionalGain, pitchAttitudePID->IntegralGain, pitchAttitudePID->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f"), yawAttitudePID->ProportionalGain, yawAttitudePID->IntegralGain, yawAttitudePID->DerivativeGain);

            // Append data to file
            FFileHelper::SaveStringToFile(GainData + TEXT("\n"), *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
        }
    }

    // Display Position PID Outputs
    ImGui::Separator();
    ImGui::Text("Position PID Outputs");
    ImGui::Text("X Output: %.2f", xOutput);
    ImGui::Text("Y Output: %.2f", yOutput);
    ImGui::Text("Z Output: %.2f", zOutput);
    ImGui::Separator();

    ImGui::Separator();
    ImGui::Spacing();

    // Display current camera mode
    ImGui::Text("Camera Mode: %s", dronePawn->CameraFPV->IsActive() ? "First Person" : "Third Person");

    // Add the button to switch camera mode
    if (ImGui::Button("Switch Camera Mode", ImVec2(200, 50)))
    {
        if (dronePawn)
        {
            dronePawn->SwitchCamera();
        }
    }

    // Release Input Button
    if (ImGui::Button("Release Input", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            dronePawn->ToggleImguiInput();
        }
    }

    // Reset Drone Button
    if (ImGui::Button("Reset Drone up high", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            // Reset position to (0, 0, 10000)
            dronePawn->SetActorLocation(
                FVector(0.0f, 0.0f, 10000.0f),
                false,
                nullptr,
                ETeleportType::TeleportPhysics // Use TeleportPhysics
            );
            // Reset rotation to zero (optional)
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            // Reset velocities to zero
            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
                // Stop all movement
                droneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                droneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                droneBody->WakeAllRigidBodies();

            }

            altitudeReached = false;
            initialTakeoff = true;

        }
    }
    if (ImGui::Button("Reset Drone 0 point", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            dronePawn->SetActorLocation(
                FVector(0.0f, 0.0f, 10.f),
                false,
                nullptr,
                ETeleportType::TeleportPhysics // Use TeleportPhysics
            );
            // Reset rotation to zero (optional)
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            // Reset velocities to zero
            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
                // Stop all movement
                droneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                droneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                droneBody->WakeAllRigidBodies();

            }

            altitudeReached = false;
            initialTakeoff = true;

        }
    }
    ImGui::End();
}

void ImGuiUtil::VelocityHud(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error, const FVector& desiredVelocity,
    const FVector& currentVelocity,
    float xOutput, float yOutput, float zOutput, float deltaTime)
{
    ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver); // Reduced window size

    ImGui::Begin("Drone Controller", nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar); // Enable vertical scroll

    // Basic Model Feedback
    ImGui::Text("Drone Model Feedback");
    float droneMass = dronePawn ? dronePawn->DroneBody->GetMass() : 0.0f;
    ImGui::Text("Drone Mass: %.2f kg", droneMass);
    // Add more model feedback as needed
    ImGui::Separator();

    ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, 600.0f);
    ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);

    ImGui::Text("Desired Velocities");

    // Initialize temporary variables with current desired velocities
    static float tempVx = desiredNewVelocity.X;
    static float tempVy = desiredNewVelocity.Y;
    static float tempVz = desiredNewVelocity.Z;

    // Track if any slider has changed
    bool velocityChanged = false;

    // Slider for Velocity X
    velocityChanged |= ImGui::SliderFloat("Desired Velocity X", &tempVx, -maxVelocity, maxVelocity);

    // Slider for Velocity Y
    velocityChanged |= ImGui::SliderFloat("Desired Velocity Y", &tempVy, -maxVelocity, maxVelocity);

    // Slider for Velocity Z
    velocityChanged |= ImGui::SliderFloat("Desired Velocity Z", &tempVz, 0, maxVelocity);

    // If any velocity slider was changed, call MoveByVelocity
    if (velocityChanged)
    {
        if (controller)
        {
            controller->SetDesiredVelocity(FVector(tempVx, tempVy, tempVz));
            controller->SetFlightMode(QuadDroneController::FlightMode::VelocityControl);
        }
    }

    ImGui::Separator();
    ImGui::Separator();
    ImGui::Text("Debug Draw");
    ImGui::Checkbox("Drone Collision Sphere", &Debug_DrawDroneCollisionSphere);
    ImGui::Checkbox("Drone Waypoint", &Debug_DrawDroneWaypoint);
    ImGui::Separator();

    ImGui::Separator();
    ImGui::Text("Control All Thrusts");

    static float AllThrustValue = 0.0f; // Initial thrust value

    if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, 0, maxPIDOutput))
    {
        // Set all thrusts to the value from the slider
        for (int i = 0; i < dronePawn->Thrusters.Num(); ++i)
        {
            dronePawn->Thrusters[i]->ThrustStrength = AllThrustValue;
        }
    }

    // Thruster Power
    ImGui::Separator();
    ImGui::Text("Thruster Power");

    // Add synchronization checkboxes for diagonals
    static bool synchronizeDiagonal1 = false;
    static bool synchronizeDiagonal2 = false;

    ImGui::Checkbox("Synchronize Diagonal Motors FL & BR", &synchronizeDiagonal1);
    ImGui::Checkbox("Synchronize Diagonal Motors FR & BL", &synchronizeDiagonal2);

    ImGui::Separator();

    if (ThrustsVal.Num() >= 4)
    {
        // Diagonal 1: Front Left (FL - index 0) and Back Right (BR - index 3)
        ImGui::Text("Diagonal 1 Motors");
        ImGui::Indent();
        if (synchronizeDiagonal1)
        {
            // Use a single slider to control both motors
            if (ImGui::SliderFloat("FL & BR Thrust", &ThrustsVal[0], 0, maxPIDOutput))
            {
                // Set both FL and BR to the same value
                ThrustsVal[3] = ThrustsVal[0];
            }
            // Display the synchronized value for BR
            ImGui::Text("Back Right (Synchronized): %.2f", ThrustsVal[3]);
        }
        else
        {
            // Individual control for FL and BR
            ImGui::SliderFloat("Front Left", &ThrustsVal[0], 0, maxPIDOutput);
            ImGui::SliderFloat("Back Right", &ThrustsVal[3], 0, maxPIDOutput);
        }
        ImGui::Unindent();

        // Diagonal 2: Front Right (FR - index 1) and Back Left (BL - index 2)
        ImGui::Text("Diagonal 2 Motors");
        ImGui::Indent();
        if (synchronizeDiagonal2)
        {
            // Use a single slider to control both motors
            if (ImGui::SliderFloat("FR & BL Thrust", &ThrustsVal[1], 0, maxPIDOutput))
            {
                // Set both FR and BL to the same value
                ThrustsVal[2] = ThrustsVal[1];
            }
            // Display the synchronized value for BL
            ImGui::Text("Back Left (Synchronized): %.2f", ThrustsVal[2]);
        }
        else
        {
            // Individual control for FR and BL
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
    ImGui::Text("Desired Position X, Y, Z: %.2f, %.2f, %.2f", waypoint.X, waypoint.Y, waypoint.Z);
    ImGui::Text("Current Position X, Y, Z: %.2f, %.2f, %.2f", currLoc.X, currLoc.Y, currLoc.Z);
    ImGui::Text("Position Error X, Y, Z: %.2f, %.2f, %.2f", error.X, error.Y, error.Z);
    ImGui::Spacing();
    ImGui::Text("Desired Velocity X, Y, Z: %.2f, %.2f, %.2f", desiredVelocity.X, desiredVelocity.Y, desiredVelocity.Z);
    ImGui::Text("Current Velocity X, Y, Z: %.2f, %.2f, %.2f", currentVelocity.X, currentVelocity.Y, currentVelocity.Z);
    ImGui::Spacing();

    // PID Settings Section
    if (ImGui::CollapsingHeader("PID Settings", ImGuiTreeNodeFlags_DefaultOpen))
    {
        bool tunerActive = controller->bZNTuningInProgress;
        if (tunerActive) {
            ImGui::BeginDisabled(); // Make PID sliders read-only
        }

        auto DrawPIDGainControl = [](const char* label, float* value, float minValue, float maxValue)
            {
                float totalWidth = ImGui::GetContentRegionAvail().x;
                float inputWidth = 80.0f;          // Width of the InputFloat
                float sliderWidth = totalWidth - inputWidth - 20.0f; // Adjust 20.0f for padding

                // Set width for SliderFloat
                ImGui::PushItemWidth(sliderWidth);
                ImGui::SliderFloat(label, value, minValue, maxValue);
                ImGui::PopItemWidth();

                // Place InputFloat next to SliderFloat
                ImGui::SameLine();

                // Set width for InputFloat
                ImGui::PushItemWidth(inputWidth);

                // Set background color to black and text color to white
                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1)); // Black background
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));    // White text

                // Use a hidden label for InputFloat to avoid label collision
                std::string inputLabel = std::string("##") + label;
                ImGui::InputFloat(inputLabel.c_str(), value, 0.0f, 0.0f, "%.3f");

                // Restore styles and widths
                ImGui::PopStyleColor(2);
                ImGui::PopItemWidth();
            };

        // Position PID Gains
        ImGui::Text("Position PID Gains");

        // Add a checkbox for synchronized control for X and Y axes
        static bool synchronizeXYGains = false;
        ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);

        // X Axis
        ImGui::Indent();
        ImGui::Text("X Axis");

        if (synchronizeXYGains) {
            // Synchronized control: adjusting X also adjusts Y
            DrawPIDGainControl("X P", &xPIDVelocity->ProportionalGain, 0.0f, 10.0f);
            yPIDVelocity->ProportionalGain = xPIDVelocity->ProportionalGain;

            DrawPIDGainControl("X I", &xPIDVelocity->IntegralGain, 0.0f, 10.0f);
            yPIDVelocity->IntegralGain = xPIDVelocity->IntegralGain;

            DrawPIDGainControl("X D", &xPIDVelocity->DerivativeGain, 0.0f, 10.0f);
            yPIDVelocity->DerivativeGain = xPIDVelocity->DerivativeGain;
        }
        else {
            // Individual X control
            DrawPIDGainControl("X P", &xPIDVelocity->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("X I", &xPIDVelocity->IntegralGain, 0.0f, 10.0f);
            DrawPIDGainControl("X D", &xPIDVelocity->DerivativeGain, 0.0f, 10.0f);
        }
        ImGui::Unindent();

        // Y Axis
        ImGui::Indent();
        ImGui::Text("Y Axis");

        if (synchronizeXYGains) {
            // Display Y sliders, synchronized with X values
            DrawPIDGainControl("Y P", &yPIDVelocity->ProportionalGain, 0.0f, 10.0f);
            xPIDVelocity->ProportionalGain = yPIDVelocity->ProportionalGain;

            DrawPIDGainControl("Y I", &yPIDVelocity->IntegralGain, 0.0f, 10.0f);
            xPIDVelocity->IntegralGain = yPIDVelocity->IntegralGain;

            DrawPIDGainControl("Y D", &yPIDVelocity->DerivativeGain, 0.0f, 10.0f);
            xPIDVelocity->DerivativeGain = yPIDVelocity->DerivativeGain;
        }
        else {
            // Individual Y control
            DrawPIDGainControl("Y P", &yPIDVelocity->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("Y I", &yPIDVelocity->IntegralGain, 0.0f, 10.0f);
            DrawPIDGainControl("Y D", &yPIDVelocity->DerivativeGain, 0.0f, 10.0f);
        }
        ImGui::Unindent();

        // Z Axis
        ImGui::Indent();
        ImGui::Text("Z Axis");
        DrawPIDGainControl("Z P", &zPIDVelocity->ProportionalGain, 0.0f, 10.0f);
        DrawPIDGainControl("Z I", &zPIDVelocity->IntegralGain, 0.0f, 10.0f);
        DrawPIDGainControl("Z D", &zPIDVelocity->DerivativeGain, 0.0f, 10.0f);
        ImGui::Unindent();

        ImGui::Separator();

        // Attitude PID Gains
        ImGui::Text("Attitude PID Gains");

        static bool synchronizeGains = false;
        ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeGains);

        // Roll
        ImGui::Indent();
        ImGui::Text("Roll");

        if (synchronizeGains) {
            // Synchronized control: adjusting Roll also adjusts Pitch
            DrawPIDGainControl("Roll P", &rollAttitudePIDVelocity->ProportionalGain, 0.0f, 20.0f);
            pitchAttitudePIDVelocity->ProportionalGain = rollAttitudePIDVelocity->ProportionalGain;

            DrawPIDGainControl("Roll I", &rollAttitudePIDVelocity->IntegralGain, 0.0f, 20.0f);
            pitchAttitudePIDVelocity->IntegralGain = rollAttitudePIDVelocity->IntegralGain;

            DrawPIDGainControl("Roll D", &rollAttitudePIDVelocity->DerivativeGain, 0.0f, 20.0f);
            pitchAttitudePIDVelocity->DerivativeGain = rollAttitudePIDVelocity->DerivativeGain;
        }
        else {
            // Individual Roll control
            DrawPIDGainControl("Roll P", &rollAttitudePIDVelocity->ProportionalGain, 0.0f, 20.0f);
            DrawPIDGainControl("Roll I", &rollAttitudePIDVelocity->IntegralGain, 0.0f, 20.0f);
            DrawPIDGainControl("Roll D", &rollAttitudePIDVelocity->DerivativeGain, 0.0f, 20.0f);
        }
        ImGui::Unindent();

        // Pitch
        ImGui::Indent();
        ImGui::Text("Pitch");

        if (synchronizeGains) {
            // Display Pitch sliders, synchronized with Roll values
            DrawPIDGainControl("Pitch P", &pitchAttitudePIDVelocity->ProportionalGain, 0.0f, 20.0f);
            rollAttitudePIDVelocity->ProportionalGain = pitchAttitudePIDVelocity->ProportionalGain;

            DrawPIDGainControl("Pitch I", &pitchAttitudePIDVelocity->IntegralGain, 0.0f, 20.0f);
            rollAttitudePIDVelocity->IntegralGain = pitchAttitudePIDVelocity->IntegralGain;

            DrawPIDGainControl("Pitch D", &pitchAttitudePIDVelocity->DerivativeGain, 0.0f, 20.0f);
            rollAttitudePIDVelocity->DerivativeGain = pitchAttitudePIDVelocity->DerivativeGain;
        }
        else {
            // Individual Pitch control
            DrawPIDGainControl("Pitch P", &pitchAttitudePIDVelocity->ProportionalGain, 0.0f, 20.0f);
            DrawPIDGainControl("Pitch I", &pitchAttitudePIDVelocity->IntegralGain, 0.0f, 20.0f);
            DrawPIDGainControl("Pitch D", &pitchAttitudePIDVelocity->DerivativeGain, 0.0f, 20.0f);
        }
        ImGui::Unindent();

        // Yaw
        ImGui::Indent();
        ImGui::Text("Yaw");
        DrawPIDGainControl("Yaw P", &yawAttitudePIDVelocity->ProportionalGain, 0.0f, 20.0f);
        DrawPIDGainControl("Yaw I", &yawAttitudePIDVelocity->IntegralGain, 0.0f, 20.0f);
        DrawPIDGainControl("Yaw D", &yawAttitudePIDVelocity->DerivativeGain, 0.0f, 20.0f);
        ImGui::Unindent();

        ImGui::Separator();
        if (tunerActive) {
            ImGui::EndDisabled(); 
        }
        // **Save PID Gains Button**
        if (ImGui::Button("Save PID Gains", ImVec2(200, 50)))
        {
            // Include necessary Unreal Engine headers for file operations
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/DateTime.h"

// File path
            FString FilePath = FPaths::ProjectDir() + "PIDGains.csv";

            IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

            bool bFileExists = PlatformFile.FileExists(*FilePath);

            // Prepare header
            FString Header = TEXT("Timestamp,xP,xI,xD,yP,yI,yD,zP,zI,zD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD\n");

            // If file doesn't exist, write the header
            if (!bFileExists)
            {
                FFileHelper::SaveStringToFile(Header, *FilePath);
            }

            // Collect gains
            FString GainData;
            GainData = FDateTime::Now().ToString() + TEXT(","); // Add timestamp
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), xPIDVelocity->ProportionalGain, xPIDVelocity->IntegralGain, xPIDVelocity->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), yPIDVelocity->ProportionalGain, yPIDVelocity->IntegralGain, yPIDVelocity->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), zPIDVelocity->ProportionalGain, zPIDVelocity->IntegralGain, zPIDVelocity->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), rollAttitudePIDVelocity->ProportionalGain, rollAttitudePIDVelocity->IntegralGain, rollAttitudePIDVelocity->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), pitchAttitudePIDVelocity->ProportionalGain, pitchAttitudePIDVelocity->IntegralGain, pitchAttitudePIDVelocity->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f"), yawAttitudePIDVelocity->ProportionalGain, yawAttitudePIDVelocity->IntegralGain, yawAttitudePIDVelocity->DerivativeGain);

            // Append data to file
            FFileHelper::SaveStringToFile(GainData + TEXT("\n"), *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
        }
    }

    // Display Position PID Outputs
    ImGui::Separator();
    ImGui::Text("Position PID Outputs");
    ImGui::Text("X Output: %.2f", xOutput);
    ImGui::Text("Y Output: %.2f", yOutput);
    ImGui::Text("Z Output: %.2f", zOutput);
    ImGui::Separator();

    ImGui::Separator();
    ImGui::Spacing();

    // Display current camera mode
    ImGui::Text("Camera Mode: %s", dronePawn->CameraFPV->IsActive() ? "First Person" : "Third Person");

    // Add the button to switch camera mode
    if (ImGui::Button("Switch Camera Mode", ImVec2(200, 50)))
    {
        if (dronePawn)
        {
            dronePawn->SwitchCamera();
        }
    }

    // Release Input Button
    if (ImGui::Button("Release Input", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            dronePawn->ToggleImguiInput();
        }
    }

    // Reset Drone Button
    if (ImGui::Button("Reset Drone up high", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            // Reset position to (0, 0, 10000)
            dronePawn->SetActorLocation(
                FVector(0.0f, 0.0f, 10000.0f),
                false,
                nullptr,
                ETeleportType::TeleportPhysics 
            );
            // Reset rotation to zero (optional)
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            // Reset velocities to zero
            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
                // Stop all movement
                droneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                droneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                droneBody->WakeAllRigidBodies();

            }

            // Optionally, reset any other physical states or flags
            // For example, reset altitudeReached or initialTakeoff flags
            altitudeReached = false;
            initialTakeoff = true;

            // Note: Do NOT reset the PID controllers here to preserve gains and internal states
        }
    }
    if (ImGui::Button("Reset Drone 0 point", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            // Reset position to (-1110, 3040, 0)
            dronePawn->SetActorLocation(
                FVector(0.0f, 0.0f, 10.f),
                false,
                nullptr,
                ETeleportType::TeleportPhysics 
            );
            // Reset rotation to zero (optional)
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            // Reset velocities to zero
            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
                // Stop all movement
                droneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                droneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                droneBody->WakeAllRigidBodies();

            }

            // Optionally, reset any other physical states or flags
            altitudeReached = false;
            initialTakeoff = true;

            // Note: Do NOT reset the PID controllers here to preserve gains and internal states
        }
    }
    ImGui::End();
}

void ImGuiUtil::JoyStickHud(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error, const FVector& desiredVelocity,
    const FVector& currentVelocity,
    float xOutput, float yOutput, float zOutput, float deltaTime)
{
    ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver); // Reduced window size

    ImGui::Begin("Drone Controller", nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar); // Enable vertical scroll

    // Basic Model Feedback
    ImGui::Text("Drone Model Feedback");
    float droneMass = dronePawn ? dronePawn->DroneBody->GetMass() : 0.0f;
    ImGui::Text("Drone Mass: %.2f kg", droneMass);
    // Add more model feedback as needed
    ImGui::Separator();

    ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, 600.0f);
    ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);

    ImGui::Separator();
    ImGui::Text("Debug Draw");
    ImGui::Checkbox("Drone Collision Sphere", &Debug_DrawDroneCollisionSphere);
    ImGui::Checkbox("Drone Waypoint", &Debug_DrawDroneWaypoint);
    ImGui::Separator();

    ImGui::Separator();
    ImGui::Text("Control All Thrusts");

    static float AllThrustValue = 0.0f; // Initial thrust value

    if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, 0, maxPIDOutput))
    {
        // Set all thrusts to the value from the slider
        for (int i = 0; i < dronePawn->Thrusters.Num(); ++i)
        {
            dronePawn->Thrusters[i]->ThrustStrength = AllThrustValue;
        }
    }

    // Thruster Power
    ImGui::Separator();
    ImGui::Text("Thruster Power");

    // Add synchronization checkboxes for diagonals
    static bool synchronizeDiagonal1 = false;
    static bool synchronizeDiagonal2 = false;

    ImGui::Checkbox("Synchronize Diagonal Motors FL & BR", &synchronizeDiagonal1);
    ImGui::Checkbox("Synchronize Diagonal Motors FR & BL", &synchronizeDiagonal2);

    ImGui::Separator();

    if (ThrustsVal.Num() >= 4)
    {
        // Diagonal 1: Front Left (FL - index 0) and Back Right (BR - index 3)
        ImGui::Text("Diagonal 1 Motors");
        ImGui::Indent();
        if (synchronizeDiagonal1)
        {
            // Use a single slider to control both motors
            if (ImGui::SliderFloat("FL & BR Thrust", &ThrustsVal[0], 0, maxPIDOutput))
            {
                // Set both FL and BR to the same value
                ThrustsVal[3] = ThrustsVal[0];
            }
            // Display the synchronized value for BR
            ImGui::Text("Back Right (Synchronized): %.2f", ThrustsVal[3]);
        }
        else
        {
            // Individual control for FL and BR
            ImGui::SliderFloat("Front Left", &ThrustsVal[0], 0, maxPIDOutput);
            ImGui::SliderFloat("Back Right", &ThrustsVal[3], 0, maxPIDOutput);
        }
        ImGui::Unindent();

        // Diagonal 2: Front Right (FR - index 1) and Back Left (BL - index 2)
        ImGui::Text("Diagonal 2 Motors");
        ImGui::Indent();
        if (synchronizeDiagonal2)
        {
            // Use a single slider to control both motors
            if (ImGui::SliderFloat("FR & BL Thrust", &ThrustsVal[1], 0, maxPIDOutput))
            {
                // Set both FR and BL to the same value
                ThrustsVal[2] = ThrustsVal[1];
            }
            // Display the synchronized value for BL
            ImGui::Text("Back Left (Synchronized): %.2f", ThrustsVal[2]);
        }
        else
        {
            // Individual control for FR and BL
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
    ImGui::Text("Desired Position X, Y, Z: %.2f, %.2f, %.2f", waypoint.X, waypoint.Y, waypoint.Z);
    ImGui::Text("Current Position X, Y, Z: %.2f, %.2f, %.2f", currLoc.X, currLoc.Y, currLoc.Z);
    ImGui::Text("Position Error X, Y, Z: %.2f, %.2f, %.2f", error.X, error.Y, error.Z);
    ImGui::Spacing();
    ImGui::Text("Desired Velocity X, Y, Z: %.2f, %.2f, %.2f", desiredVelocity.X, desiredVelocity.Y, desiredVelocity.Z);
    ImGui::Text("Current Velocity X, Y, Z: %.2f, %.2f, %.2f", currentVelocity.X, currentVelocity.Y, currentVelocity.Z);
    ImGui::Spacing();

    // PID Settings Section
    if (ImGui::CollapsingHeader("PID Settings", ImGuiTreeNodeFlags_DefaultOpen))
    {
        bool tunerActive = controller->bZNTuningInProgress;
        if (tunerActive) {
            ImGui::BeginDisabled(); // Make PID sliders read-only
        }
        auto DrawPIDGainControl = [](const char* label, float* value, float minValue, float maxValue)
            {
                float totalWidth = ImGui::GetContentRegionAvail().x;
                float inputWidth = 80.0f;          // Width of the InputFloat
                float sliderWidth = totalWidth - inputWidth - 20.0f; // Adjust 20.0f for padding

                // Set width for SliderFloat
                ImGui::PushItemWidth(sliderWidth);
                ImGui::SliderFloat(label, value, minValue, maxValue);
                ImGui::PopItemWidth();

                // Place InputFloat next to SliderFloat
                ImGui::SameLine();

                // Set width for InputFloat
                ImGui::PushItemWidth(inputWidth);

                // Set background color to black and text color to white
                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1)); // Black background
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));    // White text

                // Use a hidden label for InputFloat to avoid label collision
                std::string inputLabel = std::string("##") + label;
                ImGui::InputFloat(inputLabel.c_str(), value, 0.0f, 0.0f, "%.3f");

                // Restore styles and widths
                ImGui::PopStyleColor(2);
                ImGui::PopItemWidth();
            };

        // Position PID Gains
        ImGui::Text("Position PID Gains");

        // Add a checkbox for synchronized control for X and Y axes
        static bool synchronizeXYGains = false;
        ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);

        // X Axis
        ImGui::Indent();
        ImGui::Text("X Axis");

        if (synchronizeXYGains) {
            // Synchronized control: adjusting X also adjusts Y
            DrawPIDGainControl("X P", &xPIDJoyStick->ProportionalGain, 0.0f, 10.0f);
            yPIDJoyStick->ProportionalGain = xPIDJoyStick->ProportionalGain;

            DrawPIDGainControl("X I", &xPIDJoyStick->IntegralGain, 0.0f, 10.0f);
            yPIDJoyStick->IntegralGain = xPIDJoyStick->IntegralGain;

            DrawPIDGainControl("X D", &xPIDJoyStick->DerivativeGain, 0.0f, 10.0f);
            yPIDJoyStick->DerivativeGain = xPIDJoyStick->DerivativeGain;
        }
        else {
            // Individual X control
            DrawPIDGainControl("X P", &xPIDJoyStick->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("X I", &xPIDJoyStick->IntegralGain, 0.0f, 10.0f);
            DrawPIDGainControl("X D", &xPIDJoyStick->DerivativeGain, 0.0f, 10.0f);
        }
        ImGui::Unindent();

        // Y Axis
        ImGui::Indent();
        ImGui::Text("Y Axis");

        if (synchronizeXYGains) {
            // Display Y sliders, synchronized with X values
            DrawPIDGainControl("Y P", &yPIDJoyStick->ProportionalGain, 0.0f, 10.0f);
            xPIDJoyStick->ProportionalGain = yPIDJoyStick->ProportionalGain;

            DrawPIDGainControl("Y I", &yPIDJoyStick->IntegralGain, 0.0f, 10.0f);
            xPIDJoyStick->IntegralGain = yPIDJoyStick->IntegralGain;

            DrawPIDGainControl("Y D", &yPIDJoyStick->DerivativeGain, 0.0f, 10.0f);
            xPIDJoyStick->DerivativeGain = yPIDJoyStick->DerivativeGain;
        }
        else {
            // Individual Y control
            DrawPIDGainControl("Y P", &yPIDJoyStick->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("Y I", &yPIDJoyStick->IntegralGain, 0.0f, 10.0f);
            DrawPIDGainControl("Y D", &yPIDJoyStick->DerivativeGain, 0.0f, 10.0f);
        }
        ImGui::Unindent();

        // Z Axis
        ImGui::Indent();
        ImGui::Text("Z Axis");
        DrawPIDGainControl("Z P", &zPIDJoyStick->ProportionalGain, 0.0f, 10.0f);
        DrawPIDGainControl("Z I", &zPIDJoyStick->IntegralGain, 0.0f, 10.0f);
        DrawPIDGainControl("Z D", &zPIDJoyStick->DerivativeGain, 0.0f, 10.0f);
        ImGui::Unindent();

        ImGui::Separator();

        // Attitude PID Gains
        ImGui::Text("Attitude PID Gains");

        static bool synchronizeGains = false;
        ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeGains);

        // Roll
        ImGui::Indent();
        ImGui::Text("Roll");

        if (synchronizeGains) {
            // Synchronized control: adjusting Roll also adjusts Pitch
            DrawPIDGainControl("Roll P", &rollAttitudePIDJoyStick->ProportionalGain, 0.0f, 20.0f);
            pitchAttitudePIDJoyStick->ProportionalGain = rollAttitudePIDJoyStick->ProportionalGain;

            DrawPIDGainControl("Roll I", &rollAttitudePIDJoyStick->IntegralGain, 0.0f, 20.0f);
            pitchAttitudePIDJoyStick->IntegralGain = rollAttitudePIDJoyStick->IntegralGain;

            DrawPIDGainControl("Roll D", &rollAttitudePIDJoyStick->DerivativeGain, 0.0f, 20.0f);
            pitchAttitudePIDJoyStick->DerivativeGain = rollAttitudePIDJoyStick->DerivativeGain;
        }
        else {
            // Individual Roll control
            DrawPIDGainControl("Roll P", &rollAttitudePIDJoyStick->ProportionalGain, 0.0f, 20.0f);
            DrawPIDGainControl("Roll I", &rollAttitudePIDJoyStick->IntegralGain, 0.0f, 20.0f);
            DrawPIDGainControl("Roll D", &rollAttitudePIDJoyStick->DerivativeGain, 0.0f, 20.0f);
        }
        ImGui::Unindent();

        // Pitch
        ImGui::Indent();
        ImGui::Text("Pitch");

        if (synchronizeGains) {
            // Display Pitch sliders, synchronized with Roll values
            DrawPIDGainControl("Pitch P", &pitchAttitudePIDJoyStick->ProportionalGain, 0.0f, 20.0f);
            rollAttitudePIDJoyStick->ProportionalGain = pitchAttitudePIDJoyStick->ProportionalGain;

            DrawPIDGainControl("Pitch I", &pitchAttitudePIDJoyStick->IntegralGain, 0.0f, 20.0f);
            rollAttitudePIDJoyStick->IntegralGain = pitchAttitudePIDJoyStick->IntegralGain;

            DrawPIDGainControl("Pitch D", &pitchAttitudePIDJoyStick->DerivativeGain, 0.0f, 20.0f);
            rollAttitudePIDJoyStick->DerivativeGain = pitchAttitudePIDJoyStick->DerivativeGain;
        }
        else {
            // Individual Pitch control
            DrawPIDGainControl("Pitch P", &pitchAttitudePIDJoyStick->ProportionalGain, 0.0f, 20.0f);
            DrawPIDGainControl("Pitch I", &pitchAttitudePIDJoyStick->IntegralGain, 0.0f, 20.0f);
            DrawPIDGainControl("Pitch D", &pitchAttitudePIDJoyStick->DerivativeGain, 0.0f, 20.0f);
        }
        ImGui::Unindent();

        // Yaw
        ImGui::Indent();
        ImGui::Text("Yaw");
        DrawPIDGainControl("Yaw P", &yawAttitudePIDJoyStick->ProportionalGain, 0.0f, 20.0f);
        DrawPIDGainControl("Yaw I", &yawAttitudePIDJoyStick->IntegralGain, 0.0f, 20.0f);
        DrawPIDGainControl("Yaw D", &yawAttitudePIDJoyStick->DerivativeGain, 0.0f, 20.0f);
        ImGui::Unindent();

        ImGui::Separator();
        if (tunerActive) {
            ImGui::EndDisabled();
        }
        // **Save PID Gains Button**
        if (ImGui::Button("Save PID Gains", ImVec2(200, 50)))
        {
            // Include necessary Unreal Engine headers for file operations
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/DateTime.h"

// File path
            FString FilePath = FPaths::ProjectDir() + "PIDGains.csv";

            IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

            bool bFileExists = PlatformFile.FileExists(*FilePath);

            // Prepare header
            FString Header = TEXT("Timestamp,xP,xI,xD,yP,yI,yD,zP,zI,zD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD\n");

            // If file doesn't exist, write the header
            if (!bFileExists)
            {
                FFileHelper::SaveStringToFile(Header, *FilePath);
            }

            // Collect gains
            FString GainData;
            GainData = FDateTime::Now().ToString() + TEXT(","); // Add timestamp
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), xPIDJoyStick->ProportionalGain, xPIDJoyStick->IntegralGain, xPIDJoyStick->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), yPIDJoyStick->ProportionalGain, yPIDJoyStick->IntegralGain, yPIDJoyStick->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), zPIDJoyStick->ProportionalGain, zPIDJoyStick->IntegralGain, zPIDJoyStick->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), rollAttitudePIDJoyStick->ProportionalGain, rollAttitudePIDJoyStick->IntegralGain, rollAttitudePIDJoyStick->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), pitchAttitudePIDJoyStick->ProportionalGain, pitchAttitudePIDJoyStick->IntegralGain, pitchAttitudePIDJoyStick->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f"), yawAttitudePIDJoyStick->ProportionalGain, yawAttitudePIDJoyStick->IntegralGain, yawAttitudePIDJoyStick->DerivativeGain);

            // Append data to file
            FFileHelper::SaveStringToFile(GainData + TEXT("\n"), *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
        }
    }

    // Display Position PID Outputs
    ImGui::Separator();
    ImGui::Text("Position PID Outputs");
    ImGui::Text("X Output: %.2f", xOutput);
    ImGui::Text("Y Output: %.2f", yOutput);
    ImGui::Text("Z Output: %.2f", zOutput);
    ImGui::Separator();

    ImGui::Separator();
    ImGui::Spacing();

    // Display current camera mode
    ImGui::Text("Camera Mode: %s", dronePawn->CameraFPV->IsActive() ? "First Person" : "Third Person");

    // Add the button to switch camera mode
    if (ImGui::Button("Switch Camera Mode", ImVec2(200, 50)))
    {
        if (dronePawn)
        {
            dronePawn->SwitchCamera();
        }
    }

    // Release Input Button
    if (ImGui::Button("Release Input", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            dronePawn->ToggleImguiInput();
        }
    }

    // Reset Drone Button
    if (ImGui::Button("Reset Drone up high", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            // Reset position to (0, 0, 10000)
            dronePawn->SetActorLocation(
                FVector(0.0f, 0.0f, 10000.0f),
                false,
                nullptr,
                ETeleportType::TeleportPhysics 
            );
            // Reset rotation to zero (optional)
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            // Reset velocities to zero
            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
                // Stop all movement
                droneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                droneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                droneBody->WakeAllRigidBodies();


            }

            altitudeReached = false;
            initialTakeoff = true;

        }
    }
    if (ImGui::Button("Reset Drone 0 point", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            dronePawn->SetActorLocation(
                FVector(0.0f, 0.0f, 10.f),
                false,
                nullptr,
                ETeleportType::TeleportPhysics 
            );
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
                // Stop all movement
                droneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                droneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                droneBody->WakeAllRigidBodies();

            }

            altitudeReached = false;
            initialTakeoff = true;

        }
    }
    ImGui::End();
}