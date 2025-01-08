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

ImGuiUtil::ImGuiUtil(
    AQuadPawn* InPawn, UQuadDroneController* InController, FVector& IndesiredNewVelocity, bool& InDebug_DrawDroneCollisionSphere, bool& InDebug_DrawDroneWaypoint, float InMaxPIDOutput,float& InMaxVelocity,float& InMaxAngle)
    : dronePawn(InPawn),Debug_DrawDroneCollisionSphere(InDebug_DrawDroneCollisionSphere) , Debug_DrawDroneWaypoint(InDebug_DrawDroneWaypoint),maxPIDOutput(InMaxPIDOutput) ,maxVelocity(InMaxVelocity), maxAngle(InMaxAngle), controller(InController), desiredNewVelocity(IndesiredNewVelocity)

{
}
ImGuiUtil::~ImGuiUtil()
{
}

void ImGuiUtil::AutoWaypointHud(
    TArray<float> &ThrustsVal,
    float rollError, float pitchError,
    const FRotator &currentRotation,
    const FVector &waypoint, const FVector &currLoc,
    const FVector &error,
    const FVector &currentVelocity,
    float xOutput, float yOutput, float zOutput, float deltaTime)
{
    ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);

    ImGui::Begin("Drone Controller", nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar);

    // Display Drone Model Feedback
    DisplayDroneInfo();

    // Display Max Velocity and Max Tilt Angle sliders
    ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, 600.0f);
    ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);

    // Display Debug Draw Options
    DisplayDebugOptions();

    // Display Thruster Controls
    DisplayThrusterControls(ThrustsVal);

    // Display Desired and Current Roll/Pitch
    ImGui::Separator();
    ImGui::Text("Desired Roll: %.2f", rollError);
    ImGui::SameLine();
    ImGui::Text("Current Roll: %.2f", currentRotation.Roll);

    ImGui::Text("Desired Pitch: %.2f", pitchError);
    ImGui::SameLine();
    ImGui::Text("Current Pitch: %.2f", currentRotation.Pitch);

    // Display Desired and Current Positions and Errors
    ImGui::Separator();
    ImGui::Text("Desired Position X, Y, Z: %.2f, %.2f, %.2f", waypoint.X, waypoint.Y, waypoint.Z);
    ImGui::Text("Current Position X, Y, Z: %.2f, %.2f, %.2f", currLoc.X, currLoc.Y, currLoc.Z);
    ImGui::Text("Position Error X, Y, Z: %.2f, %.2f, %.2f", error.X, error.Y, error.Z);
    ImGui::Spacing();
    ImGui::Text("Desired Velocity X, Y, Z: %.2f, %.2f, %.2f", desiredNewVelocity.X, desiredNewVelocity.Y, desiredNewVelocity.Z);
    ImGui::Text("Current Velocity X, Y, Z: %.2f, %.2f, %.2f", currentVelocity.X, currentVelocity.Y, currentVelocity.Z);
    ImGui::Spacing();

    // PID Settings
    static bool synchronizeXYGains = false;
    static bool synchronizeGains = false;
    DisplayPIDSettings(
        EFlightMode::AutoWaypoint,
        "PID Settings",
        synchronizeXYGains,
        synchronizeGains
    );
    // Display Position PID Outputs
    ImGui::Separator();
    ImGui::Text("Position PID Outputs");
    ImGui::Text("X Output: %.2f", xOutput);
    ImGui::Text("Y Output: %.2f", yOutput);
    ImGui::Text("Z Output: %.2f", zOutput);
    ImGui::Separator();

    // Display Camera Mode and Switch Button
    DisplayCameraControls();

    // Display Buttons for Releasing Input and Resetting Drone
    DisplayResetDroneButtons();

    ImGui::End();
}

void ImGuiUtil::VelocityHud(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error,
    const FVector& currentVelocity,
    float xOutput, float yOutput, float zOutput, float deltaTime)
{
    // Set up window
    ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);

    ImGui::Begin("Drone Controller", nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar);

    // Display Drone Model Feedback
    DisplayDroneInfo();

    // Display Max Velocity and Max Tilt Angle sliders
    ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, 600.0f);
    ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);

    // Display Desired Velocities
    DisplayDesiredVelocities();

    // Display Debug Draw Options
    DisplayDebugOptions();

    // Display Thruster Controls
    DisplayThrusterControls(ThrustsVal);

    // Display Desired and Current Roll/Pitch
    ImGui::Separator();
    ImGui::Text("Desired Roll: %.2f", rollError);
    ImGui::SameLine();
    ImGui::Text("Current Roll: %.2f", currentRotation.Roll);

    ImGui::Text("Desired Pitch: %.2f", pitchError);
    ImGui::SameLine();
    ImGui::Text("Current Pitch: %.2f", currentRotation.Pitch);

    // Display Desired and Current Positions and Errors
    ImGui::Separator();
    ImGui::Text("Desired Position X, Y, Z: %.2f, %.2f, %.2f", waypoint.X, waypoint.Y, waypoint.Z);
    ImGui::Text("Current Position X, Y, Z: %.2f, %.2f, %.2f", currLoc.X, currLoc.Y, currLoc.Z);
    ImGui::Text("Position Error X, Y, Z: %.2f, %.2f, %.2f", error.X, error.Y, error.Z);
    ImGui::Spacing();
    ImGui::Text("Desired Velocity X, Y, Z: %.2f, %.2f, %.2f", desiredNewVelocity.X, desiredNewVelocity.Y, desiredNewVelocity.Z);
    ImGui::Text("Current Velocity X, Y, Z: %.2f, %.2f, %.2f", currentVelocity.X, currentVelocity.Y, currentVelocity.Z);
    ImGui::Spacing();

    // PID Settings
    static bool syncXY = false;
    static bool syncRP = false;
    DisplayPIDSettings(
        EFlightMode::VelocityControl,
        "PID Settings",
        syncXY,
        syncRP
    );
    
    // Display Position PID Outputs
    ImGui::Separator();
    ImGui::Text("Position PID Outputs");
    ImGui::Text("X Output: %.2f", xOutput);
    ImGui::Text("Y Output: %.2f", yOutput);
    ImGui::Text("Z Output: %.2f", zOutput);
    ImGui::Separator();

    // Display Camera Mode and Switch Button
    DisplayCameraControls();

    // Display Buttons for Releasing Input and Resetting Drone
    DisplayResetDroneButtons();

    ImGui::End();
}

void ImGuiUtil::JoyStickHud(
    TArray<float> &ThrustsVal,
    float rollError, float pitchError,
    const FRotator &currentRotation,
    const FVector &waypoint, const FVector &currLoc,
    const FVector &error,
    const FVector &currentVelocity,
    float xOutput, float yOutput, float zOutput, float deltaTime)
{
    // Set up window
    ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);

    ImGui::Begin("Drone Controller", nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar);

    // Display Drone Model Feedback
    DisplayDroneInfo();

    // Display Max Velocity and Max Tilt Angle sliders
    ImGui::SliderFloat("Max velocity", &maxVelocity, 0.0f, 600.0f);
    ImGui::SliderFloat("Max tilt angle", &maxAngle, 0.0f, 45.0f);

    // Display Debug Draw Options
    DisplayDebugOptions();

    // Display Thruster Controls
    DisplayThrusterControls(ThrustsVal);

    // Display Desired and Current Roll/Pitch
    ImGui::Separator();
    ImGui::Text("Desired Roll: %.2f", rollError);
    ImGui::SameLine();
    ImGui::Text("Current Roll: %.2f", currentRotation.Roll);

    ImGui::Text("Desired Pitch: %.2f", pitchError);
    ImGui::SameLine();
    ImGui::Text("Current Pitch: %.2f", currentRotation.Pitch);

    // Display Desired and Current Positions and Errors
    ImGui::Separator();
    ImGui::Text("Desired Position X, Y, Z: %.2f, %.2f, %.2f", waypoint.X, waypoint.Y, waypoint.Z);
    ImGui::Text("Current Position X, Y, Z: %.2f, %.2f, %.2f", currLoc.X, currLoc.Y, currLoc.Z);
    ImGui::Text("Position Error X, Y, Z: %.2f, %.2f, %.2f", error.X, error.Y, error.Z);
    ImGui::Spacing();
    ImGui::Text("Desired Velocity X, Y, Z: %.2f, %.2f, %.2f", desiredNewVelocity.X, desiredNewVelocity.Y, desiredNewVelocity.Z);
    ImGui::Text("Current Velocity X, Y, Z: %.2f, %.2f, %.2f", currentVelocity.X, currentVelocity.Y, currentVelocity.Z);
    ImGui::Spacing();

    // PID Settings
    static bool synchronizeXYGains = false;
    static bool synchronizeGains = false;
    DisplayPIDSettings(
        EFlightMode::JoyStickControl,
        "PID Settings",
        synchronizeXYGains,
        synchronizeGains
    );
    // Display Position PID Outputs
    ImGui::Separator();
    ImGui::Text("Position PID Outputs");
    ImGui::Text("X Output: %.2f", xOutput);
    ImGui::Text("Y Output: %.2f", yOutput);
    ImGui::Text("Z Output: %.2f", zOutput);
    ImGui::Separator();

    // Display Camera Mode and Switch Button
    DisplayCameraControls();

    // Display Buttons for Releasing Input and Resetting Drone
    DisplayResetDroneButtons();

    ImGui::End();
}

void ImGuiUtil::RenderImPlot(const TArray<float> &ThrustsVal, float deltaTime)
{
    if (ThrustsVal.Num() < 4)
    {
        return;
    }

    // Update the cumulative time
    CumulativeTime += deltaTime;

    // Add current time and thrust values to the data arrays
    TimeData.Add(CumulativeTime);
    Thrust0Data.Add(ThrustsVal[0]); // FL
    Thrust1Data.Add(ThrustsVal[1]); // FR
    Thrust2Data.Add(ThrustsVal[2]); // BL
    Thrust3Data.Add(ThrustsVal[3]); // BR

    // Remove old data points if we're beyond MaxPlotTime
    while (TimeData.Num() > 0 && (CumulativeTime - TimeData[0] > MaxPlotTime))
    {
        TimeData.RemoveAt(0);
        Thrust0Data.RemoveAt(0);
        Thrust1Data.RemoveAt(0);
        Thrust2Data.RemoveAt(0);
        Thrust3Data.RemoveAt(0);
    }

    // Ensure we don't exceed MaxDataPoints
    while (TimeData.Num() > MaxDataPoints)
    {
        TimeData.RemoveAt(0);
        Thrust0Data.RemoveAt(0);
        Thrust1Data.RemoveAt(0);
        Thrust2Data.RemoveAt(0);
        Thrust3Data.RemoveAt(0);
    }

    // Set default window position and size
    ImGui::SetNextWindowPos(ImVec2(850, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);

    // Begin ImGui window with resize enabled
    ImGui::Begin("Drone Thrust Analysis", nullptr, ImGuiWindowFlags_NoCollapse);

    // Get current window size for plot sizing
    ImVec2 windowSize = ImGui::GetContentRegionAvail();

    // Plot configuration flags for mouse interaction
    ImPlotFlags plotFlags = ImPlotFlags_None;
    ImPlotAxisFlags axisFlags = ImPlotAxisFlags_None;

    if (ImPlot::BeginPlot("Thrust Values Over Time", windowSize, plotFlags))
    {
        // Setup axes with default ranges but allow user interaction
        ImPlot::SetupAxes("Time (s)", "Thrust (N)", axisFlags, axisFlags);

        // Define vibrant colors for better visibility
        const ImVec4 FL_COLOR(1.0f, 0.2f, 0.2f, 1.0f); // Bright red
        const ImVec4 FR_COLOR(0.2f, 1.0f, 0.2f, 1.0f); // Bright green
        const ImVec4 BL_COLOR(0.2f, 0.6f, 1.0f, 1.0f); // Bright blue
        const ImVec4 BR_COLOR(1.0f, 0.8f, 0.0f, 1.0f); // Bright yellow

        // Plot each thrust component
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

    ImGui::End();
}

void ImGuiUtil::DisplayDroneInfo()
{
    ImGui::Text("Drone Model Feedback");
    float droneMass = dronePawn ? dronePawn->DroneBody->GetMass() : 0.0f;
    ImGui::Text("Drone Mass: %.2f kg", droneMass);
    ImGui::Separator();
}

void ImGuiUtil::DisplayDebugOptions()
{
    ImGui::Separator();
    ImGui::Text("Debug Draw");
    ImGui::Checkbox("Drone Collision Sphere", &Debug_DrawDroneCollisionSphere);
    ImGui::Checkbox("Drone Waypoint", &Debug_DrawDroneWaypoint);
    ImGui::Separator();
}

// Add this helper method to ImGuiUtil class
void ImGuiUtil::DisplayThrusterControls(TArray<float>& ThrustsVal)
{
    if (!dronePawn) return;

    ImGui::Separator();
    ImGui::Text("Control All Thrusts");

    static float AllThrustValue = 0.0f;

    if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, 0, maxPIDOutput))
    {
        // Apply to all thrusters
        if (dronePawn->ThrusterFL) dronePawn->ThrusterFL->ThrustStrength = AllThrustValue;
        if (dronePawn->ThrusterFR) dronePawn->ThrusterFR->ThrustStrength = AllThrustValue;
        if (dronePawn->ThrusterBL) dronePawn->ThrusterBL->ThrustStrength = AllThrustValue;
        if (dronePawn->ThrusterBR) dronePawn->ThrusterBR->ThrustStrength = AllThrustValue;
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
        // Diagonal 1
        ImGui::Text("Diagonal 1 Motors");
        ImGui::Indent();
        if (synchronizeDiagonal1)
        {
            if (ImGui::SliderFloat("FL & BR Thrust", &ThrustsVal[0], 0, maxPIDOutput))
            {
                ThrustsVal[3] = ThrustsVal[0];
                if (dronePawn->ThrusterFL) dronePawn->ThrusterFL->ThrustStrength = ThrustsVal[0];
                if (dronePawn->ThrusterBR) dronePawn->ThrusterBR->ThrustStrength = ThrustsVal[0];
            }
            ImGui::Text("Back Right (Synchronized): %.2f", ThrustsVal[3]);
        }
        else
        {
            if (ImGui::SliderFloat("Front Left", &ThrustsVal[0], 0, maxPIDOutput))
            {
                if (dronePawn->ThrusterFL) dronePawn->ThrusterFL->ThrustStrength = ThrustsVal[0];
            }
            if (ImGui::SliderFloat("Back Right", &ThrustsVal[3], 0, maxPIDOutput))
            {
                if (dronePawn->ThrusterBR) dronePawn->ThrusterBR->ThrustStrength = ThrustsVal[3];
            }
        }
        ImGui::Unindent();

        // Diagonal 2
        ImGui::Text("Diagonal 2 Motors");
        ImGui::Indent();
        if (synchronizeDiagonal2)
        {
            if (ImGui::SliderFloat("FR & BL Thrust", &ThrustsVal[1], 0, maxPIDOutput))
            {
                ThrustsVal[2] = ThrustsVal[1];
                if (dronePawn->ThrusterFR) dronePawn->ThrusterFR->ThrustStrength = ThrustsVal[1];
                if (dronePawn->ThrusterBL) dronePawn->ThrusterBL->ThrustStrength = ThrustsVal[1];
            }
            ImGui::Text("Back Left (Synchronized): %.2f", ThrustsVal[2]);
        }
        else
        {
            if (ImGui::SliderFloat("Front Right", &ThrustsVal[1], 0, maxPIDOutput))
            {
                if (dronePawn->ThrusterFR) dronePawn->ThrusterFR->ThrustStrength = ThrustsVal[1];
            }
            if (ImGui::SliderFloat("Back Left", &ThrustsVal[2], 0, maxPIDOutput))
            {
                if (dronePawn->ThrusterBL) dronePawn->ThrusterBL->ThrustStrength = ThrustsVal[2];
            }
        }
        ImGui::Unindent();
    }
}


void ImGuiUtil::DisplayPIDSettings(
    EFlightMode Mode,
    const char *headerLabel,
    bool &synchronizeXYGains,
    bool &synchronizeGains)
{
    // Fetch the PID set for the specified flight mode
    FFullPIDSet* PIDSet = controller->GetPIDSet(Mode);
    if (!PIDSet)
    {
        ImGui::Text("No PID Set found for this mode.");
        return;
    }
    if (ImGui::CollapsingHeader(headerLabel, ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Lambda to draw individual PID gain controls
        auto DrawPIDGainControl = [](const char *label, float *value, float minValue, float maxValue)
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

        // Position PID Gains
        ImGui::Text("Position PID Gains");
        ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);

        // X Axis
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

        // Y Axis
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

        // Z Axis
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

        // Attitude PID Gains
        ImGui::Text("Attitude PID Gains");
        ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeGains);

        // Roll
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

        // Pitch
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

        // Yaw
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
            // Save PID gains to file
            FString FilePath = FPaths::ProjectDir() + "PIDGains.csv";

            IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

            bool bFileExists = PlatformFile.FileExists(*FilePath);

            FString Header = TEXT("Timestamp,xP,xI,xD,yP,yI,yD,zP,zI,zD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD\n");

            if (!bFileExists)
            {
                FFileHelper::SaveStringToFile(Header, *FilePath);
            }

            FString GainData;
            GainData = FDateTime::Now().ToString() + TEXT(","); // Add timestamp

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
    
void ImGuiUtil::DisplayCameraControls()
{
    ImGui::Separator();
    ImGui::Spacing();

    // ImGui::Text("Camera Mode: %s", dronePawn->CameraFPV->IsActive() ? "First Person" : "Third Person");

    if (ImGui::Button("Switch Camera Mode", ImVec2(200, 50)))
    {
        if (dronePawn)
        {
            dronePawn->SwitchCamera();
        }
    }
}

void ImGuiUtil::DisplayResetDroneButtons()
{
    if (ImGui::Button("Release Input", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            dronePawn->ToggleImguiInput();
        }
    }

    if (ImGui::Button("Reset Drone up high", ImVec2(200, 100)))
    {
        if (controller)
        {
            controller->ResetDroneHigh();
        }
    }

    if (ImGui::Button("Reset Drone 0 point", ImVec2(200, 100)))
    {
        if (controller)
        {
            controller->ResetDroneOrigin();
        }
    }
}
void ImGuiUtil::DisplayDesiredVelocities()
{
    ImGui::Text("Desired Velocities");

    // Initialize temporary variables with current desired velocities
    float tempVx = desiredNewVelocity.X;
    float tempVy = desiredNewVelocity.Y;
    float tempVz = desiredNewVelocity.Z;

    // Track if any slider has changed
    bool velocityChanged = false;

    // Slider for Velocity X
    velocityChanged |= ImGui::SliderFloat("Desired Velocity X", &tempVx, -maxVelocity, maxVelocity);

    // Slider for Velocity Y
    velocityChanged |= ImGui::SliderFloat("Desired Velocity Y", &tempVy, -maxVelocity, maxVelocity);

    // Slider for Velocity Z
    velocityChanged |= ImGui::SliderFloat("Desired Velocity Z", &tempVz, -maxVelocity, maxVelocity);

    // If any velocity slider was changed, update desired velocity
    if (velocityChanged)
    {
        desiredNewVelocity.X = tempVx;
        desiredNewVelocity.Y = tempVy;
        desiredNewVelocity.Z = tempVz;

        if (controller)
        {
            controller->SetDesiredVelocity(desiredNewVelocity);
        }
    }

    ImGui::Separator();
}
