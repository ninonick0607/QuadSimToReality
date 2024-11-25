#include "ImGuiUtil.h"
#include "imgui.h"
#include "implot.h"
#include "QuadPawn.h"
#include "string"
#include "QuadDroneController.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/DateTime.h"


ImGuiUtil::ImGuiUtil(
    AQuadPawn* InPawn,
    UQuadDroneController* InController,
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
    : dronePawn(InPawn)                                   // 1
    , initialTakeoff(InInitialTakeoff)                    // 2
    , altitudeReached(InAltitudeReached)                  // 3
    , Debug_DrawDroneCollisionSphere(InDebug_DrawDroneCollisionSphere)  // 4
    , Debug_DrawDroneWaypoint(InDebug_DrawDroneWaypoint)  // 5
    , maxPIDOutput(InMaxPIDOutput)                        // 6
    , altitudeThresh(InAltitudeThresh)                    // 7
    , minAltitudeLocal(InMinAltitudeLocal)                // 8
    , maxVelocity(InMaxVelocity)                          // 9
    , maxAngle(InMaxAngle)                                // 10
    , xPID(InXPID)                                        // 11
    , yPID(InYPID)                                        // 12
    , zPID(InZPID)                                        // 13
    , rollAttitudePID(InRollAttitudePID)                  // 14
    , pitchAttitudePID(InPitchAttitudePID)                // 15
    , yawAttitudePID(InYawAttitudePID)                    // 16
    , xPIDVelocity(InxPIDVelocity)                        // 17
    , yPIDVelocity(InyPIDVelocity)                        // 18
    , zPIDVelocity(InzPIDVelocity)                        // 19
    , rollAttitudePIDVelocity(InrollAttitudePIDVelocity)  // 20
    , pitchAttitudePIDVelocity(InpitchAttitudePIDVelocity)// 21
    , yawAttitudePIDVelocity(InyawAttitudePIDVelocity)    // 22
    , xPIDJoyStick(InxPIDJoyStick)                        // 23
    , yPIDJoyStick(InyPIDJoyStick)                        // 24
    , zPIDJoyStick(InzPIDJoyStick)                        // 25
    , rollAttitudePIDJoyStick(InrollAttitudePIDJoyStick)  // 26
    , pitchAttitudePIDJoyStick(InpitchAttitudePIDJoyStick)// 27
    , yawAttitudePIDJoyStick(InyawAttitudePIDJoyStick)    // 28
    , controller(InController)                            // 29
    , desiredNewVelocity(IndesiredNewVelocity)            // 30
    // ... initialize other protected members
    //, TimeData(MaxBufferSize)
    //, xPIDOutputHistory(MaxBufferSize)
    //, yPIDOutputHistory(MaxBufferSize)
    //, zPIDOutputHistory(MaxBufferSize)
    //, rollPIDOutputHistory(MaxBufferSize)
    //, pitchPIDOutputHistory(MaxBufferSize)
    //, yawPIDOutputHistory(MaxBufferSize)
    //, positionErrorHistory(MaxBufferSize)
    //, velocityErrorHistory(MaxBufferSize)
{
}
ImGuiUtil::~ImGuiUtil()
{
}


void ImGuiUtil::AutoWaypointHud(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error,
    const FVector& currentVelocity,
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
        xPID, yPID, zPID,
        rollAttitudePID, pitchAttitudePID, yawAttitudePID,
        "PID Settings", synchronizeXYGains, synchronizeGains);

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
    static bool synchronizeXYGains = false;
    static bool synchronizeGains = false;
    DisplayPIDSettings(
        xPIDVelocity, yPIDVelocity, zPIDVelocity,
        rollAttitudePIDVelocity, pitchAttitudePIDVelocity, yawAttitudePIDVelocity,
        "PID Settings", synchronizeXYGains, synchronizeGains);

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
        xPIDJoyStick, yPIDJoyStick, zPIDJoyStick,
        rollAttitudePIDJoyStick, pitchAttitudePIDJoyStick, yawAttitudePIDJoyStick,
        "PID Settings", synchronizeXYGains, synchronizeGains);

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

void ImGuiUtil::RenderImPlot(const TArray<float>& ThrustsVal, float deltaTime)
{
    if (ThrustsVal.Num() < 4)
    {
        return;
    }

    // Update the cumulative time
    CumulativeTime += deltaTime;

    // Add current time and thrust values to the data arrays
    TimeData.Add(CumulativeTime);
    Thrust0Data.Add(ThrustsVal[0]);  // FL
    Thrust1Data.Add(ThrustsVal[1]);  // FR
    Thrust2Data.Add(ThrustsVal[2]);  // BL
    Thrust3Data.Add(ThrustsVal[3]);  // BR

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
        const ImVec4 FL_COLOR(1.0f, 0.2f, 0.2f, 1.0f);     // Bright red
        const ImVec4 FR_COLOR(0.2f, 1.0f, 0.2f, 1.0f);     // Bright green
        const ImVec4 BL_COLOR(0.2f, 0.6f, 1.0f, 1.0f);     // Bright blue
        const ImVec4 BR_COLOR(1.0f, 0.8f, 0.0f, 1.0f);     // Bright yellow

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
//void ImGuiUtil::RenderImPlot(
//    TArray<float>& ThrustsVal,
//    float rollError, float pitchError,
//    const FRotator& currentRotation,
//    const FVector& waypoint, const FVector& currLoc,
//    const FVector& error,
//    const FVector& currentVelocity,
//    float xOutput, float yOutput, float zOutput,
//    float rollOutput, float pitchOutput, float yawOutput,
//    float deltaTime)
//{
//    // Add time data
//    CumulativeTime += deltaTime;
//    TimeData.Add(CumulativeTime);
//
//    // Add PID outputs to history
//    xPIDOutputHistory.Add(xOutput);
//    yPIDOutputHistory.Add(yOutput);
//    zPIDOutputHistory.Add(zOutput);
//    rollPIDOutputHistory.Add(rollOutput);
//    pitchPIDOutputHistory.Add(pitchOutput);
//    yawPIDOutputHistory.Add(yawOutput);
//
//    // Add error calculations
//    float positionError = error.Size();  // Magnitude of position error
//    float velocityError = (desiredNewVelocity - currentVelocity).Size();  // Magnitude of velocity error
//
//    positionErrorHistory.Add(positionError);
//    velocityErrorHistory.Add(velocityError);
//
//    // Begin ImGui window
//    ImGui::Begin("Drone PID Analysis");
//
//    // Position Control Plot
//    if (ImPlot::BeginPlot("Position Control", ImVec2(600, 300))) {
//        float xMin = FMath::Max(0.0f, CumulativeTime - 10.0f);
//        float xMax = CumulativeTime;
//        ImPlot::SetupAxisLimits(ImAxis_X1, xMin, xMax, ImGuiCond_Always);
//        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
//        ImPlot::SetupAxis(ImAxis_Y1, "Output");
//
//        // Get data in order from oldest to newest
//        TArray<float> TimeDataArray = TimeData.GetDataInOrder();
//        TArray<float> xPIDDataArray = xPIDOutputHistory.GetDataInOrder();
//        TArray<float> yPIDDataArray = yPIDOutputHistory.GetDataInOrder();
//        TArray<float> zPIDDataArray = zPIDOutputHistory.GetDataInOrder();
//        int DataCount = TimeData.Num();
//
//        // Plot X PID
//        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
//        ImPlot::PlotLine("X PID", TimeDataArray.GetData(), xPIDDataArray.GetData(), DataCount);
//        ImPlot::PopStyleColor();
//
//        // Plot Y PID
//        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
//        ImPlot::PlotLine("Y PID", TimeDataArray.GetData(), yPIDDataArray.GetData(), DataCount);
//        ImPlot::PopStyleColor();
//
//        // Plot Z PID
//        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
//        ImPlot::PlotLine("Z PID", TimeDataArray.GetData(), zPIDDataArray.GetData(), DataCount);
//        ImPlot::PopStyleColor();
//
//        ImPlot::EndPlot();
//    }
//
//    // Attitude Control Plot
//    if (ImPlot::BeginPlot("Attitude Control", ImVec2(600, 300))) {
//        float xMin = FMath::Max(0.0f, CumulativeTime - 10.0f);
//        float xMax = CumulativeTime;
//        ImPlot::SetupAxisLimits(ImAxis_X1, xMin, xMax, ImGuiCond_Always);
//        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
//        ImPlot::SetupAxis(ImAxis_Y1, "Output");
//
//        // Get data in order
//        TArray<float> TimeDataArray = TimeData.GetDataInOrder();
//        TArray<float> rollPIDDataArray = rollPIDOutputHistory.GetDataInOrder();
//        TArray<float> pitchPIDDataArray = pitchPIDOutputHistory.GetDataInOrder();
//        TArray<float> yawPIDDataArray = yawPIDOutputHistory.GetDataInOrder();
//        int DataCount = TimeData.Num();
//
//        // Plot Roll PID
//        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.5f, 0.0f, 1.0f));
//        ImPlot::PlotLine("Roll PID", TimeDataArray.GetData(), rollPIDDataArray.GetData(), DataCount);
//        ImPlot::PopStyleColor();
//
//        // Plot Pitch PID
//        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.5f, 0.0f, 1.0f, 1.0f));
//        ImPlot::PlotLine("Pitch PID", TimeDataArray.GetData(), pitchPIDDataArray.GetData(), DataCount);
//        ImPlot::PopStyleColor();
//
//        // Plot Yaw PID
//        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
//        ImPlot::PlotLine("Yaw PID", TimeDataArray.GetData(), yawPIDDataArray.GetData(), DataCount);
//        ImPlot::PopStyleColor();
//
//        ImPlot::EndPlot();
//    }
//
//    // Error Plot
//    if (ImPlot::BeginPlot("Error Analysis", ImVec2(600, 300))) {
//        float xMin = FMath::Max(0.0f, CumulativeTime - 10.0f);
//        float xMax = CumulativeTime;
//        ImPlot::SetupAxisLimits(ImAxis_X1, xMin, xMax, ImGuiCond_Always);
//        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
//        ImPlot::SetupAxis(ImAxis_Y1, "Error Magnitude");
//
//        // Get data in order
//        TArray<float> TimeDataArray = TimeData.GetDataInOrder();
//        TArray<float> positionErrorDataArray = positionErrorHistory.GetDataInOrder();
//        TArray<float> velocityErrorDataArray = velocityErrorHistory.GetDataInOrder();
//        int DataCount = TimeData.Num();
//
//        // Plot Position Error
//        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.0f, 1.0f, 1.0f));
//        ImPlot::PlotLine("Position Error", TimeDataArray.GetData(), positionErrorDataArray.GetData(), DataCount);
//        ImPlot::PopStyleColor();
//
//        // Plot Velocity Error
//        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.0f, 1.0f, 1.0f, 1.0f));
//        ImPlot::PlotLine("Velocity Error", TimeDataArray.GetData(), velocityErrorDataArray.GetData(), DataCount);
//        ImPlot::PopStyleColor();
//
//        ImPlot::EndPlot();
//    }
//
//    ImGui::End();
//}
//-------------------------- Main functionality -------------------------//

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

void ImGuiUtil::DisplayThrusterControls(TArray<float>& ThrustsVal)
{
    ImGui::Separator();
    ImGui::Text("Control All Thrusts");

    static float AllThrustValue = 0.0f;

    if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, 0, maxPIDOutput))
    {
        for (int i = 0; i < dronePawn->Thrusters.Num(); ++i)
        {
            dronePawn->Thrusters[i]->ThrustStrength = AllThrustValue;
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
        // Diagonal 1
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

        // Diagonal 2
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
}

void ImGuiUtil::DisplayPIDSettings(
    QuadPIDController* xPIDParam, QuadPIDController* yPIDParam, QuadPIDController* zPIDParam,
    QuadPIDController* rollPIDParam, QuadPIDController* pitchPIDParam, QuadPIDController* yawPIDParam,
    const char* headerLabel, bool& synchronizeXYGains, bool& synchronizeGains)
{
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

        // Position PID Gains
        ImGui::Text("Position PID Gains");
        ImGui::Checkbox("Synchronize X and Y Axis Gains", &synchronizeXYGains);

        // X Axis
        ImGui::Indent();
        ImGui::Text("X Axis");

        if (synchronizeXYGains)
        {
            DrawPIDGainControl("X P", &xPIDParam->ProportionalGain, 0.0f, 10.0f);
            yPIDParam->ProportionalGain = xPIDParam->ProportionalGain;

            DrawPIDGainControl("X I", &xPIDParam->IntegralGain, 0.0f, 10.0f);
            yPIDParam->IntegralGain = xPIDParam->IntegralGain;

            DrawPIDGainControl("X D", &xPIDParam->DerivativeGain, 0.0f, 10.0f);
            yPIDParam->DerivativeGain = xPIDParam->DerivativeGain;
        }
        else
        {
            DrawPIDGainControl("X P", &xPIDParam->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("X I", &xPIDParam->IntegralGain, 0.0f, 10.0f);
            DrawPIDGainControl("X D", &xPIDParam->DerivativeGain, 0.0f, 10.0f);
        }
        ImGui::Unindent();

        // Y Axis
        ImGui::Indent();
        ImGui::Text("Y Axis");

        if (synchronizeXYGains)
        {
            // Note: Synchronize Y gains with X gains
            DrawPIDGainControl("Y P", &yPIDParam->ProportionalGain, 0.0f, 10.0f);
            xPIDParam->ProportionalGain = yPIDParam->ProportionalGain;

            DrawPIDGainControl("Y I", &yPIDParam->IntegralGain, 0.0f, 10.0f);
            xPIDParam->IntegralGain = yPIDParam->IntegralGain;

            DrawPIDGainControl("Y D", &yPIDParam->DerivativeGain, 0.0f, 10.0f);
            xPIDParam->DerivativeGain = yPIDParam->DerivativeGain;
        }
        else
        {
            DrawPIDGainControl("Y P", &yPIDParam->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("Y I", &yPIDParam->IntegralGain, 0.0f, 10.0f);
            DrawPIDGainControl("Y D", &yPIDParam->DerivativeGain, 0.0f, 10.0f);
        }
        ImGui::Unindent();

        // Z Axis
        ImGui::Indent();
        ImGui::Text("Z Axis");
        DrawPIDGainControl("Z P", &zPIDParam->ProportionalGain, 0.0f, 10.0f);
        DrawPIDGainControl("Z I", &zPIDParam->IntegralGain, 0.0f, 10.0f);
        DrawPIDGainControl("Z D", &zPIDParam->DerivativeGain, 0.0f, 10.0f);
        ImGui::Unindent();

        ImGui::Separator();

        // Attitude PID Gains
        ImGui::Text("Attitude PID Gains");
        ImGui::Checkbox("Synchronize Roll and Pitch Gains", &synchronizeGains);

        // Roll
        ImGui::Indent();
        ImGui::Text("Roll");

        if (synchronizeGains)
        {
            DrawPIDGainControl("Roll P", &rollPIDParam->ProportionalGain, 0.0f, 20.0f);
            pitchPIDParam->ProportionalGain = rollPIDParam->ProportionalGain;

            DrawPIDGainControl("Roll I", &rollPIDParam->IntegralGain, 0.0f, 20.0f);
            pitchPIDParam->IntegralGain = rollPIDParam->IntegralGain;

            DrawPIDGainControl("Roll D", &rollPIDParam->DerivativeGain, 0.0f, 20.0f);
            pitchPIDParam->DerivativeGain = rollPIDParam->DerivativeGain;
        }
        else
        {
            DrawPIDGainControl("Roll P", &rollPIDParam->ProportionalGain, 0.0f, 20.0f);
            DrawPIDGainControl("Roll I", &rollPIDParam->IntegralGain, 0.0f, 20.0f);
            DrawPIDGainControl("Roll D", &rollPIDParam->DerivativeGain, 0.0f, 20.0f);
        }
        ImGui::Unindent();

        // Pitch
        ImGui::Indent();
        ImGui::Text("Pitch");

        if (synchronizeGains)
        {
            DrawPIDGainControl("Pitch P", &pitchPIDParam->ProportionalGain, 0.0f, 20.0f);
            rollPIDParam->ProportionalGain = pitchPIDParam->ProportionalGain;

            DrawPIDGainControl("Pitch I", &pitchPIDParam->IntegralGain, 0.0f, 20.0f);
            rollPIDParam->IntegralGain = pitchPIDParam->IntegralGain;

            DrawPIDGainControl("Pitch D", &pitchPIDParam->DerivativeGain, 0.0f, 20.0f);
            rollPIDParam->DerivativeGain = pitchPIDParam->DerivativeGain;
        }
        else
        {
            DrawPIDGainControl("Pitch P", &pitchPIDParam->ProportionalGain, 0.0f, 20.0f);
            DrawPIDGainControl("Pitch I", &pitchPIDParam->IntegralGain, 0.0f, 20.0f);
            DrawPIDGainControl("Pitch D", &pitchPIDParam->DerivativeGain, 0.0f, 20.0f);
        }
        ImGui::Unindent();

        // Yaw
        ImGui::Indent();
        ImGui::Text("Yaw");
        DrawPIDGainControl("Yaw P", &yawPIDParam->ProportionalGain, 0.0f, 20.0f);
        DrawPIDGainControl("Yaw I", &yawPIDParam->IntegralGain, 0.0f, 20.0f);
        DrawPIDGainControl("Yaw D", &yawPIDParam->DerivativeGain, 0.0f, 20.0f);
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
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), xPIDParam->ProportionalGain, xPIDParam->IntegralGain, xPIDParam->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), yPIDParam->ProportionalGain, yPIDParam->IntegralGain, yPIDParam->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), zPIDParam->ProportionalGain, zPIDParam->IntegralGain, zPIDParam->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), rollPIDParam->ProportionalGain, rollPIDParam->IntegralGain, rollPIDParam->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f,"), pitchPIDParam->ProportionalGain, pitchPIDParam->IntegralGain, pitchPIDParam->DerivativeGain);
            GainData += FString::Printf(TEXT("%.3f,%.3f,%.3f"), yawPIDParam->ProportionalGain, yawPIDParam->IntegralGain, yawPIDParam->DerivativeGain);

            FFileHelper::SaveStringToFile(GainData + TEXT("\n"), *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
        }
    }
}

void ImGuiUtil::DisplayCameraControls()
{
    ImGui::Separator();
    ImGui::Spacing();

    //ImGui::Text("Camera Mode: %s", dronePawn->CameraFPV->IsActive() ? "First Person" : "Third Person");

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
        if (dronePawn)
        {
            dronePawn->SetActorLocation(FVector(0.0f, 0.0f, 10000.0f), false, nullptr, ETeleportType::TeleportPhysics);
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
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
            dronePawn->SetActorLocation(FVector(0.0f, 0.0f, 10.f), false, nullptr, ETeleportType::TeleportPhysics);
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
                droneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                droneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                droneBody->WakeAllRigidBodies();
            }

            altitudeReached = false;
            initialTakeoff = true;
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
    velocityChanged |= ImGui::SliderFloat("Desired Velocity Z", &tempVz, 0, maxVelocity);

    // If any velocity slider was changed, update desired velocity
    if (velocityChanged)
    {
        desiredNewVelocity.X = tempVx;
        desiredNewVelocity.Y = tempVy;
        desiredNewVelocity.Z = tempVz;

        if (controller)
        {
            controller->SetDesiredVelocity(desiredNewVelocity);
            controller->SetFlightMode(UQuadDroneController::FlightMode::VelocityControl);
        }
    }

    ImGui::Separator();
}

