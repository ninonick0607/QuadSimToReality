// QuadDroneController.cpp

#include "QuadDroneController.h"
#include "QuadPawn.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"
#include "implot.h"
#include "string"
#include "Math/UnrealMathUtility.h"

#define ACCEPTABLE_DIST 200

enum class FlightMode
{
    None,
    AutoWaypoint,
    ManualWaypoint,
    ManualFlightControl
};

FlightMode currentFlightMode = FlightMode::None;

QuadDroneController::QuadDroneController(AQuadPawn* InPawn)
    : dronePawn(InPawn), currentNav(nullptr), curPos(0)
{
    Thrusts.SetNum(4);

    xPID = new QuadPIDController();
    xPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    xPID->SetGains(1.f,  0.f, 0.1f);
    
    yPID = new QuadPIDController();
    yPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    yPID->SetGains(1.f,  0.f, 0.1f);
    
    zPID = new QuadPIDController();
    zPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    zPID->SetGains(5.f,  1.f, 0.1f);
    
    pitchAttitudePID = new QuadPIDController();
    pitchAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    pitchAttitudePID->SetGains(2.934f, 0.297f, 3.633f);

    rollAttitudePID = new QuadPIDController();
    rollAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    rollAttitudePID->SetGains(2.934f, 0.297f, 3.633f);
    
    yawAttitudePID = new QuadPIDController();
    yawAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    yawAttitudePID->SetGains(2.934f, 0.297f, 3.633f);

}

QuadDroneController::~QuadDroneController()
{
    delete xPID;
    delete yPID;
    delete zPID;
    delete rollAttitudePID;
    delete pitchAttitudePID;
    delete yawAttitudePID;

}

void QuadDroneController::AddNavPlan(FString name, TArray<FVector> waypoints)
{
    NavPlan plan;
    plan.name = name;
    plan.waypoints = waypoints;
    setPointNavigation.Add(plan);
}

void QuadDroneController::SetNavPlan(FString name)
{
    for (int i = 0; i < setPointNavigation.Num(); i++) {
        if (setPointNavigation[i].name == name) {
            currentNav = &setPointNavigation[i];
            curPos = 0;
            return;
        }
    }
}
void QuadDroneController::Reset()
{
    xPID->Reset();
    yPID->Reset();
    zPID->Reset();
    rollAttitudePID->Reset();
    pitchAttitudePID->Reset();
    yawAttitudePID->Reset();

    curPos = 0;
    altitudeReached = false;
}

void QuadDroneController::ThrustMixer(float xOutput,float yOutput,float zOutput,float rollOutput,float pitchOutput)
{
    //Order: FL,FR,BL,BR
    Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
    Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
    Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
    Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
    for (int i = 0; i < Thrusts.Num(); ++i) {
        Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, 600.f);
    }
}

inline FVector CalculateDesiredVelocity(const FVector& error, float maxVelocity)
{
    FVector desired_velocity = error.GetSafeNormal() * maxVelocity;
    return desired_velocity;
}

inline float CalculateDesiredRoll(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold)
{
    FVector horizontalError = FVector(normalizedError.X, normalizedError.Y, 0.0f);
    FVector horizontalNormalizedError = horizontalError.GetSafeNormal();

    if (FMath::Abs(normalizedError.Z) > altitudeThreshold)
    {
        return 0.0f;
    }
    else
    {
        float calculatedRoll = FMath::Atan2(normalizedError.Y, FVector::DotProduct(horizontalNormalizedError, droneForwardVector)) * FMath::RadiansToDegrees(1);
        return FMath::Clamp(calculatedRoll, -maxTilt, maxTilt);
    }
}

inline float CalculateDesiredPitch(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold)
{
    FVector horizontalError = FVector(normalizedError.X, normalizedError.Y, 0.0f);
    FVector horizontalNormalizedError = horizontalError.GetSafeNormal();

    if (FMath::Abs(normalizedError.Z) > altitudeThreshold)
    {
        return 0.0f;
    }
    else
    {
        float calculatedPitch = FMath::Atan2(-normalizedError.X, FVector::DotProduct(horizontalNormalizedError, droneForwardVector)) * FMath::RadiansToDegrees(1);
        return FMath::Clamp(calculatedPitch, -maxTilt, maxTilt);
    }
}


void QuadDroneController::Update(double a_deltaTime)
{
    // ImGui Menu for Flight Mode Selection
    ImGui::Begin("Flight Mode Selector");

    if (ImGui::Button("Auto Waypoint", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::AutoWaypoint;
        curPos = 0; // Reset to start navigation
    }

    if (ImGui::Button("Manual Waypoint", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::ManualWaypoint;
        // Additional setup for manual waypoint mode can be added here
    }

    if (ImGui::Button("Manual Flight Control", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::ManualFlightControl;
        // Setup for manual flight control (controller input) will go here
    }

    ImGui::End();

    switch (currentFlightMode)
    {
    case FlightMode::None:
        // Drone stays idle until a mode is selected
            return;

    case FlightMode::AutoWaypoint:
        AutoWaypointControl(a_deltaTime);
        break;

    case FlightMode::ManualWaypoint:
        ManualWaypointControl(a_deltaTime);
        break;

    case FlightMode::ManualFlightControl:
        // Placeholder for future manual flight control logic
            break;
    }
}
void QuadDroneController::ManualWaypointControl(double a_deltaTime)
{
    
}


void QuadDroneController::AutoWaypointControl(double a_deltaTime)
{
    if (!currentNav || curPos >= currentNav->waypoints.Num()) return;

    
    // Unreal Feedback and data collection
    float desiredRoll,desiredPitch;
    float droneMass = dronePawn->DroneBody->GetMass();
    const float mult = 0.5f;
    FVector currentPosition = dronePawn->GetActorLocation();
    FRotator currentRotation = dronePawn->GetActorRotation();
    FVector currentVelocity = dronePawn->GetVelocity();
    
    FVector setPoint = currentNav->waypoints[curPos];
    
    if (!altitudeReached)
    {
        // Drone must ascend to minAltitudeLocal
        setPoint = FVector(currentPosition.X, currentPosition.Y, minAltitudeLocal); // Set waypoint directly above the drone
    }
    else
    {
        // Proceed with current waypoint
        setPoint = currentNav->waypoints[curPos];
    }

    // Calculate position error and other variables based on the current setPoint
    FVector positionError = setPoint - currentPosition;

    // Check if the drone has reached the setPoint
    if (positionError.Size() < ACCEPTABLE_DIST)
    {
        if (!altitudeReached)
        {
            altitudeReached = true;
            // After reaching minAltitudeLocal, update the setPoint to the next waypoint
            setPoint = currentNav->waypoints[curPos];
            positionError = setPoint - currentPosition;

            // Reset the integral sums of PID controllers
            xPID->ResetIntegral();
            yPID->ResetIntegral();
            zPID->ResetIntegral();
            rollAttitudePID->ResetIntegral();
            pitchAttitudePID->ResetIntegral();
            yawAttitudePID->ResetIntegral();
        }
        else
        {
            // Move to the next waypoint
            curPos++;
            if (curPos >= currentNav->waypoints.Num())
            {
                // Reached the end of the nav plan
                currentNav = nullptr;
                curPos = 0;
                // Stop updating if no more waypoints
                return;
            }
            else
            {
                // Update setPoint and positionError for the new waypoint
                setPoint = currentNav->waypoints[curPos];
                positionError = setPoint - currentPosition;

                // Reset the integral sums of PID controllers
                xPID->ResetIntegral();
                yPID->ResetIntegral();
                zPID->ResetIntegral();
                rollAttitudePID->ResetIntegral();
                pitchAttitudePID->ResetIntegral();
                yawAttitudePID->ResetIntegral();
            }
        }
    }

    // Continue with calculations using the updated positionError
    FVector normalizedError = positionError.GetSafeNormal();
    FVector droneForwardVector = dronePawn->GetActorForwardVector();
    FVector desiredVelocity = CalculateDesiredVelocity(positionError, maxVelocity);

    DrawDebugVisuals(currentPosition, setPoint);

    // ------------------- Attitude CONTROL ---------------------

    desiredRoll = CalculateDesiredRoll(normalizedError, droneForwardVector, maxAngle, altitudeThresh);
    desiredPitch = CalculateDesiredPitch(normalizedError, droneForwardVector, maxAngle, altitudeThresh);

    float roll_error = desiredRoll - currentRotation.Roll;
    float pitch_error = desiredPitch - currentRotation.Pitch;

    float roll_output = rollAttitudePID->Calculate(roll_error, a_deltaTime);
    float pitch_output = pitchAttitudePID->Calculate(pitch_error, a_deltaTime);
    float yaw_output = yawAttitudePID->Calculate(pitch_error, a_deltaTime);
    
    // ------------------- POSITION CONTROL ---------------------
    float x_output = xPID->Calculate(desiredVelocity.X - currentVelocity.X, a_deltaTime);
    float y_output = yPID->Calculate(desiredVelocity.Y - currentVelocity.Y, a_deltaTime);
    float z_output = zPID->Calculate(desiredVelocity.Z - currentVelocity.Z, a_deltaTime);    
    //------------------- Thrust Mixing -------------
    
    ThrustMixer(x_output,y_output,z_output,roll_output,pitch_output);
    
    // Update the thrust of each rotor
    for (int i = 0; i < Thrusts.Num(); ++i) {
        dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
    }
    
    //ImGui
    RenderImGui(Thrusts, roll_error, pitch_error, currentRotation, setPoint, currentPosition, positionError, desiredVelocity, currentVelocity, x_output, y_output, z_output,a_deltaTime);
    
    RenderImPlot(Thrusts, roll_error, pitch_error, currentRotation, setPoint, currentPosition, positionError, desiredVelocity, currentVelocity, x_output, y_output, z_output,a_deltaTime);

}
void QuadDroneController::RenderImPlot(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error, const FVector& desiredVelocity,
    const FVector& currentVelocity,
    float xOutput, float yOutput, float zOutput, float deltaTime)
{
    CumulativeTime += deltaTime;
    TimeData.Add(CumulativeTime);

    waypointArrayX.Add(waypoint.X);
    waypointArrayY.Add(waypoint.Y);
    waypointArrayZ.Add(waypoint.Z);

    currentPosArrayX.Add(currLoc.X);
    currentPosArrayY.Add(currLoc.Y);
    currentPosArrayZ.Add(currLoc.Z);

    if (thrustValues.Num() == 0)
    {
        thrustValues.SetNum(ThrustsVal.Num());
    }

    for (int i = 0; i < ThrustsVal.Num(); ++i)
    {
        thrustValues[i].Add(ThrustsVal[i]);
    }

    const int MaxSize = 5000;
    if (TimeData.Num() > MaxSize)
    {
        TimeData.RemoveAt(0);
        waypointArrayX.RemoveAt(0);
        waypointArrayY.RemoveAt(0);
        waypointArrayZ.RemoveAt(0);
        currentPosArrayX.RemoveAt(0);
        currentPosArrayY.RemoveAt(0);
        currentPosArrayZ.RemoveAt(0);

        for (int i = 0; i < thrustValues.Num(); ++i)
        {
            thrustValues[i].RemoveAt(0);
        }
    }

    const float TimeWindow = 10.0f;
    const float TimeOffset = 1.0f;

    ImGui::Begin("Drone PID Plots", nullptr, ImGuiWindowFlags_None);

    auto SetupScrollingXLimits = [&](float cumulativeTime)
    {
        float xMin = cumulativeTime - TimeWindow;
        float xMax = cumulativeTime + TimeOffset;
        if (xMin < 0) xMin = 0;
        ImPlot::SetupAxisLimits(ImAxis_X1, xMin, xMax, ImGuiCond_Always);
    };

    if (ImPlot::BeginPlot("Desired vs. Current Altitude", ImVec2(600, 400)))
    {
        SetupScrollingXLimits(CumulativeTime);

        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
        ImPlot::SetupAxis(ImAxis_Y1, "Altitude (m)");

        ImPlot::PlotLine("Desired Altitude", TimeData.GetData(), waypointArrayZ.GetData(), TimeData.Num());
        ImPlot::PlotLine("Current Altitude", TimeData.GetData(), currentPosArrayZ.GetData(), TimeData.Num());

        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Thrust Values", ImVec2(600, 400)))
    {
        SetupScrollingXLimits(CumulativeTime);

        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
        ImPlot::SetupAxis(ImAxis_Y1, "Thrust (units)");

        for (int i = 0; i < thrustValues.Num(); ++i)
        {
            FString label = FString::Printf(TEXT("Thrust %d"), i + 1);
            ImPlot::PlotLine(TCHAR_TO_ANSI(*label), TimeData.GetData(), thrustValues[i].GetData(), TimeData.Num());
        }

        ImPlot::EndPlot();
    }

    ImGui::End();
}

void QuadDroneController::RenderImGui(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error, const FVector& desiredVelocity,
    const FVector& currentVelocity,
    float xOutput, float yOutput, float zOutput,float deltaTime)
{
    ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver); // Reduced window size

    ImGui::Begin("Drone Controller", nullptr, ImGuiWindowFlags_AlwaysVerticalScrollbar); // Enable vertical scroll

    // Basic Model Feedbackg
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
    
    if (ImGui::SliderFloat("All Thrusts", &AllThrustValue, -maxPIDOutput, maxPIDOutput))
    {
        // Set all thrusts to the value from the slider
        for (int i = 0; i < Thrusts.Num(); ++i)
        {
            Thrusts[i] = AllThrustValue;
        }
    }
    ImGui::Separator();
    ImGui::Text("Thruster Power");
        
    if (ThrustsVal.Num() >= 4)
    {
        ImGui::SliderFloat("Front Left", &ThrustsVal[0], 0, maxPIDOutput);
        ImGui::SliderFloat("Front Right", &ThrustsVal[1], 0, maxPIDOutput);
        ImGui::SliderFloat("Rear Left", &ThrustsVal[2], 0, maxPIDOutput);
        ImGui::SliderFloat("Rear Right", &ThrustsVal[3], 0, maxPIDOutput);
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
        // Helper function to draw SliderFloat and InputFloat side by side
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

        // X Axis
        ImGui::Indent();
        ImGui::Text("X Axis");
        DrawPIDGainControl("X P", &xPID->ProportionalGain, 0.0f, 10.0f);
        DrawPIDGainControl("X I", &xPID->IntegralGain, 0.0f, 10.0f);
        DrawPIDGainControl("X D", &xPID->DerivativeGain, 0.0f, 10.0f);
        ImGui::Unindent();

        // Y Axis
        ImGui::Indent();
        ImGui::Text("Y Axis");
        DrawPIDGainControl("Y P", &yPID->ProportionalGain, 0.0f, 10.0f);
        DrawPIDGainControl("Y I", &yPID->IntegralGain, 0.0f, 10.0f);
        DrawPIDGainControl("Y D", &yPID->DerivativeGain, 0.0f, 10.0f);
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

        // Roll
        ImGui::Indent();
        ImGui::Text("Roll");
        DrawPIDGainControl("Roll P", &rollAttitudePID->ProportionalGain, 0.0f, 20.0f);
        DrawPIDGainControl("Roll I", &rollAttitudePID->IntegralGain, 0.0f, 20.0f);
        DrawPIDGainControl("Roll D", &rollAttitudePID->DerivativeGain, 0.0f, 20.0f);
        ImGui::Unindent();

        // Pitch
        ImGui::Indent();
        ImGui::Text("Pitch");
        DrawPIDGainControl("Pitch P", &pitchAttitudePID->ProportionalGain, 0.0f, 20.0f);
        DrawPIDGainControl("Pitch I", &pitchAttitudePID->IntegralGain, 0.0f, 20.0f);
        DrawPIDGainControl("Pitch D", &pitchAttitudePID->DerivativeGain, 0.0f, 20.0f);
        ImGui::Unindent();
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
    if (ImGui::Button("Reset Drone", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            // Reset position to (0, 0, 10000)
            dronePawn->SetActorLocation(FVector(0.0f, 0.0f, 10000.0f));

            // Reset rotation to zero (optional)
            dronePawn->SetActorRotation(FRotator::ZeroRotator);

            // Reset velocities to zero
            UPrimitiveComponent* droneBody = dronePawn->DroneBody;
            if (droneBody)
            {
                // Stop all movement
                droneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                droneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
            }

            // Optionally, reset any other physical states or flags
            // For example, reset altitudeReached or initialTakeoff flags
            altitudeReached = false;
            initialTakeoff = true;

            // Note: Do NOT reset the PID controllers here to preserve gains and internal states
        }
    }

    ImGui::End();
}

void QuadDroneController::DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint) const
{
    if (Debug_DrawDroneCollisionSphere)
    {
        // Get the mesh bounds to calculate the vertical offset
        FBoxSphereBounds MeshBounds = dronePawn->DroneBody->CalcBounds(dronePawn->DroneBody->GetComponentTransform());
        float VerticalOffset = MeshBounds.BoxExtent.Z;

        // Adjust the position upwards
        FVector AdjustedPosition = currentPosition + FVector(0.0f, 0.0f, VerticalOffset);

        // Draw the debug sphere at the adjusted position
        DrawDebugSphere(
            dronePawn->GetWorld(),
            AdjustedPosition,
            dronePawn->DroneBody->GetCollisionShape().GetSphereRadius(),
            10,
            FColor::Red,
            false,     // bPersistentLines
            0.0f       // LifeTime
        );
    }

    if (Debug_DrawDroneWaypoint)
    {
        // Existing code remains the same
        DrawDebugSphere(
            dronePawn->GetWorld(),
            setPoint,
            ACCEPTABLE_DIST,
            10,
            FColor::Red,
            false,     // bPersistentLines
            0.0f       // LifeTime
        );
        DrawDebugLine(
            dronePawn->GetWorld(),
            currentPosition,
            setPoint,
            FColor::Green,
            false,     // bPersistentLines
            0.0f       // LifeTime
        );
    }
}
void QuadDroneController::IncreaseAllThrusts(float Amount)
{
    for (int i = 0; i < Thrusts.Num(); ++i)
    {
        Thrusts[i] += Amount;
        // Clamp the thrust value to ensure it stays within acceptable limits
        Thrusts[i] = FMath::Clamp(Thrusts[i], -maxPIDOutput, maxPIDOutput);
    }
}