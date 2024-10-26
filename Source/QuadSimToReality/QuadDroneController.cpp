// QuadDroneController.cpp

#include "QuadDroneController.h"
#include "QuadPawn.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"
#include "string"
#include "Math/UnrealMathUtility.h"

#define ACCEPTABLE_DIST 200


void QuadDroneController::ThrustMixer(float xOutput,float yOutput,float zOutput,float rollOutput,float pitchOutput)
{
    //Order: FL,FR,BL,BR
    Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
    Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
    Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
    Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
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


QuadDroneController::QuadDroneController(AQuadPawn* InPawn)
    : dronePawn(InPawn), currentNav(nullptr), curPos(0)
{
    Thrusts.SetNum(4);
    // 	float positionProportionalGain = 2.5f;
    // 	float positionIntegralGain = 0.3f;
    // 	float positionDerivativeGain = 0.16f;
    //
    // 	float attitudeProportionalGain = 0.1f;
    // 	float attitudeIntegralGain = 0.19f;
    // 	float attitudeDerivativeGain = 10.f;//4.1320f;
    
    xPID = new QuadPIDController();
    xPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    xPID->SetGains(2.85f,  0.457f, 0.643f);
    
    yPID = new QuadPIDController();
    yPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    yPID->SetGains(2.85f,  0.457f, 0.643f);
    
    zPID = new QuadPIDController();
    zPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    zPID->SetGains(4.25f,  0.623f, 0.870f);
    
    pitchAttitudePID = new QuadPIDController();
    pitchAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    pitchAttitudePID->SetGains(2.934f, 0.297f, 3.633f);

    rollAttitudePID = new QuadPIDController();
    rollAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    rollAttitudePID->SetGains(2.934f, 0.297f, 3.633f);

    // rollAttitudePID->attitudeControl = true;
    // pitchAttitudePID->attitudeControl = true;
}

QuadDroneController::~QuadDroneController()
{
    delete xPID;
    delete yPID;
    delete zPID;
    delete rollAttitudePID;
    delete pitchAttitudePID;
}

// void QuadDroneController::AddNavPlan(FString Name, TArray<FVector> Waypoints)
// {
//     // Check if a plan with the same name exists
//     for (int i = 0; i < setPointNavigation.Num(); i++)
//     {
//         if (setPointNavigation[i].name == Name)
//         {
//             // Update the existing plan
//             setPointNavigation[i].waypoints = Waypoints;
//             UE_LOG(LogTemp, Log, TEXT("AddNavPlan: Updated existing plan '%s' with %d waypoints"), *Name, Waypoints.Num());
//             return;
//         }
//     }
//     // If not found, add a new plan
//     NavPlan plan;
//     plan.name = Name;
//     plan.waypoints = Waypoints;
//     setPointNavigation.Add(plan);
//     UE_LOG(LogTemp, Log, TEXT("AddNavPlan: Added new plan '%s' with %d waypoints"), *Name, Waypoints.Num());
// }
//
// void QuadDroneController::SetNavPlan(FString Name)
// {
//     for (int i = 0; i < setPointNavigation.Num(); i++)
//     {
//         if (setPointNavigation[i].name == Name)
//         {
//             currentNav = &setPointNavigation[i];
//             curPos = 0;
//             UE_LOG(LogTemp, Log, TEXT("SetNavPlan: Set currentNav to plan '%s' with %d waypoints"), *Name, currentNav->waypoints.Num());
//             return;
//         }
//     }
//     UE_LOG(LogTemp, Warning, TEXT("SetNavPlan: Plan '%s' not found"), *Name);
// }

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
    curPos = 0;
    altitudeReached = false;
}

void QuadDroneController::Update(double a_deltaTime)
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
    FVector positionError = setPoint - currentPosition;
    FVector normalizedError = positionError.GetSafeNormal();
    FVector droneForwardVector = dronePawn->GetActorForwardVector();
    FVector desiredVelocity = CalculateDesiredVelocity(positionError, hoverMode ? 0 : maxVelocity);
    UE_LOG(LogTemp, Log, TEXT("Position Error %f %f %f"),positionError.X,positionError.Y,positionError.Z);
    UE_LOG(LogTemp, Log, TEXT("Position Current %f %f %f"),currentPosition.X,currentPosition.Y,currentPosition.Z);
    UE_LOG(LogTemp, Log, TEXT("Position Setpoint %f %f %f"),setPoint.X,setPoint.Y,setPoint.Z);
    UE_LOG(LogTemp, Log, TEXT("Desured Velocity %f %f %f"),desiredVelocity.X,desiredVelocity.Y,desiredVelocity.Z);
    
    DrawDebugVisuals(currentPosition, setPoint);

    if (!altitudeReached)
    { // Drone must ascend
        if (currentPosition.Z < minAltitudeLocal)
        {
            setPoint = FVector(currentPosition.X, currentPosition.Y, minAltitudeLocal); // Set waypoint to be directly above the drone
        }
        else
        {
            altitudeReached = true;
        }
    }
    
    // Check if the drone has reached the waypoint
    if (positionError.Size() < ACCEPTABLE_DIST)
    {
        // Move to the next waypoint
        if (!altitudeReached)
        {
            altitudeReached = true;
        }
        else
        {
            curPos++;
        }
        if (curPos >= currentNav->waypoints.Num())
        {
            // Reached the end of the nav plan
            currentNav = nullptr;
            curPos = 0;
        }
    }


    // ------------------- Attitude CONTROL ---------------------

    desiredRoll = CalculateDesiredRoll(normalizedError, droneForwardVector, hoverMode ? 0 : maxAngle, altitudeThresh);
    desiredPitch = CalculateDesiredPitch(normalizedError, droneForwardVector, hoverMode ? 0 : maxAngle, altitudeThresh);

    float roll_error = desiredRoll - currentRotation.Roll;
    float pitch_error = desiredPitch - currentRotation.Pitch;

    float roll_output = rollAttitudePID->Calculate(roll_error, a_deltaTime);
    float pitch_output = pitchAttitudePID->Calculate(pitch_error, a_deltaTime);
    
    // ------------------- POSITION CONTROL ---------------------
    float x_output = xPID->Calculate(desiredVelocity.X - currentVelocity.X, a_deltaTime);
    float y_output = yPID->Calculate(desiredVelocity.Y - currentVelocity.Y, a_deltaTime);
    float z_output = zPID->Calculate(desiredVelocity.Z - currentVelocity.Z, a_deltaTime);    
    //------------------- Thrust Mixing -------------
    // putting everything together
    ThrustMixer(x_output,y_output,z_output,roll_output,pitch_output);
    UE_LOG(LogTemp, Log, TEXT("Thrust %f"),Thrusts[0]);
    UE_LOG(LogTemp, Log, TEXT("Drone Mass %f"),droneMass);

    // Update the thrust of each rotor
    for (int i = 0; i < Thrusts.Num(); ++i) {
        dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
    }
    
    //ImGui
    RenderImGui(Thrusts, roll_error, pitch_error, currentRotation, setPoint, currentPosition, positionError, desiredVelocity, currentVelocity, x_output, y_output, z_output);
}


void QuadDroneController::DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint) const
{
    if (Debug_DrawDroneCollisionSphere)
    {
        DrawDebugSphere(dronePawn->GetWorld(), currentPosition, dronePawn->DroneBody->GetCollisionShape().GetSphereRadius(), 10, FColor::Red, false, 0.1f);
    }
    if (Debug_DrawDroneWaypoint)
    {
        DrawDebugSphere(dronePawn->GetWorld(), setPoint, ACCEPTABLE_DIST, 10, FColor::Red, false, 0.1f);
        DrawDebugLine(dronePawn->GetWorld(), currentPosition, setPoint, FColor::Green, false, 0.1f);

        // // Draw all waypoints
        // if (currentNav)
        // {
        //     for (const FVector& waypoint : currentNav->waypoints)
        //     {
        //         DrawDebugSphere(dronePawn->GetWorld(), waypoint, 50.0f, 10, FColor::Blue, false, 0.1f);
        //     }
        // }
    }
}

void QuadDroneController::RenderImGui(
    TArray<float>& ThrustsVal,
    float rollError, float pitchError,
    const FRotator& currentRotation,
    const FVector& waypoint, const FVector& currLoc,
    const FVector& error, const FVector& desiredVelocity,
    const FVector& currentVelocity,
    float xOutput, float yOutput, float zOutput)
{
    ImGui::Begin("Drone Controller");

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

    ImGui::Text("Thruster Power");
    if (ThrustsVal.Num() >= 4)
    {
        ImGui::SliderFloat("Front Left", &ThrustsVal[0], -maxPIDOutput, maxPIDOutput);
        ImGui::SliderFloat("Front Right", &ThrustsVal[1], -maxPIDOutput, maxPIDOutput);
        ImGui::SliderFloat("Rear Left", &ThrustsVal[2], -maxPIDOutput, maxPIDOutput);
        ImGui::SliderFloat("Rear Right", &ThrustsVal[3], -maxPIDOutput, maxPIDOutput);
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
        DrawPIDGainControl("X P", &xPID->ProportionalGain, 0.0f, 5.0f);
        DrawPIDGainControl("X I", &xPID->IntegralGain, 0.0f, 1.0f);
        DrawPIDGainControl("X D", &xPID->DerivativeGain, 0.0f, 1.0f);
        ImGui::Unindent();

        // Y Axis
        ImGui::Indent();
        ImGui::Text("Y Axis");
        DrawPIDGainControl("Y P", &yPID->ProportionalGain, 0.0f, 5.0f);
        DrawPIDGainControl("Y I", &yPID->IntegralGain, 0.0f, 1.0f);
        DrawPIDGainControl("Y D", &yPID->DerivativeGain, 0.0f, 1.0f);
        ImGui::Unindent();

        // Z Axis
        ImGui::Indent();
        ImGui::Text("Z Axis");
        DrawPIDGainControl("Z P", &zPID->ProportionalGain, 0.0f, 5.0f);
        DrawPIDGainControl("Z I", &zPID->IntegralGain, 0.0f, 1.0f);
        DrawPIDGainControl("Z D", &zPID->DerivativeGain, 0.0f, 1.0f);
        ImGui::Unindent();

        ImGui::Separator();

        // Attitude PID Gains
        ImGui::Text("Attitude PID Gains");

        // Roll
        ImGui::Indent();
        ImGui::Text("Roll");
        DrawPIDGainControl("Roll P", &rollAttitudePID->ProportionalGain, 0.0f, 5.0f);
        DrawPIDGainControl("Roll I", &rollAttitudePID->IntegralGain, 0.0f, 1.0f);
        DrawPIDGainControl("Roll D", &rollAttitudePID->DerivativeGain, 0.0f, 10.0f);
        ImGui::Unindent();

        // Pitch
        ImGui::Indent();
        ImGui::Text("Pitch");
        DrawPIDGainControl("Pitch P", &pitchAttitudePID->ProportionalGain, 0.0f, 5.0f);
        DrawPIDGainControl("Pitch I", &pitchAttitudePID->IntegralGain, 0.0f, 1.0f);
        DrawPIDGainControl("Pitch D", &pitchAttitudePID->DerivativeGain, 0.0f, 10.0f);
        ImGui::Unindent();
    }

    // Display Position PID Outputs
    ImGui::Separator();
    ImGui::Text("Position PID Outputs");
    ImGui::Text("X Output: %.2f", xOutput);
    ImGui::Text("Y Output: %.2f", yOutput);
    ImGui::Text("Z Output: %.2f", zOutput);
    ImGui::Separator();

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
