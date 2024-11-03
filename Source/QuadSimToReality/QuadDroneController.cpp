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
    : desiredYaw(0.0f),
      bDesiredYawInitialized(false),
      hoverThrustLevel(maxPIDOutput / 2.0f),
      desiredAltitude(0.0f),
      bDesiredAltitudeInitialized(false),
      dronePawn(InPawn),
      currentNav(nullptr),
      curPos(0)
{
    Thrusts.SetNum(4);

    // GO BACK TO THESE IF NEED BE 
    // xPID = new QuadPIDController();
    // xPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    // xPID->SetGains(1.f,  0.f, 0.1f);
    //
    // yPID = new QuadPIDController();
    // yPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    // yPID->SetGains(1.f,  0.f, 0.1f);
    //
    // zPID = new QuadPIDController();
    // zPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    // zPID->SetGains(5.f,  1.f, 0.1f);
    //
    // pitchAttitudePID = new QuadPIDController();
    // pitchAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    // pitchAttitudePID->SetGains(2.934f, 0.297f, 3.633f);
    //
    // rollAttitudePID = new QuadPIDController();
    // rollAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    // rollAttitudePID->SetGains(2.934f, 0.297f, 3.633f);
    //
    // yawAttitudePID = new QuadPIDController();
    // yawAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    // yawAttitudePID->SetGains(0.f, 0.f, 0.f);
    
    xPID = new QuadPIDController();
    xPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    xPID->SetGains(2.329f,  3.626f, 1.832f);
    
    yPID = new QuadPIDController();
    yPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    yPID->SetGains(2.329f,  3.626f, 1.832f);

    zPID = new QuadPIDController();
    zPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    zPID->SetGains(5.344f,  1.f, 0.1f);
    
    pitchAttitudePID = new QuadPIDController();
    pitchAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    pitchAttitudePID->SetGains(11.755f, 5.267f, 9.008f);

    rollAttitudePID = new QuadPIDController();
    rollAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    rollAttitudePID->SetGains(11.755f, 5.267f, 9.008f);
    
    yawAttitudePID = new QuadPIDController();
    yawAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    yawAttitudePID->SetGains(0.f, 0.f, 0.f);
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
    
    // Thrusts[0] = zOutput;// - xOutput + yOutput + rollOutput + pitchOutput;
    // Thrusts[1] = zOutput;// - xOutput - yOutput - rollOutput + pitchOutput;
    // Thrusts[2] = zOutput;// + xOutput + yOutput + rollOutput - pitchOutput;
    // Thrusts[3] = zOutput;// + xOutput - yOutput - rollOutput - pitchOutput;
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

void QuadDroneController::HandleThrustInput(float Value)
{
    // Value ranges from -1 to 1
    thrustInput = Value; // Store the input value for use in ApplyControllerInput
}

void QuadDroneController::HandleYawInput(float Value)
{
    yawInput = Value; // Store the raw input for later use with interpolation
}

void QuadDroneController::HandlePitchInput(float Value)
{
    pitchInput = Value; // Map Value to desired pitch range
}

void QuadDroneController::HandleRollInput(float Value)
{
    rollInput = Value; // Map Value to desired roll range
}

void QuadDroneController::SmoothRotateTowardsWaypoint(const FVector& waypoint, float deltaTime)
{
    if (!dronePawn) return;

    // Get the current position and rotation of the drone
    FVector currentPosition = dronePawn->GetActorLocation();
    FRotator currentRotation = dronePawn->GetActorRotation();
 
    // Calculate the direction from the drone to the waypoint (ignore Z for yaw control)
    FVector directionToWaypoint = (waypoint - currentPosition).GetSafeNormal2D();

    // Calculate the desired yaw angle based on the direction vector
    float targetYaw = FMath::RadiansToDegrees(FMath::Atan2(directionToWaypoint.Y, directionToWaypoint.X));

    // Create a target rotation that only changes the yaw (preserves current pitch and roll)
    FRotator targetRotation = FRotator(currentRotation.Pitch, targetYaw, currentRotation.Roll);

    // Interpolate the yaw smoothly to the target yaw
    FRotator newRotation = FMath::RInterpTo(currentRotation, targetRotation, deltaTime, 0.5f); // Adjust interp speed as needed

    // Apply the new rotation to the drone
    dronePawn->SetActorRotation(newRotation);
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
    }
    // Release Input Button
    if (ImGui::Button("Release Input", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            dronePawn->ToggleImguiInput();
        }
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
        ApplyControllerInput(a_deltaTime);
        break;
    }
}
void QuadDroneController::ManualWaypointControl(double a_deltaTime)
{
    
}


void QuadDroneController::ApplyControllerInput(double a_deltaTime)
{
    if (!dronePawn) return;

    float droneMass = dronePawn->DroneBody->GetMass();
    const float mult = 0.5f;

    FVector currentPosition = dronePawn->GetActorLocation();
    FRotator currentRotation = dronePawn->GetActorRotation();

    // Initialize desiredAltitude and desiredYaw on the first run
    if (!bDesiredAltitudeInitialized)
    {
        desiredAltitude = currentPosition.Z;
        bDesiredAltitudeInitialized = true;
    }

    if (!bDesiredYawInitialized)
    {
        desiredYaw = currentRotation.Yaw;
        bDesiredYawInitialized = true;
    }

    // Adjust desiredAltitude based on thrustInput
    float climbRate = 200.0f; // Units per second, adjust as necessary
    desiredAltitude += thrustInput * climbRate * a_deltaTime;

    // Adjust desiredYaw based on yawInput
    float yawRate = 90.0f; // Degrees per second, adjust as necessary
    desiredYaw += yawInput * yawRate * a_deltaTime;

    // Normalize desiredYaw to [-180, 180]
    desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

    // Compute altitude error
    float z_error = desiredAltitude - currentPosition.Z;
    float z_output = zPID->Calculate(z_error, a_deltaTime);

    // Roll Stabilization
    float desiredRoll = rollInput * maxAngle; // Assuming rollInput ranges from -1 to 1
    float roll_error = desiredRoll - currentRotation.Roll;
    float roll_output = rollAttitudePID->Calculate(roll_error, a_deltaTime);

    // Pitch Stabilization
    float desiredPitch = pitchInput * maxAngle; // Assuming pitchInput ranges from -1 to 1
    float pitch_error = desiredPitch - currentRotation.Pitch;
    float pitch_output = pitchAttitudePID->Calculate(pitch_error, a_deltaTime);

    // Yaw Control using RInterpTo
    float yawInterpSpeed = 5.0f; // Adjust interpolation speed as needed
    float newYaw = FMath::FInterpTo(currentRotation.Yaw, desiredYaw, a_deltaTime, yawInterpSpeed);

    // Apply new rotation to the drone
    FRotator newRotation = FRotator(currentRotation.Pitch, newYaw, currentRotation.Roll);
    dronePawn->SetActorRotation(newRotation);

    // Thrust Mixing
    ThrustMixer(0, 0, z_output, roll_output, pitch_output);

    // Apply to rotors
    for (int i = 0; i < Thrusts.Num(); ++i)
    {
        dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
    }

    // Optionally, render ImGui for debugging and visualization
    // Collect data for ImGui
    TArray<float> ThrustsVal = Thrusts;

    // Prepare data for ImGui
    FVector waypoint(0, 0, desiredAltitude); // For visualization
    FVector error(0, 0, z_error);
    FVector desiredVelocity(0, 0, 0); // Not used in manual control
    float xOutput = 0.0f;
    float yOutput = 0.0f;

    // Call RenderImGui
    RenderImGui(ThrustsVal, roll_error, pitch_error, newRotation, waypoint, currentPosition, error, desiredVelocity, dronePawn->GetVelocity(), xOutput, yOutput, z_output, a_deltaTime);
}



void QuadDroneController::AutoWaypointControl(double a_deltaTime)
{
    if (!currentNav || curPos >= currentNav->waypoints.Num()) return;

    
    // Unreal Feedback and data collection
    float droneMass = dronePawn->DroneBody->GetMass();
    const float mult = 0.5f;
    FVector currentPosition = dronePawn->GetActorLocation();
    FRotator currentRotation = dronePawn->GetActorRotation();
    FVector currentVelocity = dronePawn->GetVelocity();
    
    FVector setPoint = currentNav->waypoints[curPos];
    SmoothRotateTowardsWaypoint(setPoint, a_deltaTime);

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
    //float yawTorque = CalculateYawTorque(setPoint, currentPosition, currentRotation, a_deltaTime);
    float desiredRoll = CalculateDesiredRoll(normalizedError, droneForwardVector, maxAngle, altitudeThresh);
    float desiredPitch = CalculateDesiredPitch(normalizedError, droneForwardVector, maxAngle, altitudeThresh);

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
    //ApplyTorqueToRotors(yawTorque);
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
        } else {
            // Individual X control
            DrawPIDGainControl("X P", &xPID->ProportionalGain, 0.0f, 10.0f);
            DrawPIDGainControl("X I", &xPID->IntegralGain, 0.0f, 10.0f);
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
        } else {
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
        } else {
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
        } else {
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
    if (ImGui::Button("Reset Drone 0 point", ImVec2(200, 100)))
    {
        if (dronePawn)
        {
            // Reset position to (-1110, 3040, 0)
            dronePawn->SetActorLocation(FVector(-1110.0f, 3040.0f, 0.0f));

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