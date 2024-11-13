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
    ManualThrustControl,
    ManualFlightControl,
    VelocityControl
};

FlightMode currentFlightMode = FlightMode::None;

// ---------------------- Constructor ------------------------

QuadDroneController::QuadDroneController(AQuadPawn* InPawn)
    : desiredYaw(0.f),
      bDesiredYawInitialized(false),
      hoverThrustLevel(maxPIDOutput / 2.0f),
      desiredAltitude(0.0f),
      bDesiredAltitudeInitialized(false),
      dronePawn(InPawn),
      currentNav(nullptr),
      curPos(0)
{
    Thrusts.SetNum(4);
    // This all just setup for the gains i have for each flight mode.
    // Each flight mode requires different games due to the differences in how they operate and manuever
    //Auto Waypoint
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
     yawAttitudePID->SetGains(0.f, 0.f, 0.f);

     // Try these
     //xPID = new QuadPIDController();
     //xPID->SetLimits(-maxPIDOutput, maxPIDOutput);
     //xPID->SetGains(2.329f, 3.626f, 1.832f);

     //yPID = new QuadPIDController();
     //yPID->SetLimits(-maxPIDOutput, maxPIDOutput);
     //yPID->SetGains(2.329f, 3.626f, 1.832f);

     //zPID = new QuadPIDController();
     //zPID->SetLimits(-maxPIDOutput, maxPIDOutput);
     //zPID->SetGains(5.344f, 1.f, 0.1f);

     //pitchAttitudePID = new QuadPIDController();
     //pitchAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
     //pitchAttitudePID->SetGains(11.755f, 5.267f, 9.008f);

     //rollAttitudePID = new QuadPIDController();
     //rollAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
     //rollAttitudePID->SetGains(11.755f, 5.267f, 9.008f);

     //yawAttitudePID = new QuadPIDController();
     //yawAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
     //yawAttitudePID->SetGains(0.f, 0.f, 0.f);

    // Move by Velocity
     xPIDVelocity = new QuadPIDController();
     xPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
     xPIDVelocity->SetGains(1.f, 0.f, 0.1f);

     yPIDVelocity = new QuadPIDController();
     yPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
     yPIDVelocity->SetGains(1.f, 0.f, 0.1f);

     zPIDVelocity = new QuadPIDController();
     zPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
     zPIDVelocity->SetGains(5.f, 1.f, 0.1f);

     pitchAttitudePIDVelocity = new QuadPIDController();
     pitchAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
     pitchAttitudePIDVelocity->SetGains(2.934f, 0.297f, 3.633f);

     rollAttitudePIDVelocity = new QuadPIDController();
     rollAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
     rollAttitudePIDVelocity->SetGains(2.934f, 0.297f, 3.633f);

     yawAttitudePIDVelocity = new QuadPIDController();
     yawAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
     yawAttitudePIDVelocity->SetGains(0.f, 0.f, 0.f);

     // Move by Controller
     xPIDJoyStick = new QuadPIDController();
     xPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
     xPIDJoyStick->SetGains(2.329f, 3.626f, 1.832f);

     yPIDJoyStick = new QuadPIDController();
     yPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
     yPIDJoyStick->SetGains(2.329f, 3.626f, 1.832f);

     zPIDJoyStick = new QuadPIDController();
     zPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
     zPIDJoyStick->SetGains(5.344f, 1.f, 0.1f);

     pitchAttitudePIDJoyStick = new QuadPIDController();
     pitchAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
     pitchAttitudePIDJoyStick->SetGains(11.755f, 5.267f, 9.008f);

     rollAttitudePIDJoyStick = new QuadPIDController();
     rollAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
     rollAttitudePIDJoyStick->SetGains(11.755f, 5.267f, 9.008f);

     yawAttitudePIDJoyStick = new QuadPIDController();
     yawAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
     yawAttitudePIDJoyStick->SetGains(0.f, 0.f, 0.f);

}

QuadDroneController::~QuadDroneController()
{
    // Just deleting all my PIDs
    delete xPID;
    delete yPID;
    delete zPID;
    delete rollAttitudePID;
    delete pitchAttitudePID;
    delete yawAttitudePID;

    delete xPIDVelocity;
    delete yPIDVelocity;
    delete zPIDVelocity;
    delete rollAttitudePIDVelocity;
    delete pitchAttitudePIDVelocity;
    delete yawAttitudePIDVelocity;

    delete xPIDJoyStick;
    delete yPIDJoyStick;
    delete zPIDJoyStick;
    delete rollAttitudePIDJoyStick;
    delete pitchAttitudePIDJoyStick;
    delete yawAttitudePIDJoyStick;

}

// ---------------------- Waypoint Nav ------------------------

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

// ---------------------- Reset PIDs ------------------------

void QuadDroneController::Reset()
{
    xPID->Reset();
    yPID->Reset();
    zPID->Reset();
    rollAttitudePID->Reset();
    pitchAttitudePID->Reset();
    yawAttitudePID->Reset();

    xPIDVelocity->Reset();
    yPIDVelocity->Reset();
    zPIDVelocity->Reset();
    rollAttitudePIDVelocity->Reset();
    pitchAttitudePIDVelocity->Reset();
    yawAttitudePIDVelocity->Reset();

    xPIDJoyStick->Reset();
    yPIDJoyStick->Reset();
    zPIDJoyStick->Reset();
    rollAttitudePIDJoyStick->Reset();
    pitchAttitudePIDJoyStick->Reset();
    yawAttitudePIDJoyStick->Reset();
    curPos = 0;
    altitudeReached = false;
}

// ---------------------- Thrust, Desired Vel, Pitch, and Roll ------------------------

void QuadDroneController::ThrustMixer(float xOutput,float yOutput,float zOutput,float rollOutput,float pitchOutput)
{
    //Order: FL,FR,BL,BR

    switch (currentFlightMode)
    {
    case FlightMode::AutoWaypoint :
        Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
        Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
        Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
        Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
        break;
    case FlightMode::VelocityControl:
        Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
        Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
        Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
        Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
        break;
    case FlightMode::ManualFlightControl:
        Thrusts[0] = zOutput + rollOutput + pitchOutput; 
        Thrusts[1] = zOutput - rollOutput + pitchOutput;
        Thrusts[2] = zOutput + rollOutput - pitchOutput; 
        Thrusts[3] = zOutput - rollOutput - pitchOutput; 
        break;
    }


    
    for (int i = 0; i < Thrusts.Num(); ++i) {
        Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, 600.f);
    }
}

inline FVector CalculateDesiredVelocity(const FVector& error, float maxVelocity)
{
    // Normalizing position error and multiplying with maxVel to get a smaller desiredVel
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

// Might remove function, seems redundant when I can just change desiredNewVel on the go anywhere
void QuadDroneController::MoveByVelocity(float vx, float vy, float vz)
{
    desiredNewVelocity = FVector(vx, vy, vz);
    currentFlightMode = FlightMode::VelocityControl;
}

// ---------------------- Update ------------------------

void QuadDroneController::Update(double a_deltaTime)
{
    // ImGui Menu for Flight Mode Selection
    ImGui::Begin("Flight Mode Selector");

    if (ImGui::Button("Auto Waypoint", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::AutoWaypoint;
        curPos = 0; // Reset to start navigation
    }
    if (ImGui::Button("Manual Thrust Control", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::ManualThrustControl;
    }
    if (ImGui::Button("Manual Waypoint", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::ManualWaypoint;
        // Additional setup for manual waypoint mode can be added here
    }
    if (ImGui::Button("Move By Velocity", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::VelocityControl;
        // Additional setup for manual waypoint mode can be added here
    }
    if (ImGui::Button("Apply Controller Input", ImVec2(200, 50)))
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
    // I like switch statements but I dont know the performance behind it, should be O(n) since theres no loop but idk
    switch (currentFlightMode)
    {
    case FlightMode::None:
        return;
    case FlightMode::AutoWaypoint:
        AutoWaypointControl(a_deltaTime);
        break;
    case FlightMode::ManualWaypoint:
        //ManualWaypointControl(a_deltaTime);
        break;
    case FlightMode::ManualFlightControl:
        ApplyControllerInput(a_deltaTime);
        break;
    case FlightMode::ManualThrustControl:
        ManualThrustControl(a_deltaTime);
        break;
    case FlightMode::VelocityControl:
        VelocityControl(a_deltaTime);
    }

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

    // ------ Altitude Control ---------
    // Modify desired altitude based on thrust input
    float altitudeRate = 400.0f; // Units per second for altitude change
    desiredAltitude += thrustInput * altitudeRate * a_deltaTime;

    // Calculate error between desired and current altitude
    float z_error = desiredAltitude - currentPosition.Z;
    float z_output = zPIDJoyStick->Calculate(z_error, a_deltaTime);

    // ------ Attitude Control ---------
    float desiredRoll = rollInput * maxAngle;
    float roll_error = desiredRoll - currentRotation.Roll;
    float roll_output = rollAttitudePIDJoyStick->Calculate(roll_error, a_deltaTime);

    float desiredPitch = pitchInput * maxAngle;
    float pitch_error = desiredPitch - currentRotation.Pitch;
    float pitch_output = pitchAttitudePIDJoyStick->Calculate(pitch_error, a_deltaTime);

    // Yaw control 
    float yawRate = 90.0f; // Degrees per second
    desiredYaw += yawInput * yawRate * a_deltaTime;
    desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

    float yaw_error = desiredYaw - currentRotation.Yaw;
    yaw_error = FMath::UnwindDegrees(yaw_error);
    float yaw_output = yawAttitudePIDJoyStick->Calculate(yaw_error, a_deltaTime);

    // Apply yaw torque
    FVector yawTorque = FVector(0.0f, 0.0f, yaw_output);
    dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);

    // Thrust Mixing
    ThrustMixer(0, 0, z_output, roll_output, pitch_output);

    // Apply thrusts to rotors
    for (int i = 0; i < Thrusts.Num(); ++i)
    {
        dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
    }

    // Collect data for ImGui display
    TArray<float> ThrustsVal = Thrusts;
    FVector waypoint(0, 0, desiredAltitude);
    FVector error(0, 0, z_error);
    FVector desiredVelocity(0, 0, 0);
    float xOutput = 0.0f;
    float yOutput = 0.0f;

    RenderImGuiJoyStick(ThrustsVal, roll_error, pitch_error, currentRotation, waypoint, currentPosition, error, desiredVelocity, dronePawn->GetVelocity(), xOutput, yOutput, z_output, a_deltaTime);
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

    // Check if the drone has reached the setPoint, if so then reset integral sums    
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

    // ------------------- Yaw Control ---------------------

    // Calculate desiredYaw based on the direction to the waypoint
    desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(positionError.Y, positionError.X));

    // Normalize desiredYaw to [-180, 180]
    desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

    // Compute yaw error
    float yaw_error = desiredYaw - currentRotation.Yaw;

    // Normalize yaw error to [-180, 180]
    yaw_error = FMath::UnwindDegrees(yaw_error);

    // Use yaw PID controller to calculate yaw torque
    float yaw_output = yawAttitudePID->Calculate(yaw_error, a_deltaTime);

    // Apply yaw torque directly to the drone body
    FVector yawTorque = FVector(0.0f, 0.0f, yaw_output); // Torque around Z-axis
    dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);
    
    // ------------------- Attitude CONTROL ---------------------
    //float yawTorque = CalculateYawTorque(setPoint, currentPosition, currentRotation, a_deltaTime);
    float desiredRoll = CalculateDesiredRoll(normalizedError, droneForwardVector, maxAngle, altitudeThresh);
    float desiredPitch = CalculateDesiredPitch(normalizedError, droneForwardVector, maxAngle, altitudeThresh);

    float roll_error = desiredRoll - currentRotation.Roll;
    float pitch_error = desiredPitch - currentRotation.Pitch;

    float roll_output = rollAttitudePID->Calculate(roll_error, a_deltaTime);
    float pitch_output = pitchAttitudePID->Calculate(pitch_error, a_deltaTime);
    
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
    RenderImGuiWaypoint(Thrusts, roll_error, pitch_error, currentRotation, setPoint, currentPosition, positionError, desiredVelocity, currentVelocity, x_output, y_output, z_output,a_deltaTime);
    
    RenderImPlot(Thrusts, roll_error, pitch_error, currentRotation, setPoint, currentPosition, positionError, desiredVelocity, currentVelocity, x_output, y_output, z_output,roll_output,pitch_output,yaw_output,a_deltaTime);

}

void QuadDroneController::ManualThrustControl(double a_deltaTime)
{
    if (!dronePawn) return;

    float droneMass = dronePawn->DroneBody->GetMass();
    const float mult = 0.5f;

    // Reset net torque
    FVector NetTorque = FVector::ZeroVector;

    // Apply thrusts and calculate torque
    for (int i = 0; i < Thrusts.Num(); ++i)
    {
        // Apply thrust
        dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];

        // Calculate rotor torque (assumed proportional to thrust)
        float rotorTorque = rotorSpinDirections[i] * Thrusts[i] * rotorTorqueConstant;

        // Sum net torque around Z-axis (yaw)
        NetTorque += FVector(0.0f, 0.0f, rotorTorque);
    }

    // Apply net torque to the drone body
    dronePawn->DroneBody->AddTorqueInRadians(NetTorque, NAME_None, true);

    // Optional: Render ImGui for debugging and visualization
    FVector currentPosition = dronePawn->GetActorLocation();
    FRotator currentRotation = dronePawn->GetActorRotation();
    FVector currentVelocity = dronePawn->GetVelocity();

    FVector waypoint = currentPosition; // No waypoint in manual thrust control
    FVector error(0, 0, 0);
    FVector desiredVelocity(0, 0, 0);
    float xOutput = 0.0f;
    float yOutput = 0.0f;
    float zOutput = 0.0f;

    // Call RenderImGui
    RenderImGuiWaypoint(Thrusts, 0.0f, 0.0f, currentRotation, waypoint, currentPosition, error, desiredVelocity, currentVelocity, xOutput, yOutput, zOutput, a_deltaTime);
}

void QuadDroneController::VelocityControl(double a_deltaTime)
{
    if (!dronePawn) return;

    float droneMass = dronePawn->DroneBody->GetMass();
    const float mult = 0.5f;

    FVector currentVelocity = dronePawn->GetVelocity();
    FRotator currentRotation = dronePawn->GetActorRotation();

    // Calculate velocity error
    FVector velocityError = desiredNewVelocity - currentVelocity;

    // Use PID controllers to calculate outputs
    float x_output = xPIDVelocity->Calculate(velocityError.X, a_deltaTime);
    float y_output = yPIDVelocity->Calculate(velocityError.Y, a_deltaTime);
    float z_output = zPIDVelocity->Calculate(velocityError.Z, a_deltaTime);

    // Attitude stabilization (keeping roll and pitch at zero)
    float roll_error = -currentRotation.Roll;
    float pitch_error = -currentRotation.Pitch;

    float roll_output = rollAttitudePIDVelocity->Calculate(roll_error, a_deltaTime);
    float pitch_output = pitchAttitudePIDVelocity->Calculate(pitch_error, a_deltaTime);

    // Calculate desired yaw based on velocity direction
    FVector horizontalVelocity = desiredNewVelocity;
    horizontalVelocity.Z = 0; // Ignore vertical component for yaw calculation

    // Only update desired yaw if we have significant horizontal velocity
    const float MIN_VELOCITY_FOR_YAW = 10.0f; // Adjust this threshold as needed
    if (horizontalVelocity.SizeSquared() > MIN_VELOCITY_FOR_YAW * MIN_VELOCITY_FOR_YAW)
    {
        // Calculate the desired yaw angle based on velocity direction
        desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(horizontalVelocity.Y, horizontalVelocity.X));
    }
    else
    {
        // If velocity is too low, maintain current yaw to prevent erratic rotation
        desiredYaw = currentRotation.Yaw;
    }

    // Normalize desiredYaw to [-180, 180]
    desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

    // Calculate yaw error
    float yaw_error = desiredYaw - currentRotation.Yaw;
    yaw_error = FMath::UnwindDegrees(yaw_error);  // Normalize the error to [-180, 180]

    // Calculate yaw output using PID
    float yaw_output = yawAttitudePIDVelocity->Calculate(yaw_error, a_deltaTime);

    // Apply yaw torque directly to the drone body
    FVector yawTorque = FVector(0.0f, 0.0f, yaw_output);
    dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);

    // Apply thrust mixing
    ThrustMixer(x_output, y_output, z_output, roll_output, pitch_output);

    // Update rotors
    for (int i = 0; i < Thrusts.Num(); ++i)
    {
        dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
    }

    // Debug visualization for velocity direction
    if (Debug_DrawDroneWaypoint)
    {
        FVector dronePos = dronePawn->GetActorLocation();
        FVector velocityDirection = desiredNewVelocity.GetSafeNormal() * 200.0f; // Scale for visualization

        DrawDebugLine(
            dronePawn->GetWorld(),
            dronePos,
            dronePos + velocityDirection,
            FColor::Yellow,
            false,
            -1.0f,
            0,
            2.0f
        );
    }

    // Optional: Render ImGui for debugging
    RenderImGuiVelocity(
        Thrusts,
        roll_error,
        pitch_error,
        currentRotation,
        FVector::ZeroVector,
        dronePawn->GetActorLocation(),
        FVector::ZeroVector,
        desiredNewVelocity,
        currentVelocity,
        x_output,
        y_output,
        z_output,
        a_deltaTime
    );

    // Update plots
    RenderImPlot(
        Thrusts,
        roll_error,
        pitch_error,
        currentRotation,
        FVector::ZeroVector,
        dronePawn->GetActorLocation(),
        FVector::ZeroVector,
        desiredNewVelocity,
        currentVelocity,
        x_output,
        y_output,
        z_output,
        roll_output,
        pitch_output,
        yaw_output,
        a_deltaTime
    );
}

// ---------------------- Controller Handling ------------------------

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


// ---------------------- Debugging ------------------------

void QuadDroneController::RenderImPlot(
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

void QuadDroneController::RenderImGuiWaypoint(
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
        for (int i = 0; i < Thrusts.Num(); ++i)
        {
            Thrusts[i] = AllThrustValue;
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

void QuadDroneController::RenderImGuiVelocity(
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
        MoveByVelocity(tempVx, tempVy, tempVz);
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
        for (int i = 0; i < Thrusts.Num(); ++i)
        {
            Thrusts[i] = AllThrustValue;
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

void QuadDroneController::RenderImGuiJoyStick(
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
        for (int i = 0; i < Thrusts.Num(); ++i)
        {
            Thrusts[i] = AllThrustValue;
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