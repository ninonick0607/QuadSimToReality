// QuadDroneController.cpp

#include "QuadDroneController.h"
#include "QuadPawn.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"
#include "implot.h"
#include "string"
#include "zmq.hpp"
#include "ImGuiUtil.h"
#include "Math/UnrealMathUtility.h"

#define ACCEPTABLE_DIST 200

const FVector start = FVector(0, 0, 1000);

static TArray<FVector> make_test_dests()
{
    const int step = 1000;
    const int z_step = 200;
    TArray<FVector> xyzSetpoint;
    xyzSetpoint.Add(start);
    for (int i = 0; i < 1000; i++)
    {
        bool z = FMath::RandBool();
        bool x = FMath::RandBool();
        bool y = FMath::RandBool();
        FVector last = xyzSetpoint[xyzSetpoint.Num() - 1];
        float z_base = 1000;
        xyzSetpoint.Add(FVector(last.X + (x ? step : -step), last.Y + (y ? step : -step), z ? z_base + z_step : z_base - z_step));
    }
    return xyzSetpoint;
}

//static TArray<FVector> spiralWaypoints()
//{
//    TArray<FVector> xyzSetpoint;
//    const float startHeight = 1000.0f;  // Height relative to current position (cm)
//    const float maxHeight = 10000.0f;   // Maximum height relative to start (cm)
//    const float radius = 3000.0f;       // Radius of spiral (cm)
//    const float heightStep = 500.0f;    // Vertical distance between loops (cm)
//    const int pointsPerLoop = 8;        // Number of points to create per loop
//    const float angleStep = 2.0f * PI / pointsPerLoop;  // Angle between each point
//
//    // Get current position
//    AQuadPawn* drone = nullptr;
//    FVector currentPos = FVector::ZeroVector;
//
//    // Find the drone in the world
//    for (TActorIterator<AQuadPawn> ActorItr(GWorld); ActorItr; ++ActorItr)
//    {
//        drone = *ActorItr;
//        if (drone)
//        {
//            currentPos = drone->GetActorLocation();
//            break;
//        }
//    }
//
//    // First point at starting height above current position
//    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + startHeight));
//
//    // Calculate number of loops needed
//    int numLoops = FMath::CeilToInt((maxHeight - startHeight) / heightStep);
//
//    // Generate upward spiral
//    for (int loop = 0; loop < numLoops; loop++)
//    {
//        float height = currentPos.Z + startHeight + (loop * heightStep);
//
//        for (int point = 0; point < pointsPerLoop; point++)
//        {
//            float angle = point * angleStep;
//            float x = currentPos.X + radius * FMath::Cos(angle);
//            float y = currentPos.Y + radius * FMath::Sin(angle);
//            xyzSetpoint.Add(FVector(x, y, height));
//        }
//    }
//
//    // Add point at max height above current position
//    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + maxHeight));
//
//    // Generate downward spiral (in reverse order)
//    for (int loop = numLoops - 1; loop >= 0; loop--)
//    {
//        float height = currentPos.Z + startHeight + (loop * heightStep);
//
//        for (int point = pointsPerLoop - 1; point >= 0; point--)
//        {
//            float angle = point * angleStep;
//            float x = currentPos.X + radius * FMath::Cos(angle);
//            float y = currentPos.Y + radius * FMath::Sin(angle);
//            xyzSetpoint.Add(FVector(x, y, height));
//        }
//    }
//
//    // Final point back at starting height above current position
//    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + startHeight));
//
//    return xyzSetpoint;
//}

// ---------------------- Constructor ------------------------

UQuadDroneController::UQuadDroneController(const FObjectInitializer& ObjectInitializer)
    : dronePawn(nullptr),                                // 5
    Thrusts({ 0, 0, 0, 0 }), 
    desiredYaw(0.f),                                  // 1
    bDesiredYawInitialized(false),                    // 2
    desiredAltitude(0.0f),                            // 3
    bDesiredAltitudeInitialized(false),               // 4

                          // 6
    currentFlightMode(FlightMode::None),              // 7

    // setPointNavigation is default-initialized         // 8
    currentNav(nullptr),                              // 9
    curPos(0),                                        // 10

    AutoWaypointHUD(nullptr),                         // 11
    VelocityHUD(nullptr),                             // 12
    JoyStickHUD(nullptr),                             // 13
    ManualThrustHUD(nullptr),                         // 14

    desiredNewVelocity(FVector(0, 0, 0)),             // 15

    maxVelocity(250.0f),                              // 16
    maxAngle(15.f),                                   // 17
    initialTakeoff(true),                             // 18
    altitudeReached(false),                           // 19
    Debug_DrawDroneCollisionSphere(true),             // 20
    Debug_DrawDroneWaypoint(true),                    // 21

    // Static constants are not initialized here         // 22-24

    thrustInput(0.0f),                                // 25
    yawInput(0.0f),                                   // 26
    pitchInput(0.0f),                                 // 27
    rollInput(0.0f),                                  // 28
    bHoverThrustInitialized(false)                    // 29
{


    Thrusts.SetNum(4);

    // PID stuff
    xPID = MakeUnique<QuadPIDController>();
    xPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    xPID->SetGains(1.f,  0.f, 0.1f);
    
    yPID = MakeUnique<QuadPIDController>();
    yPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    yPID->SetGains(1.f,  0.f, 0.1f);
    
    zPID = MakeUnique<QuadPIDController>();
    zPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    zPID->SetGains(5.f,  1.f, 0.1f);
    
    pitchAttitudePID = MakeUnique<QuadPIDController>();
    pitchAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    pitchAttitudePID->SetGains(2.934f, 0.297f, 3.633f);
    
    rollAttitudePID = MakeUnique<QuadPIDController>();
    rollAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
    rollAttitudePID->SetGains(2.934f, 0.297f, 3.633f);
    
    yawAttitudePID = MakeUnique<QuadPIDController>();
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
    xPIDVelocity = MakeUnique<QuadPIDController>();
    xPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
    xPIDVelocity->SetGains(1.f, 0.f, 0.1f);

    yPIDVelocity = MakeUnique<QuadPIDController>();
    yPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
    yPIDVelocity->SetGains(1.f, 0.f, 0.1f);

    zPIDVelocity = MakeUnique<QuadPIDController>();
    zPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
    zPIDVelocity->SetGains(5.f, 1.f, 0.1f);

    pitchAttitudePIDVelocity = MakeUnique<QuadPIDController>();
    pitchAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
    pitchAttitudePIDVelocity->SetGains(2.934f, 0.297f, 3.633f);

    rollAttitudePIDVelocity = MakeUnique<QuadPIDController>();
    rollAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
    rollAttitudePIDVelocity->SetGains(2.934f, 0.297f, 3.633f);

    yawAttitudePIDVelocity = MakeUnique<QuadPIDController>();
    yawAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
    yawAttitudePIDVelocity->SetGains(0.f, 0.f, 0.f);

    // Move by Controller
    xPIDJoyStick = MakeUnique<QuadPIDController>();
    xPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
    xPIDJoyStick->SetGains(2.329f, 3.626f, 1.832f);

    yPIDJoyStick = MakeUnique<QuadPIDController>();
    yPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
    yPIDJoyStick->SetGains(2.329f, 3.626f, 1.832f);

    zPIDJoyStick = MakeUnique<QuadPIDController>();
    zPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
    zPIDJoyStick->SetGains(5.344f, 1.f, 0.1f);

    pitchAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
    pitchAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
    pitchAttitudePIDJoyStick->SetGains(11.755f, 5.267f, 9.008f);

    rollAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
    rollAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
    rollAttitudePIDJoyStick->SetGains(11.755f, 5.267f, 9.008f);

    yawAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
    yawAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
    yawAttitudePIDJoyStick->SetGains(0.f, 0.f, 0.f);



    // Initialize ImGuiUtil instances using MakeUnique
    AutoWaypointHUD = MakeUnique<ImGuiUtil>(
        dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
        Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
        altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
        xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
        pitchAttitudePID.Get(), yawAttitudePID.Get(),
        xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
        rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
        yawAttitudePIDVelocity.Get(),
        xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
        rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
        yawAttitudePIDJoyStick.Get()
    );

    VelocityHUD = MakeUnique<ImGuiUtil>(
        dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
        Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
        altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
        xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
        pitchAttitudePID.Get(), yawAttitudePID.Get(),
        xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
        rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
        yawAttitudePIDVelocity.Get(),
        xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
        rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
        yawAttitudePIDJoyStick.Get()
    );

    JoyStickHUD = MakeUnique<ImGuiUtil>(
        dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
        Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
        altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
        xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
        pitchAttitudePID.Get(), yawAttitudePID.Get(),
        xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
        rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
        yawAttitudePIDVelocity.Get(),
        xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
        rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
        yawAttitudePIDJoyStick.Get()
    );

    ManualThrustHUD = MakeUnique<ImGuiUtil>(
        dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
        Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
        altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
        xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
        pitchAttitudePID.Get(), yawAttitudePID.Get(),
        xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
        rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
        yawAttitudePIDVelocity.Get(),
        xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
        rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
        yawAttitudePIDJoyStick.Get()
    );
}

UQuadDroneController::~UQuadDroneController()
{
}

void UQuadDroneController::Initialize(AQuadPawn* InPawn)
{
    
    dronePawn = InPawn;
   
}
// ------------ Setter and Getter -------------------
void UQuadDroneController::SetDesiredVelocity(const FVector& NewVelocity)
{
    desiredNewVelocity = NewVelocity;
    UE_LOG(LogTemp, Display, TEXT("SetDesiredVelocity called: %s"), *desiredNewVelocity.ToString());

}

void UQuadDroneController::SetFlightMode(FlightMode NewMode)
{
    currentFlightMode = NewMode;
}
UQuadDroneController::FlightMode UQuadDroneController::GetFlightMode() const
{
    return currentFlightMode;
}


// ---------------------- Waypoint Nav ------------------------

void UQuadDroneController::AddNavPlan(FString name, TArray<FVector> waypoints)
{
    NavPlan plan;
    plan.name = name;
    plan.waypoints = waypoints;
    setPointNavigation.Add(plan);
}

void UQuadDroneController::SetNavPlan(FString name)
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

void UQuadDroneController::ResetPID()
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

void UQuadDroneController::ThrustMixer(float xOutput,float yOutput,float zOutput,float rollOutput,float pitchOutput)
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

FVector UQuadDroneController::CalculateDesiredVelocity(const FVector& error, float InMaxVelocity)
{
    // Normalizing position error and multiplying with maxVel to get a smaller desiredVel
    FVector desired_velocity = error.GetSafeNormal() * InMaxVelocity;
    return desired_velocity;
}

float UQuadDroneController::CalculateDesiredRoll(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold)
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

float UQuadDroneController::CalculateDesiredPitch(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold)
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


// ---------------------- Update ------------------------

void UQuadDroneController::Update(double a_deltaTime)
{
    //this->QuadController->AddNavPlan("TestPlan", spiralWaypoints());
    //this->QuadController->SetNavPlan("TestPlan");
    AddNavPlan("TestPlan", make_test_dests());
    SetNavPlan("TestPlan");

    UE_LOG(LogTemp, Display, TEXT("uPDATE called: %s"), *desiredNewVelocity.ToString());

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
    }
    if (ImGui::Button("Move By Velocity", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::VelocityControl;
    }
    if (ImGui::Button("Apply Controller Input", ImVec2(200, 50)))
    {
        currentFlightMode = FlightMode::ManualFlightControl;
    }

    // Adjust ImGui layout
    ImGui::End();


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
        break;
    }
   
}

void UQuadDroneController::ApplyControllerInput(double a_deltaTime)
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

    //JoyStickHUD->JoyStickHud(ThrustsVal, roll_error, pitch_error, currentRotation, waypoint, currentPosition, error, desiredVelocity, dronePawn->GetVelocity(), xOutput, yOutput, z_output, a_deltaTime);
}

void UQuadDroneController::AutoWaypointControl(double a_deltaTime)
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
    
    AutoWaypointHUD->AutoWaypointHud(Thrusts, roll_error, pitch_error, currentRotation, setPoint, currentPosition, positionError, currentVelocity, x_output, y_output, z_output,a_deltaTime); //****
    AutoWaypointHUD->RenderImPlot(Thrusts,a_deltaTime);

}

void UQuadDroneController::ManualThrustControl(double a_deltaTime)
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
        //float rotorTorque = rotorSpinDirections[i] * Thrusts[i] * rotorTorqueConstant;

        // Sum net torque around Z-axis (yaw)
        NetTorque += FVector(0.0f, 0.0f, 0.f);
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
    //ManualThrustHUD->AutoWaypointHud(Thrusts, 0.0f, 0.0f, currentRotation, waypoint, currentPosition, error, desiredVelocity, currentVelocity, xOutput, yOutput, zOutput, a_deltaTime);
}

void UQuadDroneController::VelocityControl(double a_deltaTime)
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
    VelocityHUD->VelocityHud(Thrusts, roll_error, pitch_error, currentRotation, FVector::ZeroVector, dronePawn->GetActorLocation(), FVector::ZeroVector, currentVelocity, x_output, y_output, z_output, a_deltaTime);
    VelocityHUD->RenderImPlot(Thrusts, a_deltaTime);

}

// ---------------------- Controller Handling ------------------------

void UQuadDroneController::HandleThrustInput(float Value)
{
    // Value ranges from -1 to 1
    thrustInput = Value; // Store the input value for use in ApplyControllerInput
}

void UQuadDroneController::HandleYawInput(float Value)
{
    yawInput = Value; // Store the raw input for later use with interpolation
}

void UQuadDroneController::HandlePitchInput(float Value)
{
    pitchInput = Value; // Map Value to desired pitch range
}

void UQuadDroneController::HandleRollInput(float Value)
{
    rollInput = Value; // Map Value to desired roll range
}

void UQuadDroneController::DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint) const
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

void UQuadDroneController::IncreaseAllThrusts(float Amount)
{
    for (int i = 0; i < Thrusts.Num(); ++i)
    {
        Thrusts[i] += Amount;
        // Clamp the thrust value to ensure it stays within acceptable limits
        Thrusts[i] = FMath::Clamp(Thrusts[i], -maxPIDOutput, maxPIDOutput);
    }
}

