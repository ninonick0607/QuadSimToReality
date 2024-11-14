#pragma once

#include "CoreMinimal.h"

// Forward declarations
class AQuadPawn;
class QuadPIDController;
class QuadDroneController;

class QUADSIMTOREALITY_API ImGuiUtil
{
public:


    ImGuiUtil(
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
    );
    ~ImGuiUtil();

    // Your existing methods...
    void AutoWaypointHud(
        TArray<float>& ThrustsVal,
        float rollError,
        float pitchError,
        const FRotator& currentRotation,
        const FVector& waypoint,
        const FVector& currLoc,
        const FVector& error,
        const FVector& desiredVelocity,
        const FVector& currentVelocity,
        float xOutput,
        float yOutput,
        float zOutput,
        float deltaTime
    );

    void VelocityHud(
        TArray<float>& ThrustsVal,
        float rollError,
        float pitchError,
        const FRotator& currentRotation,
        const FVector& waypoint,
        const FVector& currLoc,
        const FVector& error,
        const FVector& desiredVelocity,
        const FVector& currentVelocity,
        float xOutput,
        float yOutput,
        float zOutput,
        float deltaTime
    );

    void JoyStickHud(
        TArray<float>& ThrustsVal,
        float rollError,
        float pitchError,
        const FRotator& currentRotation,
        const FVector& waypoint,
        const FVector& currLoc,
        const FVector& error,
        const FVector& desiredVelocity,
        const FVector& currentVelocity,
        float xOutput,
        float yOutput,
        float zOutput,
        float deltaTime
    );
    void RenderImPlot(TArray<float>& ThrustsVal, float rollError, float pitchError, const FRotator& currentRotation, const FVector& waypoint, const FVector& currLoc, const FVector& error, const FVector& desiredVelocity, const FVector& currentVelocity, float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput, float yawOutput, float deltaTime);

private:
    // Member variables
    AQuadPawn* dronePawn;
    bool& initialTakeoff;
    bool& altitudeReached;
    bool& Debug_DrawDroneCollisionSphere;
    bool& Debug_DrawDroneWaypoint;
    float maxPIDOutput;
    float altitudeThresh;
    float minAltitudeLocal;
    float& maxVelocity;
    float& maxAngle;
    QuadPIDController* xPID;
    QuadPIDController* yPID;
    QuadPIDController* zPID;
    QuadPIDController* rollAttitudePID;
    QuadPIDController* pitchAttitudePID;
    QuadPIDController* yawAttitudePID;
    QuadPIDController* xPIDVelocity;
    QuadPIDController* yPIDVelocity;
    QuadPIDController* zPIDVelocity;
    QuadPIDController* rollAttitudePIDVelocity;
    QuadPIDController* pitchAttitudePIDVelocity;
    QuadPIDController* yawAttitudePIDVelocity;

    QuadPIDController* xPIDJoyStick;
    QuadPIDController* yPIDJoyStick;
    QuadPIDController* zPIDJoyStick;
    QuadPIDController* rollAttitudePIDJoyStick;
    QuadPIDController* pitchAttitudePIDJoyStick;
    QuadPIDController* yawAttitudePIDJoyStick;

    QuadDroneController* controller;
    FVector desiredNewVelocity;

protected:
    // Your existing protected variables...
    TArray<float> TimeData;
    TArray<float> waypointArrayX;
    TArray<float> waypointArrayY;
    TArray<float> waypointArrayZ;
    TArray<float> currentPosArrayX;
    TArray<float> currentPosArrayY;
    TArray<float> currentPosArrayZ;
    TArray<TArray<float>> thrustValues;

    TArray<float> xPIDOutputHistory;
    TArray<float> yPIDOutputHistory;
    TArray<float> zPIDOutputHistory;
    TArray<float> rollPIDOutputHistory;
    TArray<float> pitchPIDOutputHistory;
    TArray<float> yawPIDOutputHistory;
    TArray<float> positionErrorHistory;
    TArray<float> velocityErrorHistory;
    float CumulativeTime = 0.0f;
};