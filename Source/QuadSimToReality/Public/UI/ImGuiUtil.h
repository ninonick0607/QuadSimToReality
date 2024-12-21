#pragma once

#include "CoreMinimal.h"

// Forward declarations
class AQuadPawn;
class QuadPIDController;
class UQuadDroneController;

class QUADSIMTOREALITY_API ImGuiUtil
{
public:
    ImGuiUtil(
        AQuadPawn *InPawn,
        UQuadDroneController *InController,
        FVector &IndesiredNewVelocity,
        bool &InInitialTakeoff,
        bool &InAltitudeReached,
        bool &InDebug_DrawDroneCollisionSphere,
        bool &InDebug_DrawDroneWaypoint,
        float InMaxPIDOutput,
        float InAltitudeThresh,
        float InMinAltitudeLocal,
        float &InMaxVelocity,
        float &InMaxAngle,
        QuadPIDController *InXPID,
        QuadPIDController *InYPID,
        QuadPIDController *InZPID,
        QuadPIDController *InRollAttitudePID,
        QuadPIDController *InPitchAttitudePID,
        QuadPIDController *InYawAttitudePID,
        QuadPIDController *InxPIDVelocity,
        QuadPIDController *InyPIDVelocity,
        QuadPIDController *InzPIDVelocity,
        QuadPIDController *InrollAttitudePIDVelocity,
        QuadPIDController *InpitchAttitudePIDVelocity,
        QuadPIDController *InyawAttitudePIDVelocity,
        QuadPIDController *InxPIDJoyStick,
        QuadPIDController *InyPIDJoyStick,
        QuadPIDController *InzPIDJoyStick,
        QuadPIDController *InrollAttitudePIDJoyStick,
        QuadPIDController *InpitchAttitudePIDJoyStick,
        QuadPIDController *InyawAttitudePIDJoyStick);
    ~ImGuiUtil();
    void AutoWaypointHud(
        TArray<float> &ThrustsVal,
        float rollError,
        float pitchError,
        const FRotator &currentRotation,
        const FVector &waypoint,
        const FVector &currLoc,
        const FVector &error,
        const FVector &currentVelocity,
        float xOutput,
        float yOutput,
        float zOutput,
        float deltaTime);

    void VelocityHud(
        TArray<float> &ThrustsVal,
        float rollError,
        float pitchError,
        const FRotator &currentRotation,
        const FVector &waypoint,
        const FVector &currLoc,
        const FVector &error,
        const FVector &currentVelocity,
        float xOutput,
        float yOutput,
        float zOutput,
        float deltaTime);

    void JoyStickHud(
        TArray<float> &ThrustsVal,
        float rollError,
        float pitchError,
        const FRotator &currentRotation,
        const FVector &waypoint,
        const FVector &currLoc,
        const FVector &error,
        const FVector &currentVelocity,
        float xOutput,
        float yOutput,
        float zOutput,
        float deltaTime);
    void RenderImPlot(
        const TArray<float> &ThrustsVal,
        float deltaTime);

private:
    void DisplayDroneInfo();
    void DisplayDebugOptions();
    void DisplayThrusterControls(TArray<float> &ThrustsVal);
    void DisplayPIDSettings(
        QuadPIDController *xPIDParam, QuadPIDController *yPIDParam, QuadPIDController *zPIDParam,
        QuadPIDController *rollPIDParam, QuadPIDController *pitchPIDParam, QuadPIDController *yawPIDParam,
        const char *headerLabel, bool &synchronizeXYGains, bool &synchronizeGains);
    void DisplayCameraControls();
    void DisplayResetDroneButtons();
    void DisplayDesiredVelocities();

    // Member variables
    AQuadPawn *dronePawn;
    bool &initialTakeoff;
    bool &altitudeReached;
    bool &Debug_DrawDroneCollisionSphere;
    bool &Debug_DrawDroneWaypoint;
    float maxPIDOutput;
    float altitudeThresh;
    float minAltitudeLocal;
    float &maxVelocity;
    float &maxAngle;
    QuadPIDController *xPID;
    QuadPIDController *yPID;
    QuadPIDController *zPID;
    QuadPIDController *rollAttitudePID;
    QuadPIDController *pitchAttitudePID;
    QuadPIDController *yawAttitudePID;
    QuadPIDController *xPIDVelocity;
    QuadPIDController *yPIDVelocity;
    QuadPIDController *zPIDVelocity;
    QuadPIDController *rollAttitudePIDVelocity;
    QuadPIDController *pitchAttitudePIDVelocity;
    QuadPIDController *yawAttitudePIDVelocity;

    QuadPIDController *xPIDJoyStick;
    QuadPIDController *yPIDJoyStick;
    QuadPIDController *zPIDJoyStick;
    QuadPIDController *rollAttitudePIDJoyStick;
    QuadPIDController *pitchAttitudePIDJoyStick;
    QuadPIDController *yawAttitudePIDJoyStick;

    UQuadDroneController *controller;
    FVector &desiredNewVelocity;
    // Member variables for thrust plotting
    TArray<float> TimeData;
    TArray<float> Thrust0Data;
    TArray<float> Thrust1Data;
    TArray<float> Thrust2Data;
    TArray<float> Thrust3Data;
    float CumulativeTime = 0.0f;
    float MaxPlotTime = 10.0f; // Shows last 10 seconds
    static const int MaxDataPoints = 1000;
};
