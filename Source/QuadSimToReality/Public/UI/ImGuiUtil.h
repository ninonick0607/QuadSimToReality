// ImGuiUtil.h
#pragma once
#include "CoreMinimal.h"
#include "Controllers/ZMQController.h"

// Forward declarations
class AQuadPawn;
class QuadPIDController;
class UQuadDroneController;

enum class EFlightMode : uint8;  // Forward declare the enum

class QUADSIMTOREALITY_API ImGuiUtil
{
public:
    ImGuiUtil(
        AQuadPawn* InPawn,
        UQuadDroneController* InController,
        FVector& IndesiredNewVelocity,
        bool& InDebug_DrawDroneCollisionSphere,
        bool& InDebug_DrawDroneWaypoint,
        float InMaxPIDOutput,
        float& InMaxVelocity,
        float& InMaxAngle);
    ~ImGuiUtil();

    void AutoWaypointHud(
        TArray<float>& ThrustsVal,
        float rollError, float pitchError,
        const FRotator& currentRotation,
        const FVector& waypoint, const FVector& currLoc,
        const FVector& error,
        const FVector& currentVelocity,
        float xOutput, float yOutput, float zOutput, float deltaTime);

    void VelocityHud(
        TArray<float>& ThrustsVal,
        float rollError, float pitchError,
        const FRotator& currentRotation,
        const FVector& waypoint, const FVector& currLoc,
        const FVector& error,
        const FVector& currentVelocity,
        float xOutput, float yOutput, float zOutput, float deltaTime);

    void JoyStickHud(
        TArray<float>& ThrustsVal,
        float rollError, float pitchError,
        const FRotator& currentRotation,
        const FVector& waypoint, const FVector& currLoc,
        const FVector& error,
        const FVector& currentVelocity,
        float xOutput, float yOutput, float zOutput, float deltaTime);

    void RenderImPlot(const TArray<float>& ThrustsVal, float deltaTime);

private:
    void DisplayDroneInfo();
    void DisplayDebugOptions();
    void DisplayPIDSettings(
        EFlightMode Mode,
        const char* headerLabel,
        bool& synchronizeXYGains,
        bool& synchronizeGains);
    void DisplayCameraControls();
    void DisplayResetDroneButtons();
    void DisplayDesiredVelocities();
    
    AQuadPawn* dronePawn;
    UQuadDroneController* controller;

    // Member variables
    bool& Debug_DrawDroneCollisionSphere;
    bool& Debug_DrawDroneWaypoint;
    float maxPIDOutput;
    float& maxVelocity;
    float& maxAngle;
    FVector& desiredNewVelocity;
    
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