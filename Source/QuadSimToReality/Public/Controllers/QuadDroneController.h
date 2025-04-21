#pragma once

#include "CoreMinimal.h"
#include "Utility/QuadPIDController.h"
#include "UI/ImGuiUtil.h"
#include "Msgs/ROS2Quat.h"
#include "QuadDroneController.generated.h"

class AQuadPawn;

UENUM(BlueprintType)
enum class EFlightMode : uint8
{
    None UMETA(DisplayName = "None"),
    AutoWaypoint UMETA(DisplayName = "AutoWaypoint"),
    JoyStickControl UMETA(DisplayName = "JoyStickControl"),
    VelocityControl UMETA(DisplayName = "VelocityControl")
};

USTRUCT()
struct FFullPIDSet
{
    GENERATED_BODY()

    QuadPIDController* XPID;
    QuadPIDController* YPID;
    QuadPIDController* ZPID;
    QuadPIDController* RollPID;
    QuadPIDController* PitchPID;
    QuadPIDController* YawPID;

    FFullPIDSet()
        : XPID(nullptr)
        , YPID(nullptr)
        , ZPID(nullptr)
        , RollPID(nullptr)
        , PitchPID(nullptr)
        , YawPID(nullptr)
    {
    }
};


UCLASS(Blueprintable, BlueprintType)
class QUADSIMTOREALITY_API UQuadDroneController : public UObject
{
    GENERATED_BODY()

public:

    UPROPERTY()
    AQuadPawn* dronePawn;
    UPROPERTY()
    TArray<float> Thrusts;

    UQuadDroneController(const FObjectInitializer& ObjectInitializer);
    virtual ~UQuadDroneController();

    void Initialize(AQuadPawn* InPawn);
    void Update(double DeltaTime);

    void VelocityControl(double a_deltaTime);
    //void ApplyControllerInput(double a_deltaTime);
    void AutoWaypointControl(double DeltaTime);
    void ThrustMixer(double currentRoll, double currentPitch, double zOutput, double rollOutput, double pitchOutput);
    void YawStabilization(double DeltaTime);
    void YawRateControl(double DeltaTime);
    
    void ResetPID();
    void ResetDroneIntegral();
    void ResetDroneHigh();
    void ResetDroneOrigin();

    void SetDesiredVelocity(const FVector& NewVelocity);
    void SetManualThrustMode(bool bEnable);
    void SetHoverMode(bool bActive, float TargetAltitude = 250.0f);
    void SetDestination(FVector desiredSetPoints);

    void DrawDebugVisuals(const FVector& currentPosition)const;
    void DrawDebugVisualsVel(const FVector& horizontalVelocity) const;
    void SafetyReset();
    void ApplyManualThrusts();
 
    float GetDesiredYaw() const { return desiredYaw; }
    FVector GetDesiredVelocity() const { return desiredNewVelocity; }
    bool GetDebugVisualsEnabled() const { return bDebugVisualsEnabled; }
    FVector GetCurrentLocalVelocity() const { return currentLocalVelocity; }
    float GetDesiredYawRate() const { return desiredYawRate; }
    FVector GetCurrentSetPoint() const { return setPoint; }

    void SetDebugVisualsEnabled(bool bEnabled) { bDebugVisualsEnabled = bEnabled; }
    void SetDesiredYawRate(float NewYawRate) { desiredYawRate = NewYawRate; }
    void SetDesiredAngle(float newAngle) { maxAngle = newAngle; }
    void SetMaxVelocity(float newMaxVelocity) { maxVelocity = newMaxVelocity;}
    void SetMaxAngle(float newMaxAngle) { maxAngle = newMaxAngle;}
    bool IsHoverModeActive() const { return bHoverModeActive; }

    float GetCurrentThrustOutput(int32 ThrusterIndex) const;
    UFUNCTION(BlueprintPure, Category = "Drone State")
    FVector GetCurrentVelocity() const; 
    UFUNCTION(BlueprintPure, Category = "Drone State|ROS")
    FQuat GetOrientationAsQuat() const;
    UFUNCTION(BlueprintPure, Category = "Drone State|ROS")
    FVector GetCurrentAngularVelocityRADPS() const;
    
    UFUNCTION(BlueprintCallable, Category = "Flight")
    void SetFlightMode(EFlightMode NewMode);
    FFullPIDSet* GetPIDSet(EFlightMode Mode)
    {
        return PIDMap.Find(Mode); 
    }
    
private:

    UPROPERTY()
    TMap<EFlightMode, FFullPIDSet> PIDMap;
    QuadPIDController* AltitudePID;
    EFlightMode currentFlightMode;

    //Global Drone Variables
    float desiredYaw;
    FVector currentLocalVelocity;
    float maxVelocity;
    float maxAngle;
    float maxPIDOutput;
    FVector desiredForwardVector;
    double YawTorqueForce;
    double LastYawTorqueApplied; // Maybe not needed, why global??
    float desiredYawRate;
    bool bDebugVisualsEnabled = false;

    // AutoWaypointControl variables
    FVector setPoint;
    float minAltitudeLocal;
    float acceptableDistance;

    // VelocityControl
    FVector desiredNewVelocity;

    // Hover Mode
    float hoverTargetAltitude;
    bool bHoverModeActive;

    // Debug
    bool Debug_DrawDroneCollisionSphere;
    bool Debug_DrawDroneWaypoint;
    
    bool bManualThrustMode = false;

};