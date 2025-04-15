#pragma once

#include "CoreMinimal.h"
#include "Utility/QuadPIDController.h"
#include "UI/ImGuiUtil.h"
#include "Msgs/ROS2Quat.h"
#include "QuadDroneController.generated.h"

class AQuadPawn;

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
    void ThrustMixer(double currentRoll, double currentPitch, double zOutput, double rollOutput, double pitchOutput);
    void YawStabilization(double DeltaTime);
    void YawRateControl(double DeltaTime);
    void ResetPID();
    void ResetDroneIntegral();
    void ResetDroneHigh();
    void ResetDroneOrigin();

    void DrawDebugVisuals(const FVector& horizontalVelocity) const;
    void SetManualThrustMode(bool bEnable);
    void SafetyReset();
    void ApplyManualThrusts();

    bool IsHoverModeActive() const { return bHoverModeActive; }
    void SetHoverMode(bool bActive, float TargetAltitude);

    bool GetDebugVisualsEnabled() const { return bDebugVisualsEnabled; }
    void SetDebugVisualsEnabled(bool bEnabled) { bDebugVisualsEnabled = bEnabled; }
    void SetDesiredYawRate(float NewYawRate) { desiredYawRate = NewYawRate; }
    float GetDesiredYawRate() const { return desiredYawRate; }
    void SetDesiredRoll(float NewRoll) { desiredRoll = NewRoll; }
    void SetDesiredPitch(float NewPitch) { desiredPitch = NewPitch; }
    void SetDesiredAngle(float newAngle) { maxAngle = newAngle; }
    FFullPIDSet* GetPIDSet() { return PIDMap.Num() > 0 ? &PIDMap[0] : nullptr; }
    float GetDesiredYaw() const { return desiredYaw; }
    FVector GetDesiredVelocity() const { return desiredNewVelocity; }
    float GetCurrentThrustOutput(int32 ThrusterIndex) const;
    void SetDesiredVelocity(const FVector& NewVelocity);
    FVector GetCurrentLocalVelocity() const { return currentLocalVelocity; }
    UFUNCTION(BlueprintPure, Category = "Drone State")
    FVector GetCurrentVelocity() const; // Make sure this is implemented to return world velocity

    UFUNCTION(BlueprintPure, Category = "Drone State|ROS")
    FQuat GetOrientationAsQuat() const;

    UFUNCTION(BlueprintPure, Category = "Drone State|ROS")
    FVector GetCurrentAngularVelocityRADPS() const;
    
private:

    UPROPERTY()
    TArray<FFullPIDSet> PIDMap;

    float desiredYaw;
    float desiredAltitude;
    FVector currentLocalVelocity;
    FVector desiredNewVelocity;

    float maxVelocity;
    float maxAngle;
    float maxPIDOutput;
    float altitudeThresh;
    float minAltitudeLocal;
    float acceptableDistance;

    bool initialTakeoff;
    bool altitudeReached;
    bool bDebugVisualsEnabled = false;
    
    double MaxAngularVelocity;
    double YawTorqueForce;
    double LastYawTorqueApplied;
    bool UpsideDown;
    FVector desiredForwardVector;
    FVector initialDronePosition;

    QuadPIDController* AltitudePID;
    bool bHoverModeActive;
    float hoverTargetAltitude;
    
    float desiredYawRate;
    float desiredRoll;
    float desiredPitch;

    bool bManualThrustMode = false;

};