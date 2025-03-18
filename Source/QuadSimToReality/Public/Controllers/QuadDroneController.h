#pragma once

#include "CoreMinimal.h"
#include "Utility/QuadPIDConroller.h"
#include "UI/ImGuiUtil.h"
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
    {}
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
    void ThrustMixer(double xOutput, double yOutput, double zOutput, double rollOutput, double pitchOutput);
    void YawStabilization(double DeltaTime);

    void ResetPID();
    void ResetDroneIntegral();
    void ResetDroneHigh();
    void ResetDroneOrigin();
    
    void DrawDebugVisuals(const FVector& horizontalVelocity) const;
    void SetDesiredVelocity(const FVector& NewVelocity);
    FFullPIDSet* GetPIDSet() { return PIDMap.Num() > 0 ? &PIDMap[0] : nullptr; }
    float GetDesiredYaw() const { return desiredYaw; }
    FVector GetDesiredVelocity() const { return desiredNewVelocity; }

    bool bManualThrustMode = false;  // defaults to false
    void SetManualThrustMode(bool bEnable);
    void SafetyReset();
    void ApplyManualThrusts();
private:
    
    UPROPERTY()
    TArray<FFullPIDSet> PIDMap; 

    float desiredYaw;
    float desiredAltitude;
    FVector desiredNewVelocity;

    float maxVelocity;
    float maxAngle;
    float maxPIDOutput;
    float altitudeThresh; 
    float minAltitudeLocal;
    float acceptableDistance;
    
    bool initialTakeoff;
    bool altitudeReached;
    bool Debug_DrawDroneCollisionSphere;
    bool Debug_DrawDroneWaypoint;
    
    double MaxAngularVelocity;
    double YawTorqueForce;
    double LastYawTorqueApplied;
    bool UpsideDown;
    FVector desiredForwardVector;
    FVector initialDronePosition;

    bool bHoverModeActive = false;
    float hoverTargetAltitude = 0.0f;
    
};