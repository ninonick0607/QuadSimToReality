#pragma once

#include "CoreMinimal.h"
#include "Utility/QuadPIDConroller.h"
#include "UI/ImGuiUtil.h"
#include "QuadDroneController.generated.h"

class AQuadPawn; 

UENUM(BlueprintType)
enum class EFlightMode : uint8
{
    None UMETA(DisplayName = "None"),
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
    void ThrustMixer(float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput);
    void YawStabilization(double DeltaTime);

    void ResetPID();
    void ResetDroneIntegral();
    void ResetDroneHigh();
    void ResetDroneOrigin();
    
    void DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint)const;
    void SetDesiredVelocity(const FVector& NewVelocity);
    void SetFlightMode(EFlightMode NewMode);
    EFlightMode GetFlightMode() const;
    FFullPIDSet* GetPIDSet(EFlightMode Mode) { return PIDMap.Find(Mode); }

    bool bManualThrustMode = false;  // defaults to false
    void SetManualThrustMode(bool bEnable);
    void SafetyReset();
    void ApplyManualThrusts();
    void UpdatePropellerRPMs();
private:
    
    UPROPERTY()
    TMap<EFlightMode, FFullPIDSet> PIDMap;

    float desiredYaw;
    float desiredAltitude;
    EFlightMode currentFlightMode;
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
    
    float T_k =.001f;

    double MaxAngularVelocity;
    double YawTorqueForce;
    double LastYawTorqueApplied;
    bool UpsideDown;
    
    FVector initialDronePosition;

};