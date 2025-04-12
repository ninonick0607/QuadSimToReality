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
    
    void DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint)const;
    //void DrawDebugVisuals(const FVector& horizontalVelocity) const;
    void SetDesiredVelocity(const FVector& NewVelocity);
    float GetDesiredYaw() const { return desiredYaw; }
    FVector GetDesiredVelocity() const { return desiredNewVelocity; }

    bool bManualThrustMode = false;
    void SetManualThrustMode(bool bEnable);
    void SafetyReset();
    void ApplyManualThrusts();

    bool IsHoverModeActive() const { return bHoverModeActive; }
    void SetHoverMode(bool bActive, float TargetAltitude = 250.0f);

    bool GetDebugVisualsEnabled() const { return bDebugVisualsEnabled; }
    void SetDebugVisualsEnabled(bool bEnabled) { bDebugVisualsEnabled = bEnabled; }
    void SetDesiredYawRate(float NewYawRate) { desiredYawRate = NewYawRate; }
    float GetDesiredYawRate() const { return desiredYawRate; }
    FVector GetCurrentLocalVelocity() const { return currentLocalVelocity; }
    void SetDesiredAngle(float newAngle) { maxAngle = newAngle; }
    void AddNavPlan(const FString& name, const TArray<FVector>& waypoints);
    void SetNavPlan(const FString& name);
    void SetFlightMode(EFlightMode NewMode);
    EFlightMode GetFlightMode() const;
    FFullPIDSet* GetPIDSet(EFlightMode Mode)
    {
        return PIDMap.Find(Mode); 
    }
    
private:

    UPROPERTY()
    TMap<EFlightMode, FFullPIDSet> PIDMap;


    EFlightMode currentFlightMode;
    
    struct NavPlan
    {
        TArray<FVector> waypoints;
        FString name;
    };

    TArray<NavPlan> setPointNavigation;
    NavPlan* currentNav;
    int32 curPos;
     
    // TUniquePtr<ImGuiUtil> AutoWaypointHUD;
    // TUniquePtr<ImGuiUtil> VelocityHUD;
    // TUniquePtr<ImGuiUtil>JoyStickHUD;
    
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

    bool Debug_DrawDroneCollisionSphere;
    bool Debug_DrawDroneWaypoint;
    float thrustInput;
    float yawInput;
    float pitchInput;
    float rollInput;
    bool bHoverThrustInitialized;
    
    
};