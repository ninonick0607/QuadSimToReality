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

    // Raw pointers
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
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UI")
    UImGuiUtil* ControllerHUD;
    
    UQuadDroneController(const FObjectInitializer& ObjectInitializer);

    void Initialize(AQuadPawn* InPawn);
    virtual ~UQuadDroneController();
    
    void ResetPID();
    void ResetDroneIntegral();

    void ThrustMixer(float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput, float yawOutput);
    
    void Update(double DeltaTime);

    void VelocityControl(double a_deltaTime);
    
    void DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint)const;
    
    void ResetDroneHigh();
    void ResetDroneOrigin();

    void bufferDebug(FFullPIDSet* PID_Set);
    
    const FVector& GetInitialPosition() const { return initialDronePosition; }
    
    void SetDesiredVelocity(const FVector& NewVelocity);
    void SetFlightMode(EFlightMode NewMode);
    EFlightMode GetFlightMode() const;
    FFullPIDSet* GetPIDSet(EFlightMode Mode)
    {
       return PIDMap.Find(Mode); 
    }


private:
    
    UPROPERTY()
    TMap<EFlightMode, FFullPIDSet> PIDMap;

    float desiredYaw;
    bool bDesiredYawInitialized;
    float desiredAltitude;
    bool bDesiredAltitudeInitialized;

    EFlightMode currentFlightMode;
    

    struct NavPlan
    {
       TArray<FVector> waypoints;
       FString name;
    };

    TArray<NavPlan> setPointNavigation;
    NavPlan* currentNav;
    int32 curPos;
     

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
    float thrustInput;
    float yawInput;
    float pitchInput;
    float rollInput;
    float hoverThrust;
    bool bHoverThrustInitialized;

    FVector initialDronePosition;

};