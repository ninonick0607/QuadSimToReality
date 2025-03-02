// ImGuiUtil.h
#pragma once
#include "CoreMinimal.h"
#include "Controllers/ZMQController.h"
#include "ImGuiUtil.generated.h"
// Forward declarations
class AQuadPawn;
class QuadPIDController;
class UQuadDroneController;

enum class EFlightMode : uint8;  // Forward declare the enum

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UImGuiUtil : public UActorComponent
{
    GENERATED_BODY()
public:

    UImGuiUtil();

    /** Call this once the owning controller and pawn are valid */
    void Initialize(AQuadPawn* InPawn, UQuadDroneController* InController, const FVector& InDesiredNewVelocity,
                    bool InDebug_DrawDroneCollisionSphere, bool InDebug_DrawDroneWaypoint,
                    float InMaxPIDOutput, float InMaxVelocity, float InMaxAngle);

    /** Main functions to draw the UI */
    void VelocityHud(TArray<float>& ThrustsVal,
                     float rollError, float pitchError,
                     const FRotator& currentRotation,
                     const FVector& waypoint, const FVector& currLoc,
                     const FVector& error,
                     const FVector& currentVelocity,
                     float xOutput, float yOutput, float zOutput, float deltaTime);

    void RenderImPlot(const TArray<float>& ThrustsVal, float deltaTime);

    void DisplayDroneInfo();
    void DisplayDebugOptions();
    void DisplayPIDSettings(EFlightMode Mode, const char* headerLabel, bool& synchronizeXYGains, bool& synchronizeGains);
    void DisplayCameraControls();
    void DisplayResetDroneButtons();
    void DisplayDesiredVelocities();
    void DisplayPIDHistoryWindow();

protected:
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
 

    UPROPERTY()
    AQuadPawn* DronePawn;

    UPROPERTY()
    UQuadDroneController* Controller;

    // Parameters (stored by value now)
    bool Debug_DrawDroneCollisionSphere;
    bool Debug_DrawDroneWaypoint;
    float maxPIDOutput;
    float maxVelocity;
    float maxAngle;
    FVector desiredNewVelocity;

    // Data for plotting
    TArray<float> TimeData;
    TArray<float> Thrust0Data;
    TArray<float> Thrust1Data;
    TArray<float> Thrust2Data;
    TArray<float> Thrust3Data;
    float CumulativeTime;
    float MaxPlotTime;
    

    // Helper method to load PID values from a CSV row
    void LoadPIDValues(const TArray<FString>& Values);

    static const int32 MaxDataPoints = 500;
};