
#pragma once

#include "CoreMinimal.h"
#include "Utility/QuadPIDConroller.h"
#include "UI/ImGuiUtil.h"
#include "UI/FlightModeHUD.h" 
#include "QuadDroneController.generated.h"


class AQuadPawn; // Forward declaration

UCLASS(Blueprintable, BlueprintType)
class QUADSIMTOREALITY_API UQuadDroneController : public UObject
{
	GENERATED_BODY()

public:
	UQuadDroneController(const FObjectInitializer& ObjectInitializer);

	void Initialize(AQuadPawn* InPawn);

	virtual ~UQuadDroneController();
	
	UPROPERTY()
	AQuadPawn* dronePawn;
	UPROPERTY()
	TArray<float> Thrusts;

	void SetDesiredVelocity(const FVector& NewVelocity);

	void ResetPID();
	void ResetAutoDroneIntegral() const; 
	void ResetVelocityDroneIntegral() const;

	void ThrustMixer(float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput);

	void SetFlightMode(EFlightOptions  NewMode);
	void Update(double DeltaTime);
	void ApplyControllerInput(double a_deltaTime);
	void AutoWaypointControl(double DeltaTime);
	void VelocityControl(double a_deltaTime);
	
	void AddNavPlan(const FString& name, const TArray<FVector>& waypoints);
	void SetNavPlan(const FString& name);
	void DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint)const;
	void HandleThrustInput(float Value);
	void HandleYawInput(float Value);
	void HandlePitchInput(float Value);
	void HandleRollInput(float Value);

	void ResetDroneHigh();
	void ResetDroneOrigin();

	const FVector& GetInitialPosition() const { return initialDronePosition; }

	// Flight data accessors
	UFUNCTION(BlueprintCallable, Category = "Drone|Flight Data")
	FVector GetCurrentAltitude() const;

	UFUNCTION(BlueprintCallable, Category = "Drone|Flight Data")
	FVector GetCurrentVelocity() const;

	UFUNCTION(BlueprintCallable, Category = "Drone|Flight Data")
	float GetMaxVelocity() const { return maxVelocity; }

	UFUNCTION(BlueprintCallable, Category = "Drone|Flight Mode")
	EFlightOptions GetCurrentFlightMode() const { return currentFlightMode; }

	UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
	void SwitchCamera();
	
private:

	

	float desiredYaw;
	bool bDesiredYawInitialized;
	float desiredAltitude;
	bool bDesiredAltitudeInitialized;

	EFlightOptions  currentFlightMode;

	struct NavPlan
	{
		TArray<FVector> waypoints;
		FString name;
	};

	TArray<NavPlan> setPointNavigation;
	NavPlan* currentNav;
	int32 curPos;
     
	TUniquePtr<ImGuiUtil> AutoWaypointHUD;
	TUniquePtr<ImGuiUtil> VelocityHUD;
	TUniquePtr<ImGuiUtil> JoyStickHUD;
	TUniquePtr<ImGuiUtil> ManualThrustHUD;

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

	TUniquePtr<QuadPIDController> xPID;
	TUniquePtr<QuadPIDController> yPID;
	TUniquePtr<QuadPIDController> zPID;
	TUniquePtr<QuadPIDController> rollAttitudePID;
	TUniquePtr<QuadPIDController> pitchAttitudePID;
	TUniquePtr<QuadPIDController> yawAttitudePID;

	TUniquePtr<QuadPIDController> xPIDVelocity;
	TUniquePtr<QuadPIDController> yPIDVelocity;
	TUniquePtr<QuadPIDController> zPIDVelocity;
	TUniquePtr<QuadPIDController> rollAttitudePIDVelocity;
	TUniquePtr<QuadPIDController> pitchAttitudePIDVelocity;
	TUniquePtr<QuadPIDController> yawAttitudePIDVelocity;

	TUniquePtr<QuadPIDController> xPIDJoyStick;
	TUniquePtr<QuadPIDController> yPIDJoyStick;
	TUniquePtr<QuadPIDController> zPIDJoyStick;
	TUniquePtr<QuadPIDController> rollAttitudePIDJoyStick;
	TUniquePtr<QuadPIDController> pitchAttitudePIDJoyStick;
	TUniquePtr<QuadPIDController> yawAttitudePIDJoyStick;

	FVector initialDronePosition;

};
