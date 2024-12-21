
#pragma once

#include "CoreMinimal.h"
#include "Utility/QuadPIDConroller.h"
#include "UI/ImGuiUtil.h"
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

	enum class FlightMode
	{
		None,
		AutoWaypoint,
		JoyStickControl,
		VelocityControl
	};


	UPROPERTY()
	AQuadPawn* dronePawn;
	UPROPERTY()
	TArray<float> Thrusts;

	void SetDesiredVelocity(const FVector& NewVelocity);
	void SetFlightMode(FlightMode NewMode);
	FlightMode GetFlightMode() const;

	void ResetPID();
	void ResetAutoDroneIntegral() const; 
	void ResetVelocityDroneIntegral() const;

	void ThrustMixer(float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput);
	static FVector CalculateDesiredVelocity(const FVector& error, float InMaxVelocity);
	static float CalculateDesiredRoll(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold);
	static float CalculateDesiredPitch(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold);

	void Update(double DeltaTime);
	void AutoWaypointControl(double DeltaTime);
	void VelocityControl(double a_deltaTime);
	void AddNavPlan(const FString& name, const TArray<FVector>& waypoints);
	void SetNavPlan(const FString& name);
	void DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint)const;
	void HandleThrustInput(float Value);
	void HandleYawInput(float Value);
	void HandlePitchInput(float Value);
	void HandleRollInput(float Value);
	void ApplyControllerInput(double a_deltaTime);

	void ResetDroneHigh();
	void ResetDroneOrigin();

	const FVector& GetInitialPosition() const { return initialDronePosition; }
private:


	float desiredYaw;
	bool bDesiredYawInitialized;
	float desiredAltitude;
	bool bDesiredAltitudeInitialized;

	FlightMode currentFlightMode;

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
	TUniquePtr<ImGuiUtil>JoyStickHUD;
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
