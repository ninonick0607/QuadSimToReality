// QuadDroneController.h

#pragma once

#include "CoreMinimal.h"
#include "QuadPIDConroller.h"
#include "ImGuiUtil.h"

class AQuadPawn; // Forward declaration


class QUADSIMTOREALITY_API QuadDroneController
{
	friend class ImGuiUtil;

public:

	enum class FlightMode
	{
		None,
		AutoWaypoint,
		ManualWaypoint,
		ManualThrustControl,
		ManualFlightControl,
		VelocityControl
	};


	QuadDroneController(AQuadPawn* InPawn);
	~QuadDroneController();

	void SetDesiredVelocity(const FVector& NewVelocity);
	void SetFlightMode(FlightMode NewMode);
	FlightMode GetFlightMode() const;
	void Reset();
	void Update(double DeltaTime);
	void AutoWaypointControl(double DeltaTime);
	void ThrustMixer(float xOutput,float yOutput,float zOutput,float rollOutput,float pitchOutput);
	void VelocityControl(double a_deltaTime);
	void ManualThrustControl(double a_deltaTime);
	void AddNavPlan(FString Name, TArray<FVector> Waypoints);
	void SetNavPlan(FString Name);
	void DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint)const;
	void IncreaseAllThrusts(float Amount);
	void HandleThrustInput(float Value);
	void HandleYawInput(float Value);
	void HandlePitchInput(float Value);
	void HandleRollInput(float Value);
	void ApplyControllerInput(double a_deltaTime);

	bool CheckForCrashOrUnstableCondition();
	void StartAutoTune();            
	void StopAutoTune();           

private:

	float desiredYaw;
	bool bDesiredYawInitialized;
	float desiredAltitude;
	bool bDesiredAltitudeInitialized;

	AQuadPawn* dronePawn;
	TArray<float> Thrusts;
	FlightMode currentFlightMode;

	struct NavPlan
	{
		TArray<FVector> waypoints;
		FString name;
	};

	TArray<NavPlan> setPointNavigation;
	NavPlan* currentNav;
	int32 curPos;

	ImGuiUtil* AutoWaypointHUD;
	ImGuiUtil* VelocityHUD;
	ImGuiUtil* JoyStickHUD;
	ImGuiUtil* ManualThrustHUD;

     
	float totalElapsedTime;
	FVector desiredNewVelocity = FVector(0, 0, 0);

	float maxVelocity;
	float maxAngle;
	bool initialTakeoff;
	bool altitudeReached;
	bool Debug_DrawDroneCollisionSphere;
	bool Debug_DrawDroneWaypoint;
	static inline const float maxPIDOutput=600.f;
	static constexpr float altitudeThresh=0.6f;
	static constexpr float minAltitudeLocal=500.f;
	float thrustInput;
	float yawInput;
	float pitchInput;
	float rollInput;
	float hoverThrust;
	bool bHoverThrustInitialized;

	QuadPIDController* xPID;
	QuadPIDController* yPID;
	QuadPIDController* zPID;
	QuadPIDController* rollAttitudePID;
	QuadPIDController* pitchAttitudePID;
	QuadPIDController* yawAttitudePID;

	QuadPIDController* xPIDVelocity;
	QuadPIDController* yPIDVelocity;
	QuadPIDController* zPIDVelocity;
	QuadPIDController* rollAttitudePIDVelocity;
	QuadPIDController* pitchAttitudePIDVelocity;
	QuadPIDController* yawAttitudePIDVelocity;

	QuadPIDController* xPIDJoyStick;
	QuadPIDController* yPIDJoyStick;
	QuadPIDController* zPIDJoyStick;
	QuadPIDController* rollAttitudePIDJoyStick;
	QuadPIDController* pitchAttitudePIDJoyStick;
	QuadPIDController* yawAttitudePIDJoyStick;


	//Auto tuner testing
	bool bAutoTune;
	bool bAutoTuneInProgress;
	float autoTuneIterationTime;
	float autoTuneElapsedTime;
	int currentPIDParameterIndex;

	// Ziegler–Nichols tuning variables
	bool bZNTuningInProgress;
	float kpInitial;
	float kpCurrent;
	float ultimateGain;
	float ultimatePeriod;
	float znTestDuration;
	float znObservationTime;
	int znOscillationCount;
	bool bOscillating;
	float znPrevError;
	float lastSetTime;

	void PerformZieglerNicholsTuning(double deltaTime);
};
