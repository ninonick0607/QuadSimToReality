// QuadDroneController.h

#pragma once

#include "CoreMinimal.h"
#include "QuadPIDConroller.h"

class AQuadPawn; // Forward declaration


class QUADSIMTOREALITY_API QuadDroneController
{
public:
	QuadDroneController(AQuadPawn* InPawn);
	~QuadDroneController();
	
	void Reset();
	void Update(double DeltaTime);
	void AutoWaypointControl(double DeltaTime);
	//void ManualWaypointControl(double DeltaTime);
	void ThrustMixer(float xOutput,float yOutput,float zOutput,float rollOutput,float pitchOutput);
	void MoveByVelocity(float vx, float vy, float vz);
	void VelocityControl(double a_deltaTime);
	void ManualThrustControl(double a_deltaTime);
	void AddNavPlan(FString Name, TArray<FVector> Waypoints);
	void SetNavPlan(FString Name);
	void DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint)const;
	void RenderImGuiWaypoint(TArray<float>& ThrustsVal,float rollError, float pitchError,const FRotator& currentRotation,const FVector& waypoint, const FVector& currLoc,const FVector& error, const FVector& desiredVelocity,const FVector& currentVelocity,float xOutput, float yOutput, float zOutput,float deltaTime);
	void RenderImGuiVelocity(TArray<float>& ThrustsVal, float rollError, float pitchError, const FRotator& currentRotation, const FVector& waypoint, const FVector& currLoc, const FVector& error, const FVector& desiredVelocity, const FVector& currentVelocity, float xOutput, float yOutput, float zOutput, float deltaTime);
	void RenderImGuiJoyStick(TArray<float>& ThrustsVal,float rollError, float pitchError,const FRotator& currentRotation,const FVector& waypoint, const FVector& currLoc,const FVector& error, const FVector& desiredVelocity,const FVector& currentVelocity,float xOutput, float yOutput, float zOutput,float deltaTime);
	void RenderImPlot(
		TArray<float>& ThrustsVal,
		float rollError, float pitchError,
		const FRotator& currentRotation,
		const FVector& waypoint, const FVector& currLoc,
		const FVector& error, const FVector& desiredVelocity,
		const FVector& currentVelocity,
		float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput, float yawOutput, float deltaTime);
	void IncreaseAllThrusts(float Amount);

	TArray<float> Thrusts;
	float maxVelocity = 250.0f;   // The maximum desired velocity of each axis
	float maxAngle = 15.f;         // The maximum tilt angle of the drone
	float totalElapsedTime;
	FVector desiredNewVelocity = FVector(0, 0, 0);

	float thrustInput = 0.0f;
	float yawInput = 0.0f;
	float pitchInput = 0.0f;
	float rollInput = 0.0f;

	void HandleThrustInput(float Value);
	void HandleYawInput(float Value);
	void HandlePitchInput(float Value);
	void HandleRollInput(float Value);
    
	void ApplyControllerInput(double a_deltaTime);

	
private:
	float desiredYaw = 0.0f;
	bool bDesiredYawInitialized = false;
	float hoverThrustLevel;
	float desiredAltitude = 0.0f;
	bool bDesiredAltitudeInitialized = false; // Flag to initialize desiredAltitude once
	const int rotorSpinDirections[4] = { -1, +1, +1, -1 };
	const float rotorTorqueConstant = 0.01f;
	struct NavPlan
	{
		TArray<FVector> waypoints; // Series of discrete points to follow
		FString name;
	};
	
	float hoverThrust;
	bool bHoverThrustInitialized = false;
	AQuadPawn* dronePawn;
	TArray<NavPlan> setPointNavigation;
	NavPlan* currentNav;
	int32 curPos; // Current position in the nav plan

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

	bool initialTakeoff = true;
	bool altitudeReached = false;
	bool Debug_DrawDroneCollisionSphere = true;
	bool Debug_DrawDroneWaypoint = true;
	static inline const float maxPIDOutput = 600.f; // Max thrusting power individual rotor can provide
	static constexpr float altitudeThresh = 0.6f; // If the relative z is more than this, we set roll and pitch to 0
	static constexpr float minAltitudeLocal = 500.f;  // If the altitude is less than this in the beginning, the drone must ascend to this altitude

protected:
	TArray<float> TimeData;
	TArray<float> waypointArrayX;
	TArray<float> waypointArrayY;
	TArray<float> waypointArrayZ;
	TArray<float> currentPosArrayX;
	TArray<float> currentPosArrayY;
	TArray<float> currentPosArrayZ;
	TArray<TArray<float>> thrustValues;

	TArray<float> xPIDOutputHistory;
	TArray<float> yPIDOutputHistory;
	TArray<float> zPIDOutputHistory;
	TArray<float> rollPIDOutputHistory;
	TArray<float> pitchPIDOutputHistory;
	TArray<float> yawPIDOutputHistory;
	TArray<float> positionErrorHistory;
	TArray<float> velocityErrorHistory;
	float CumulativeTime = 0.0f;
};
