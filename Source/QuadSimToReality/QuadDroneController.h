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
	
	void ThrustMixer(float xOutput,float yOutput,float zOutput,float rollOutput,float pitchOutput);
	void Update(double DeltaTime);
	void AddNavPlan(FString Name, TArray<FVector> Waypoints);
	void SetNavPlan(FString Name);
	void Reset();
	void DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint)const;

	bool hoverMode = false;
	bool IsNavigationComplete() const;
	// Moved from global variables to member variables
	float maxVelocity = 200.0f;   // The maximum desired velocity of each axis
	float maxAngle = 30.f;         // The maximum tilt angle of the drone
	
	TArray<float> Thrusts;

	void RenderImGui(
	 TArray<float>& ThrustsVal,
	 float rollError, float pitchError,
	 const FRotator& currentRotation,
	 const FVector& waypoint, const FVector& currLoc,
	 const FVector& error, const FVector& desiredVelocity,
	 const FVector& currentVelocity,
	 float xOutput, float yOutput, float zOutput);

private:
	bool Debug_DrawDroneCollisionSphere = true;
	bool Debug_DrawDroneWaypoint = true;
	
	struct NavPlan
	{
		TArray<FVector> waypoints; // Series of discrete points to follow
		FString name;
	};
	

	AQuadPawn* dronePawn;
	TArray<NavPlan> setPointNavigation;
	NavPlan* currentNav;
	int32 curPos; // Current position in the nav plan

	QuadPIDController* xPID;
	QuadPIDController* yPID;
	QuadPIDController* zPID;
	QuadPIDController* rollAttitudePID;
	QuadPIDController* pitchAttitudePID;

	bool altitudeReached = false;
	static inline const float maxPIDOutput = 600.f; // Max thrusting power individual rotor can provide
	bool initialTakeoff = true;

	// Constants
	static constexpr float altitudeThresh = 0.6f; // If the relative z is more than this, we set roll and pitch to 0
	static constexpr float minAltitudeLocal = 500.f;      // If the altitude is less than this in the beginning, the drone must ascend to this altitude
};
