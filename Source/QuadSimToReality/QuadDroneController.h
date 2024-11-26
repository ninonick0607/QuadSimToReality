// QuadDroneController.h

#pragma once

#include "CoreMinimal.h"
#include "QuadPIDConroller.h"
#include "ImGuiUtil.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "QuadDroneController.generated.h"

class AQuadPawn; // Forward declaration
namespace zmq {
	class context_t;
	class socket_t;
	class message_t;
	class multipart_t;
	enum class socket_type : int;
	enum class recv_flags : int;
	enum class send_flags : int;
	class error_t;
}
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
		ManualWaypoint,
		ManualThrustControl,
		ManualFlightControl,
		VelocityControl
	};


	UPROPERTY()
	AQuadPawn* dronePawn;
	UPROPERTY()
	TArray<float> Thrusts;

	void BeginZMQController();


	void SetDesiredVelocity(const FVector& NewVelocity);
	void SetFlightMode(FlightMode NewMode);
	FlightMode GetFlightMode() const;

	void ResetPID();

	void ThrustMixer(float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput);
	FVector CalculateDesiredVelocity(const FVector& error, float InMaxVelocity);
	float CalculateDesiredRoll(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold);
	float CalculateDesiredPitch(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold);

	void Update(double DeltaTime);
	void AutoWaypointControl(double DeltaTime);
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

	void StoreInitialPosition();
	void ResetDroneHigh();
	void ResetDroneOrigin();

	const FVector& GetInitialPosition() const { return initialDronePosition; }
private:

	void CaptureAndSendImage();
	void SendImageData(const TArray<uint8>& CompressedBitmap);
	void ReceiveVelocityCommand();

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

	//ZeroMQ Variables
	static zmq::context_t* SharedZMQContext;
	static zmq::socket_t* SharedZMQSocket;

	zmq::socket_t* ZMQSocket;
	zmq::socket_t* CommandSocket;


	UPROPERTY()
	USceneCaptureComponent2D* SceneCaptureComponent;

	UPROPERTY()
	UTextureRenderTarget2D* RenderTarget;

	float UpdateInterval;
	float TimeSinceLastUpdate;
	static int32 SharedResourceRefCount;
	bool bIsActive;
	FString DroneID;
	FVector initialDronePosition;

};
