// QuadDroneController.cpp

#include "Controllers/QuadDroneController.h"
#include "Pawns/QuadPawn.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"
#include "Core/DroneGlobalState.h"
#include "UI/ImGuiUtil.h"
#include "Core/DroneJSONConfig.h"
#include "Core/DroneMathUtils.h"
#include "Math/UnrealMathUtility.h"


// ---------------------- Constructor ------------------------

UQuadDroneController::UQuadDroneController(const FObjectInitializer& ObjectInitializer)
	: dronePawn(nullptr)
	  , Thrusts({0, 0, 0, 0})
	  , desiredYaw(0.f)
	  , bDesiredYawInitialized(false)
	  , desiredAltitude(0.0f)
	  , bDesiredAltitudeInitialized(false)
      , currentFlightMode(EFlightOptions::AutoWaypoint)  // Add initial mode
	  , currentNav(nullptr)
	  , curPos(0)
	  , AutoWaypointHUD(nullptr)
	  , VelocityHUD(nullptr)
	  , JoyStickHUD(nullptr)
	  , ManualThrustHUD(nullptr)
	  , desiredNewVelocity(FVector::ZeroVector)
{
	// Load config values
	const auto& Config = UDroneJSONConfig::Get().Config;
	maxVelocity = Config.FlightParams.MaxVelocity;
	maxAngle = Config.FlightParams.MaxAngle;
	maxPIDOutput = Config.FlightParams.MaxPIDOutput;
	altitudeThresh = Config.FlightParams.AltitudeThreshold;
	minAltitudeLocal = Config.FlightParams.MinAltitudeLocal;
	acceptableDistance = Config.FlightParams.AcceptableDistance;

	// Initialize other values
	initialTakeoff = true;
	altitudeReached = false;
	Debug_DrawDroneCollisionSphere = true;
	Debug_DrawDroneWaypoint = true;
	thrustInput = 0.0f;
	yawInput = 0.0f;
	pitchInput = 0.0f;
	rollInput = 0.0f;
	hoverThrust = 0.0f;
	bHoverThrustInitialized = false;
	Thrusts.SetNum(4);
	// PID stuff
	xPID = MakeUnique<QuadPIDController>();
	xPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	xPID->SetGains(1.f, 0.f, 0.1f);

	yPID = MakeUnique<QuadPIDController>();
	yPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	yPID->SetGains(1.f, 0.f, 0.1f);

	zPID = MakeUnique<QuadPIDController>();
	zPID->SetLimits(-maxPIDOutput, maxPIDOutput);
	zPID->SetGains(5.f, 1.f, 0.1f);

	pitchAttitudePID = MakeUnique<QuadPIDController>();
	pitchAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
	pitchAttitudePID->SetGains(2.934f, 0.297f, 3.633f);

	rollAttitudePID = MakeUnique<QuadPIDController>();
	rollAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
	rollAttitudePID->SetGains(2.934f, 0.297f, 3.633f);

	yawAttitudePID = MakeUnique<QuadPIDController>();
	yawAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
	yawAttitudePID->SetGains(0.f, 0.f, 0.f);

	// Move by Velocity
	xPIDVelocity = MakeUnique<QuadPIDController>();
	xPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
	xPIDVelocity->SetGains(1.f, 0.f, 0.1f);

	yPIDVelocity = MakeUnique<QuadPIDController>();
	yPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
	yPIDVelocity->SetGains(1.f, 0.f, 0.1f);

	zPIDVelocity = MakeUnique<QuadPIDController>();
	zPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
	zPIDVelocity->SetGains(5.f, 1.f, 0.1f);

	pitchAttitudePIDVelocity = MakeUnique<QuadPIDController>();
	pitchAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
	pitchAttitudePIDVelocity->SetGains(8.f, 0.3f, 3.7);

	rollAttitudePIDVelocity = MakeUnique<QuadPIDController>();
	rollAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
	rollAttitudePIDVelocity->SetGains(8.f, 0.3f, 3.7);

	yawAttitudePIDVelocity = MakeUnique<QuadPIDController>();
	yawAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
	yawAttitudePIDVelocity->SetGains(1.8f, 0.15f, 1.5f);

	// Move by Controller
	xPIDJoyStick = MakeUnique<QuadPIDController>();
	xPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
	xPIDJoyStick->SetGains(2.329f, 3.626f, 1.832f);

	yPIDJoyStick = MakeUnique<QuadPIDController>();
	yPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
	yPIDJoyStick->SetGains(2.329f, 3.626f, 1.832f);

	zPIDJoyStick = MakeUnique<QuadPIDController>();
	zPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
	zPIDJoyStick->SetGains(5.344f, 1.f, 0.1f);

	pitchAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
	pitchAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
	pitchAttitudePIDJoyStick->SetGains(11.755f, 5.267f, 9.008f);

	rollAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
	rollAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
	rollAttitudePIDJoyStick->SetGains(11.755f, 5.267f, 9.008f);

	yawAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
	yawAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
	yawAttitudePIDJoyStick->SetGains(0.f, 0.f, 0.f);

	// Initialize ImGuiUtil instances using MakeUnique
	AutoWaypointHUD = MakeUnique<ImGuiUtil>(
		dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
		Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
		altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
		xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
		pitchAttitudePID.Get(), yawAttitudePID.Get(),
		xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
		rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
		yawAttitudePIDVelocity.Get(),
		xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
		rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
		yawAttitudePIDJoyStick.Get()
	);

	VelocityHUD = MakeUnique<ImGuiUtil>(
		dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
		Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
		altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
		xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
		pitchAttitudePID.Get(), yawAttitudePID.Get(),
		xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
		rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
		yawAttitudePIDVelocity.Get(),
		xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
		rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
		yawAttitudePIDJoyStick.Get()
	);

	JoyStickHUD = MakeUnique<ImGuiUtil>(
		dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
		Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
		altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
		xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
		pitchAttitudePID.Get(), yawAttitudePID.Get(),
		xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
		rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
		yawAttitudePIDVelocity.Get(),
		xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
		rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
		yawAttitudePIDJoyStick.Get()
	);

	ManualThrustHUD = MakeUnique<ImGuiUtil>(
		dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
		Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
		altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
		xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
		pitchAttitudePID.Get(), yawAttitudePID.Get(),
		xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
		rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
		yawAttitudePIDVelocity.Get(),
		xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
		rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
		yawAttitudePIDJoyStick.Get()
	);

	DroneGlobalState::Get().BindController(this);
}

UQuadDroneController::~UQuadDroneController()
{
	DroneGlobalState::Get().UnbindController();
}

void UQuadDroneController::Initialize(AQuadPawn* InPawn)
{
	if (!InPawn)
	{
		UE_LOG(LogTemp, Error, TEXT("Initialize called with null pawn"));
		return;
	}

	if (dronePawn != InPawn)
	{
		UE_LOG(LogTemp, Display, TEXT("Initializing controller for pawn: %s"), *InPawn->GetName());
		dronePawn = InPawn;
	}
}

// ------------ Setter and Getter -------------------
void UQuadDroneController::SetDesiredVelocity(const FVector& NewVelocity)
{
	desiredNewVelocity = NewVelocity;
	UE_LOG(LogTemp, Display, TEXT("SetDesiredVelocity called: %s"), *desiredNewVelocity.ToString());
}

void UQuadDroneController::SetFlightMode(EFlightOptions NewMode)
{
	UE_LOG(LogTemp, Error, TEXT("Setting flight mode to: %s"), *UEnum::GetValueAsString(NewMode));
	currentFlightMode = NewMode;

	// Initialize HUD or control based on mode
	switch (currentFlightMode)
	{
	case EFlightOptions::AutoWaypoint:
		if (AutoWaypointHUD)
		{
			UE_LOG(LogTemp, Error, TEXT(">>> Entered AutoWaypoint Mode <<<"));
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT(">>> Failed to enter AutoWaypoint Mode - HUD not initialized <<<"));
		}
		break;

	case EFlightOptions::JoyStickControl:
		if (JoyStickHUD)
		{
			UE_LOG(LogTemp, Error, TEXT(">>> Entered JoyStick Mode <<<"));
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT(">>> Failed to enter JoyStick Mode - HUD not initialized <<<"));
		}
		break;

	case EFlightOptions::VelocityControl:
		if (VelocityHUD)
		{
			UE_LOG(LogTemp, Error, TEXT(">>> Entered Velocity Mode <<<"));
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT(">>> Failed to enter Velocity Mode - HUD not initialized <<<"));
		}
		break;

	default:
		UE_LOG(LogTemp, Error, TEXT(">>> INVALID FLIGHT MODE SELECTED! <<<"));
		break;
	}
}
// ---------------------- Waypoint Nav ------------------------

void UQuadDroneController::AddNavPlan(const FString& name, const TArray<FVector>& waypoints)
{
	NavPlan plan;
	plan.name = name;
	plan.waypoints = waypoints;
	setPointNavigation.Add(plan);
}

void UQuadDroneController::SetNavPlan(const FString& name)
{
	for (int i = 0; i < setPointNavigation.Num(); i++)
	{
		if (setPointNavigation[i].name == name)
		{
			currentNav = &setPointNavigation[i];
			curPos = 0;
			return;
		}
	}
}

// ---------------------- Reset PIDs ------------------------

void UQuadDroneController::ResetPID()
{
	xPID->Reset();
	yPID->Reset();
	zPID->Reset();
	rollAttitudePID->Reset();
	pitchAttitudePID->Reset();
	yawAttitudePID->Reset();

	xPIDVelocity->Reset();
	yPIDVelocity->Reset();
	zPIDVelocity->Reset();
	rollAttitudePIDVelocity->Reset();
	pitchAttitudePIDVelocity->Reset();
	yawAttitudePIDVelocity->Reset();

	xPIDJoyStick->Reset();
	yPIDJoyStick->Reset();
	zPIDJoyStick->Reset();
	rollAttitudePIDJoyStick->Reset();
	pitchAttitudePIDJoyStick->Reset();
	yawAttitudePIDJoyStick->Reset();
	curPos = 0;
	altitudeReached = false;
}

void UQuadDroneController::ResetAutoDroneIntegral() const
{
	xPID->ResetIntegral();
	yPID->ResetIntegral();
	zPID->ResetIntegral();
	rollAttitudePID->ResetIntegral();
	pitchAttitudePID->ResetIntegral();
	yawAttitudePID->ResetIntegral();
}

void UQuadDroneController::ResetVelocityDroneIntegral() const
{
	xPIDVelocity->ResetIntegral();
	yPIDVelocity->ResetIntegral();
	zPIDVelocity->ResetIntegral();
	rollAttitudePIDVelocity->ResetIntegral();
	pitchAttitudePIDVelocity->ResetIntegral();
	yawAttitudePIDVelocity->ResetIntegral();
}


// ---------------------- Thrust, Desired Vel, Pitch, and Roll ------------------------

void UQuadDroneController::ThrustMixer(float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput)
{
	//Order: FL,FR,BL,BR

	switch (currentFlightMode)
	{
	case EFlightOptions::AutoWaypoint:
		Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
		Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
		Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
		Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
		break;
	case EFlightOptions::VelocityControl:
		Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
		Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
		Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
		Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
		break;
	case EFlightOptions::JoyStickControl:
		Thrusts[0] = zOutput + rollOutput + pitchOutput;
		Thrusts[1] = zOutput - rollOutput + pitchOutput;
		Thrusts[2] = zOutput + rollOutput - pitchOutput;
		Thrusts[3] = zOutput - rollOutput - pitchOutput;
		break;
	}
	for (int i = 0; i < Thrusts.Num(); ++i)
	{
		Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, 600.f);
	}
}


// ---------------------- Update ------------------------
void UQuadDroneController::Update(double DeltaTime)
{
	if (!dronePawn)
	{
		UE_LOG(LogTemp, Error, TEXT("Update called with null dronePawn"));
		return;
	}

	UE_LOG(LogTemp, Error, TEXT("Updating in mode: %s"), *UEnum::GetValueAsString(currentFlightMode));

	switch (currentFlightMode)
	{
	case EFlightOptions::AutoWaypoint:
		UE_LOG(LogTemp, Error, TEXT(">>> Running AutoWaypoint Control <<<"));
		AutoWaypointControl(DeltaTime);
		break;

	case EFlightOptions::JoyStickControl:
		UE_LOG(LogTemp, Error, TEXT(">>> Running JoyStick Control <<<"));
		ApplyControllerInput(DeltaTime);
		break;

	case EFlightOptions::VelocityControl:
		UE_LOG(LogTemp, Error, TEXT(">>> Running Velocity Control <<<"));
		VelocityControl(DeltaTime);
		break;

	default:
		UE_LOG(LogTemp, Error, TEXT(">>> UNKNOWN FLIGHT MODE IN UPDATE! <<<"));
		break;
	}
}
void UQuadDroneController::ApplyControllerInput(double a_deltaTime)
{
	if (!dronePawn) return;

	float droneMass = dronePawn->DroneBody->GetMass();
	const float mult = 0.5f;

	FVector currentPosition = dronePawn->GetActorLocation();
	FRotator currentRotation = dronePawn->GetActorRotation();

	// Initialize desiredAltitude and desiredYaw on the first run
	if (!bDesiredAltitudeInitialized)
	{
		desiredAltitude = currentPosition.Z;
		bDesiredAltitudeInitialized = true;
	}

	if (!bDesiredYawInitialized)
	{
		desiredYaw = currentRotation.Yaw;
		bDesiredYawInitialized = true;
	}

	// ------ Altitude Control ---------
	// Modify desired altitude based on thrust input
	float altitudeRate = 400.0f; // Units per second for altitude change
	desiredAltitude += thrustInput * altitudeRate * a_deltaTime;

	// Calculate error between desired and current altitude
	float z_error = desiredAltitude - currentPosition.Z;
	float z_output = zPIDJoyStick->Calculate(z_error, a_deltaTime);

	// ------ Attitude Control ---------
	float desiredRoll = rollInput * maxAngle;
	float roll_error = desiredRoll - currentRotation.Roll;
	float roll_output = rollAttitudePIDJoyStick->Calculate(roll_error, a_deltaTime);

	float desiredPitch = pitchInput * maxAngle;
	float pitch_error = desiredPitch - currentRotation.Pitch;
	float pitch_output = pitchAttitudePIDJoyStick->Calculate(pitch_error, a_deltaTime);

	// Yaw control 
	float yawRate = 90.0f; // Degrees per second
	desiredYaw += yawInput * yawRate * a_deltaTime;
	desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

	float yaw_error = desiredYaw - currentRotation.Yaw;
	yaw_error = FMath::UnwindDegrees(yaw_error);
	float yaw_output = yawAttitudePIDJoyStick->Calculate(yaw_error, a_deltaTime);

	// Apply yaw torque
	FVector yawTorque = FVector(0.0f, 0.0f, yaw_output);
	dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);

	// Thrust Mixing
	ThrustMixer(0, 0, z_output, roll_output, pitch_output);

	// Apply thrusts to rotors

	for (int i = 0; i < Thrusts.Num(); ++i)
	{
		dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
	}

	// Collect data for ImGui display
	TArray<float> ThrustsVal = Thrusts;
	FVector waypoint(0, 0, desiredAltitude);
	FVector error(0, 0, z_error);
	FVector desiredVelocity(0, 0, 0);
	float xOutput = 0.0f;
	float yOutput = 0.0f;

	JoyStickHUD->JoyStickHud(ThrustsVal, roll_error, pitch_error, currentRotation, waypoint, currentPosition, error,
	                         desiredVelocity, xOutput, yOutput, z_output, a_deltaTime);
}

void UQuadDroneController::AutoWaypointControl(double a_deltaTime)
{
	if (!currentNav || curPos >= currentNav->waypoints.Num()) return;

	// Unreal Feedback and data collection
	float droneMass = dronePawn->DroneBody->GetMass();
	const float mult = 0.5f;
	FVector currentPosition = dronePawn->GetActorLocation();
	FRotator currentRotation = dronePawn->GetActorRotation();
	FVector currentVelocity = dronePawn->GetVelocity();

	// Initialize setPoint based on altitude status
	FVector setPoint = !altitudeReached
		                   ? FVector(currentPosition.X, currentPosition.Y, minAltitudeLocal)
		                   // Must reach minimum altitude first
		                   : currentNav->waypoints[curPos]; // Proceed to waypoint

	// Calculate position error and other variables based on the current setPoint
	FVector positionError = setPoint - currentPosition;

	// Check if the drone has reached the setPoint, if so then reset integral sums    
	if (positionError.Size() < acceptableDistance)
	{
		if (!altitudeReached)
		{
			altitudeReached = true;
			// After reaching minAltitudeLocal, update the setPoint to the next waypoint
			setPoint = currentNav->waypoints[curPos];
			positionError = setPoint - currentPosition;

			// Reset the integral sums of PID controllers
			ResetAutoDroneIntegral();
		}
		else
		{
			// Move to the next waypoint
			curPos++;
			if (curPos >= currentNav->waypoints.Num())
			{
				// Reached the end of the nav plan
				currentNav = nullptr;
				curPos = 0;
				// Stop updating if no more waypoints
				return;
			}
			else
			{
				// Update setPoint and positionError for the new waypoint
				setPoint = currentNav->waypoints[curPos];
				positionError = setPoint - currentPosition;

				// Reset the integral sums of PID controllers
				ResetAutoDroneIntegral();
			}
		}
	}

	// Continue with calculations using the updated positionError
	FVector normalizedError = positionError.GetSafeNormal();
	FVector droneForwardVector = dronePawn->GetActorForwardVector();
	FVector desiredVelocity = DroneMathUtils::CalculateDesiredVelocity(positionError, maxVelocity);

	DrawDebugVisuals(currentPosition, setPoint);

	// ------------------- Yaw Control ---------------------

	// Calculate desiredYaw based on the direction to the waypoint
	desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(positionError.Y, positionError.X));

	// Normalize desiredYaw to [-180, 180]
	desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

	// Compute yaw error
	float yaw_error = desiredYaw - currentRotation.Yaw;

	// Normalize yaw error to [-180, 180]
	yaw_error = FMath::UnwindDegrees(yaw_error);

	// Use yaw PID controller to calculate yaw torque
	float yaw_output = yawAttitudePID->Calculate(yaw_error, a_deltaTime);

	// Apply yaw torque directly to the drone body
	FVector yawTorque = FVector(0.0f, 0.0f, yaw_output); // Torque around Z-axis
	dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);

	// ------------------- Attitude CONTROL ---------------------
	//float yawTorque = CalculateYawTorque(setPoint, currentPosition, currentRotation, a_deltaTime);
	float desiredRoll = DroneMathUtils::CalculateDesiredRoll(normalizedError, droneForwardVector, maxAngle, altitudeThresh);
	float desiredPitch = DroneMathUtils::CalculateDesiredPitch(normalizedError, droneForwardVector, maxAngle, altitudeThresh);

	float roll_error = desiredRoll - currentRotation.Roll;
	float pitch_error = desiredPitch - currentRotation.Pitch;

	float roll_output = rollAttitudePID->Calculate(roll_error,a_deltaTime);
	float pitch_output = pitchAttitudePID->Calculate(pitch_error, a_deltaTime);

	// ------------------- POSITION CONTROL ---------------------
	float x_output = xPID->Calculate(desiredVelocity.X - currentVelocity.X, a_deltaTime);
	float y_output = yPID->Calculate(desiredVelocity.Y - currentVelocity.Y, a_deltaTime);
	float z_output = zPID->Calculate(desiredVelocity.Z - currentVelocity.Z, a_deltaTime);
	//------------------- Thrust Mixing -------------

	ThrustMixer(x_output, y_output, z_output, roll_output, pitch_output);
	// Update the thrust of each rotor
	for (int i = 0; i < Thrusts.Num(); ++i)
	{
		dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
	}

	AutoWaypointHUD->AutoWaypointHud(Thrusts, roll_error, pitch_error, currentRotation, setPoint, currentPosition,
	                                 positionError, currentVelocity, x_output, y_output, z_output, a_deltaTime); //****
	AutoWaypointHUD->RenderImPlot(Thrusts, a_deltaTime);
}


void UQuadDroneController::VelocityControl(double a_deltaTime)
{
	if (!dronePawn) return;

	float droneMass = dronePawn->DroneBody->GetMass();
	const float mult = 0.5f;

	FVector currentVelocity = dronePawn->GetVelocity();
	FRotator currentRotation = dronePawn->GetActorRotation();

	// Calculate velocity error
	FVector velocityError = desiredNewVelocity - currentVelocity;

	// Use PID controllers to calculate outputs
	float x_output = xPIDVelocity->Calculate(velocityError.X, a_deltaTime);
	float y_output = yPIDVelocity->Calculate(velocityError.Y, a_deltaTime);
	float z_output = zPIDVelocity->Calculate(velocityError.Z, a_deltaTime);

	// Attitude stabilization (keeping roll and pitch at zero)
	float roll_error = -currentRotation.Roll;
	float pitch_error = -currentRotation.Pitch;

	float roll_output = rollAttitudePIDVelocity->Calculate(roll_error, a_deltaTime);
	float pitch_output = pitchAttitudePIDVelocity->Calculate(pitch_error, a_deltaTime);

	// Calculate desired yaw based on velocity direction
	FVector horizontalVelocity = desiredNewVelocity;
	horizontalVelocity.Z = 0; // Ignore vertical component for yaw calculation

	// Only update desired yaw if we have significant horizontal velocity
	static constexpr float MIN_VELOCITY_FOR_YAW = 10.0f; // Adjust this threshold as needed
	if (horizontalVelocity.SizeSquared() > MIN_VELOCITY_FOR_YAW * MIN_VELOCITY_FOR_YAW)
	{
		// Calculate the desired yaw angle based on velocity direction
		desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(horizontalVelocity.Y, horizontalVelocity.X));
	}
	else
	{
		// If velocity is too low, maintain current yaw to prevent erratic rotation
		desiredYaw = currentRotation.Yaw;
	}

	// Normalize desiredYaw to [-180, 180]
	desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

	// Calculate yaw error
	float yaw_error = desiredYaw - currentRotation.Yaw;
	yaw_error = FMath::UnwindDegrees(yaw_error); // Normalize the error to [-180, 180]

	// Calculate yaw output using PID
	float yaw_output = yawAttitudePIDVelocity->Calculate(yaw_error, a_deltaTime);

	// Apply yaw torque directly to the drone body
	FVector yawTorque = FVector(0.0f, 0.0f, yaw_output);
	dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);

	// Apply thrust mixing
	ThrustMixer(x_output, y_output, z_output, roll_output, pitch_output);

	// Update rotors
	for (int i = 0; i < Thrusts.Num(); ++i)
	{
		dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
	}

	// Debug visualization for velocity direction
	if (Debug_DrawDroneWaypoint)
	{
		FVector dronePos = dronePawn->GetActorLocation();
		FVector velocityDirection = desiredNewVelocity.GetSafeNormal() * 200.0f; // Scale for visualization

		DrawDebugLine(
			dronePawn->GetWorld(),
			dronePos,
			dronePos + velocityDirection,
			FColor::Yellow,
			false,
			-1.0f,
			0,
			2.0f
		);
	}
	VelocityHUD->VelocityHud(Thrusts, roll_error, pitch_error, currentRotation, FVector::ZeroVector,
	                         dronePawn->GetActorLocation(), FVector::ZeroVector, currentVelocity, x_output, y_output,
	                         z_output, a_deltaTime);
	VelocityHUD->RenderImPlot(Thrusts, a_deltaTime);
}

// ---------------------- Helper Functions ------------------------

void UQuadDroneController::ResetDroneHigh()
{
	if (dronePawn)
	{
		// First, disable physics simulation temporarily
		if (dronePawn->DroneBody)
		{
			dronePawn->DroneBody->SetSimulatePhysics(false);
		}

		// Reset position and rotation
		dronePawn->SetActorLocation(FVector(0.0f, 0.0f, 10000.0f), false, nullptr, ETeleportType::TeleportPhysics);
		dronePawn->SetActorRotation(FRotator::ZeroRotator);

		if (dronePawn->DroneBody)
		{
			// Re-enable physics
			dronePawn->DroneBody->SetSimulatePhysics(true);

			// Reset velocities
			dronePawn->DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
			dronePawn->DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);

			// Wake the physics body to ensure it responds to the new state
			dronePawn->DroneBody->WakeAllRigidBodies();
		}

		// Reset controller states
		ResetPID();
		desiredNewVelocity = FVector::ZeroVector;
		initialTakeoff = true;
		altitudeReached = false;
	}
}

void UQuadDroneController::ResetDroneOrigin()
{
	if (dronePawn)
	{
		// First, disable physics simulation temporarily
		if (dronePawn->DroneBody)
		{
			dronePawn->DroneBody->SetSimulatePhysics(false);
		}

		// Reset position and rotation
		dronePawn->SetActorLocation(FVector(0.0f, 0.0f, 10.0f), false, nullptr, ETeleportType::TeleportPhysics);
		dronePawn->SetActorRotation(FRotator::ZeroRotator);

		if (dronePawn->DroneBody)
		{
			// Re-enable physics
			dronePawn->DroneBody->SetSimulatePhysics(true);

			// Reset velocities
			dronePawn->DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
			dronePawn->DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);

			// Wake the physics body to ensure it responds to the new state
			dronePawn->DroneBody->WakeAllRigidBodies();
		}

		// Reset controller states
		ResetPID();
		desiredNewVelocity = FVector::ZeroVector;
		initialTakeoff = true;
		altitudeReached = false;
	}
}

void UQuadDroneController::DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint) const
{
	if (Debug_DrawDroneCollisionSphere)
	{
		// Get the mesh bounds to calculate the vertical offset
		FBoxSphereBounds MeshBounds = dronePawn->DroneBody->CalcBounds(dronePawn->DroneBody->GetComponentTransform());
		float VerticalOffset = MeshBounds.BoxExtent.Z;

		// Adjust the position upwards
		FVector AdjustedPosition = currentPosition + FVector(0.0f, 0.0f, VerticalOffset);

		// Draw the debug sphere at the adjusted position
		DrawDebugSphere(
			dronePawn->GetWorld(),
			AdjustedPosition,
			dronePawn->DroneBody->GetCollisionShape().GetSphereRadius(),
			10,
			FColor::Red,
			false, // bPersistentLines
			0.0f // LifeTime
		);
	}

	if (Debug_DrawDroneWaypoint)
	{
		// Existing code remains the same
		DrawDebugSphere(
			dronePawn->GetWorld(),
			setPoint,
			acceptableDistance,
			10,
			FColor::Red,
			false, // bPersistentLines
			0.0f // LifeTime
		);
		DrawDebugLine(
			dronePawn->GetWorld(),
			currentPosition,
			setPoint,
			FColor::Green,
			false, // bPersistentLines
			0.0f // LifeTime
		);
	}
}

void UQuadDroneController::HandleThrustInput(float Value)
{
	// Value ranges from -1 to 1
	thrustInput = Value; // Store the input value for use in ApplyControllerInput
}

void UQuadDroneController::HandleYawInput(float Value)
{
	yawInput = Value; // Store the raw input for later use with interpolation
}

void UQuadDroneController::HandlePitchInput(float Value)
{
	pitchInput = Value; // Map Value to desired pitch range
}

void UQuadDroneController::HandleRollInput(float Value)
{
	rollInput = Value; // Map Value to desired roll range
}


FVector UQuadDroneController::GetCurrentAltitude() const 
{
	if (dronePawn)
	{
		return dronePawn->GetActorLocation();
	}
	return FVector::ZeroVector;
}

FVector UQuadDroneController::GetCurrentVelocity() const 
{
	if (dronePawn)
	{
		return dronePawn->GetVelocity();
	}
	return FVector::ZeroVector;
}

void UQuadDroneController::SwitchCamera()
{
	if (dronePawn)
	{
		dronePawn->SwitchCamera();
	}
}