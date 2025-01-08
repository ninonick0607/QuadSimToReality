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
	  , currentFlightMode(EFlightMode::None)
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
	
    // 1) Create 3 separate FFullPIDSets: 
    FFullPIDSet AutoWaypointSet;
    FFullPIDSet VelocitySet;
    FFullPIDSet JoyStickSet;

    // 2) Fill AutoWaypointSet
    AutoWaypointSet.XPID = MakeUnique<QuadPIDController>();
    AutoWaypointSet.XPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    AutoWaypointSet.XPID->SetGains(1.f, 0.f, 0.1f);

    AutoWaypointSet.YPID = MakeUnique<QuadPIDController>();
    AutoWaypointSet.YPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    AutoWaypointSet.YPID->SetGains(1.f, 0.f, 0.1f);

    AutoWaypointSet.ZPID = MakeUnique<QuadPIDController>();
    AutoWaypointSet.ZPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    AutoWaypointSet.ZPID->SetGains(5.f, 1.f, 0.1f);

    AutoWaypointSet.RollPID = MakeUnique<QuadPIDController>();
    AutoWaypointSet.RollPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    AutoWaypointSet.RollPID->SetGains(2.934f, 0.297f, 3.633f);

    AutoWaypointSet.PitchPID = MakeUnique<QuadPIDController>();
    AutoWaypointSet.PitchPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    AutoWaypointSet.PitchPID->SetGains(2.934f, 0.297f, 3.633f);

    AutoWaypointSet.YawPID = MakeUnique<QuadPIDController>();
    AutoWaypointSet.YawPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    AutoWaypointSet.YawPID->SetGains(0.f, 0.f, 0.f);

    // 3) Fill VelocitySet
    VelocitySet.XPID = MakeUnique<QuadPIDController>();
    VelocitySet.XPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.XPID->SetGains(1.f, 0.f, 0.1f);

    VelocitySet.YPID = MakeUnique<QuadPIDController>();
    VelocitySet.YPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.YPID->SetGains(1.f, 0.f, 0.1f);

    VelocitySet.ZPID = MakeUnique<QuadPIDController>();
    VelocitySet.ZPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.ZPID->SetGains(5.f, 1.f, 0.1f);

    VelocitySet.RollPID = MakeUnique<QuadPIDController>();
    VelocitySet.RollPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.RollPID->SetGains(8.f, 0.3f, 3.7f);

    VelocitySet.PitchPID = MakeUnique<QuadPIDController>();
    VelocitySet.PitchPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.PitchPID->SetGains(8.f, 0.3f, 3.7f);

    VelocitySet.YawPID = MakeUnique<QuadPIDController>();
    VelocitySet.YawPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.YawPID->SetGains(1.8f, 0.15f, 1.5f);

    // 4) Fill JoyStickSet
    JoyStickSet.XPID = MakeUnique<QuadPIDController>();
    JoyStickSet.XPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    JoyStickSet.XPID->SetGains(2.329f, 3.626f, 1.832f);

    JoyStickSet.YPID = MakeUnique<QuadPIDController>();
    JoyStickSet.YPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    JoyStickSet.YPID->SetGains(2.329f, 3.626f, 1.832f);

    JoyStickSet.ZPID = MakeUnique<QuadPIDController>();
    JoyStickSet.ZPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    JoyStickSet.ZPID->SetGains(5.344f, 1.f, 0.1f);

    JoyStickSet.RollPID = MakeUnique<QuadPIDController>();
    JoyStickSet.RollPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    JoyStickSet.RollPID->SetGains(11.755f, 5.267f, 9.008f);

    JoyStickSet.PitchPID = MakeUnique<QuadPIDController>();
    JoyStickSet.PitchPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    JoyStickSet.PitchPID->SetGains(11.755f, 5.267f, 9.008f);

    JoyStickSet.YawPID = MakeUnique<QuadPIDController>();
    JoyStickSet.YawPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    JoyStickSet.YawPID->SetGains(0.f, 0.f, 0.f);

	PIDMap.Add(EFlightMode::AutoWaypoint, MoveTemp(AutoWaypointSet));
    PIDMap.Add(EFlightMode::VelocityControl, MoveTemp(VelocitySet));
    PIDMap.Add(EFlightMode::JoyStickControl, MoveTemp(JoyStickSet));
	
	// Initialize ImGuiUtil instances using MakeUnique
	AutoWaypointHUD = MakeUnique<ImGuiUtil>(dronePawn, this, desiredNewVelocity,Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput, maxVelocity, maxAngle);
	VelocityHUD = MakeUnique<ImGuiUtil>(dronePawn, this, desiredNewVelocity,Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput, maxVelocity, maxAngle);
	JoyStickHUD = MakeUnique<ImGuiUtil>(dronePawn, this, desiredNewVelocity,Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput, maxVelocity, maxAngle);
	
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
	// For each flight mode in the TMap, reset all six
	for (auto& Elem : PIDMap)
	{
		FFullPIDSet& ThisSet = Elem.Value;

		ThisSet.XPID->Reset();
		ThisSet.YPID->Reset();
		ThisSet.ZPID->Reset();
		ThisSet.RollPID->Reset();
		ThisSet.PitchPID->Reset();
		ThisSet.YawPID->Reset();
	}

	curPos = 0;
	altitudeReached = false;
}
void UQuadDroneController::ResetDroneIntegral()
{
	FFullPIDSet* CurrentSet = PIDMap.Find(currentFlightMode);
	if (!CurrentSet)
	{
		UE_LOG(LogTemp, Warning, TEXT("ResetDroneIntegral: No PID set found for current flight mode %d"), (int32)currentFlightMode);
		return;
	}

	CurrentSet->XPID->ResetIntegral();
	CurrentSet->YPID->ResetIntegral();
	CurrentSet->ZPID->ResetIntegral();
	CurrentSet->RollPID->ResetIntegral();
	CurrentSet->PitchPID->ResetIntegral();
	CurrentSet->YawPID->ResetIntegral();
}



// ---------------------- Thrust, Desired Vel, Pitch, and Roll ------------------------

void UQuadDroneController::ThrustMixer(float xOutput, float yOutput, float zOutput, float rollOutput, float pitchOutput)
{
	//Order: FL,FR,BL,BR

	switch (currentFlightMode)
	{
	case EFlightMode::None:
		break;
	case EFlightMode::AutoWaypoint:
		Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
		Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
		Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
		Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
		break;
	case EFlightMode::VelocityControl:
		Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
		Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
		Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
		Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
		break;
	case EFlightMode::JoyStickControl:
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

void UQuadDroneController::Update(double a_deltaTime)
{
	ImGui::Begin("Flight Mode Selector");

	if (ImGui::Button("Auto Waypoint", ImVec2(200, 50)))
	{
		SetFlightMode(EFlightMode::AutoWaypoint);
		curPos = 0; // Reset to start navigation
	}
	if (ImGui::Button("JoyStick Control", ImVec2(200, 50)))
	{
		SetFlightMode(EFlightMode::JoyStickControl);
	}

	if (ImGui::Button("Move By Velocity", ImVec2(200, 50)))
	{
		SetFlightMode(EFlightMode::VelocityControl);
	}
	ImGui::End();

	switch (currentFlightMode)
	{
	case EFlightMode::None:
		return;
	case EFlightMode::AutoWaypoint:
		AutoWaypointControl(a_deltaTime);
		break;
	case EFlightMode::JoyStickControl:
		ApplyControllerInput(a_deltaTime);
		break;
	case EFlightMode::VelocityControl:
		VelocityControl(a_deltaTime);
		break;
	}
}

void UQuadDroneController::ApplyControllerInput(double a_deltaTime)
{
	FFullPIDSet* CurrentSet = PIDMap.Find(EFlightMode::AutoWaypoint);
	if (!CurrentSet) return;
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
	float z_output =  CurrentSet->ZPID->Calculate(z_error, a_deltaTime);

	// ------ Attitude Control ---------
	float desiredRoll = rollInput * maxAngle;
	float roll_error = desiredRoll - currentRotation.Roll;
	float roll_output =  CurrentSet->RollPID->Calculate(roll_error, a_deltaTime);

	float desiredPitch = pitchInput * maxAngle;
	float pitch_error = desiredPitch - currentRotation.Pitch;
	float pitch_output = CurrentSet->PitchPID->Calculate(pitch_error, a_deltaTime);

	// Yaw control 
	float yawRate = 90.0f; // Degrees per second
	desiredYaw += yawInput * yawRate * a_deltaTime;
	desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

	float yaw_error = desiredYaw - currentRotation.Yaw;
	yaw_error = FMath::UnwindDegrees(yaw_error);
	float yaw_output = CurrentSet->YawPID->Calculate(yaw_error, a_deltaTime);

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
	FFullPIDSet* CurrentSet = PIDMap.Find(EFlightMode::AutoWaypoint);
	if (!CurrentSet) return;
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
			ResetDroneIntegral();
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
				ResetDroneIntegral();
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
	float yaw_output   = CurrentSet->YawPID->Calculate(yaw_error, a_deltaTime);

	// Apply yaw torque directly to the drone body
	FVector yawTorque = FVector(0.0f, 0.0f, yaw_output); // Torque around Z-axis
	dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);

	// ------------------- Attitude CONTROL ---------------------
	//float yawTorque = CalculateYawTorque(setPoint, currentPosition, currentRotation, a_deltaTime);
	float desiredRoll = DroneMathUtils::CalculateDesiredRoll(normalizedError, droneForwardVector, maxAngle, altitudeThresh);
	float desiredPitch = DroneMathUtils::CalculateDesiredPitch(normalizedError, droneForwardVector, maxAngle, altitudeThresh);

	float roll_error = desiredRoll - currentRotation.Roll;
	float pitch_error = desiredPitch - currentRotation.Pitch;

	float roll_output  = CurrentSet->RollPID->Calculate(roll_error, a_deltaTime);
	float pitch_output = CurrentSet->PitchPID->Calculate(pitch_error, a_deltaTime);

	// ------------------- POSITION CONTROL ---------------------
	float x_output = CurrentSet->XPID->Calculate(desiredVelocity.X - currentVelocity.X, a_deltaTime);
	float y_output = CurrentSet->YPID->Calculate(desiredVelocity.Y - currentVelocity.Y, a_deltaTime);
	float z_output = CurrentSet->ZPID->Calculate(desiredVelocity.Z - currentVelocity.Z, a_deltaTime);

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
	FFullPIDSet* CurrentSet = PIDMap.Find(EFlightMode::AutoWaypoint);
	if (!CurrentSet) return;
	if (!dronePawn) return;

	float droneMass = dronePawn->DroneBody->GetMass();
	const float mult = 0.5f;

	FVector currentVelocity = dronePawn->GetVelocity();
	FRotator currentRotation = dronePawn->GetActorRotation();

	// Calculate velocity error
	FVector velocityError = desiredNewVelocity - currentVelocity;

	// Use PID controllers to calculate outputs
	float x_output = CurrentSet->XPID->Calculate(velocityError.X, a_deltaTime);
	float y_output = CurrentSet->YPID->Calculate(velocityError.Y, a_deltaTime);
	float z_output = CurrentSet->ZPID->Calculate(velocityError.Z, a_deltaTime);

	// Attitude stabilization (keeping roll and pitch at zero)
	float roll_error = -currentRotation.Roll;
	float pitch_error = -currentRotation.Pitch;


	float roll_output  = CurrentSet->RollPID->Calculate(roll_error, a_deltaTime);
	float pitch_output = CurrentSet->PitchPID->Calculate(pitch_error, a_deltaTime);
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
	float yaw_output   = CurrentSet->YawPID->Calculate(yaw_error, a_deltaTime);

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



// ------------ Setter and Getter -------------------
void UQuadDroneController::SetDesiredVelocity(const FVector& NewVelocity)
{
	desiredNewVelocity = NewVelocity;
	UE_LOG(LogTemp, Display, TEXT("SetDesiredVelocity called: %s"), *desiredNewVelocity.ToString());
}

void UQuadDroneController::SetFlightMode(EFlightMode NewMode)
{
	currentFlightMode = NewMode;
}

EFlightMode UQuadDroneController::GetFlightMode() const
{
	return currentFlightMode;
}


