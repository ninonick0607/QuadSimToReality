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

static constexpr float MotorThrustCoefficient = 1e-7f;

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
	  , VelocityHUD(nullptr)
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
	
    FFullPIDSet VelocitySet;

    // 3) Fill VelocitySet
    VelocitySet.XPID = new QuadPIDController();
    VelocitySet.XPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.XPID->SetGains(1.f, 0.f, 0.1f);

    VelocitySet.YPID = new QuadPIDController();
    VelocitySet.YPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.YPID->SetGains(1.f, 0.f, 0.1f);

    VelocitySet.ZPID = new QuadPIDController();
    VelocitySet.ZPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.ZPID->SetGains(5.f, 1.f, 0.1f);

    VelocitySet.RollPID = new QuadPIDController();
    VelocitySet.RollPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.RollPID->SetGains(8.f, 0.3f, 3.7f);

    VelocitySet.PitchPID = new QuadPIDController();
    VelocitySet.PitchPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.PitchPID->SetGains(8.f, 0.3f, 3.7f);

    VelocitySet.YawPID = new QuadPIDController();
    VelocitySet.YawPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.YawPID->SetGains(1.8f, 0.15f, 1.5f);
	PIDMap.Add(EFlightMode::VelocityControl, MoveTemp(VelocitySet));

	VelocityHUD = MakeUnique<ImGuiUtil>(dronePawn, this, desiredNewVelocity,Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput, maxVelocity, maxAngle);
	
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


// ---------------------- Update ------------------------

void UQuadDroneController::Update(double a_deltaTime)
{
	SetFlightMode(EFlightMode::VelocityControl);
	VelocityControl(a_deltaTime);

	switch (currentFlightMode)
	{
	case EFlightMode::None:
		return;
	case EFlightMode::VelocityControl:

		break;
	}
}

// ---------------------- Cascaded Control: VelocityControl ------------------------
//
// This implements the PX4 cascaded architecture. We assume:
// 1. Outer Loop: Position controller (P only) that maps the error (setpoint minus current position)
//    into a desired velocity (in cm/s). The desired velocity is then clamped to maxVelocity.
// 2. Middle Loop: Velocity PID controllers produce acceleration commands. However, here we want
//    the PID outputs to directly be RPM values (for our motors). For the vertical channel, we compute
//    the required upward acceleration (gravity + PID output) and convert that into a "base" RPM via ThrustToRPM().
// 3. Inner Loop: The horizontal acceleration commands (ax and ay) are converted into desired pitch and roll
//    angles (using a small-angle approximation with gravity = 980 cm/s²). Their errors are fed into the attitude PIDs,
//    whose outputs are interpreted as RPM corrections.
// 4. Finally, the ThrustMixer sums the base RPM with the attitude corrections to yield the desired motor RPM for each rotor.
// 5. When applying force, the computed motor RPM is converted back into a force (Newtons) via RPMToThrust().
//    (Note: We ignore yaw here.)
//
void UQuadDroneController::VelocityControl(double a_deltaTime)
{
	FFullPIDSet* CurrentSet = PIDMap.Find(EFlightMode::VelocityControl);
	if (!CurrentSet || !dronePawn)
		return;
	
	FVector currentPosition = dronePawn->GetActorLocation();
	FVector currentVelocity = dronePawn->GetVelocity();
	FRotator currentRotation = dronePawn->GetActorRotation();
	FVector droneForwardVector = dronePawn->GetActorForwardVector();

	// Safety check: if dangerous orientation, reset.
	bool needsReset = false;
	if (currentPosition.Z && (FMath::Abs(currentRotation.Roll) > 60.0f || FMath::Abs(currentRotation.Pitch) > 60.0f))
	{
		needsReset = true;
		UE_LOG(LogTemp, Warning, TEXT("Safety Reset: Dangerous orientation - Roll: %f, Pitch: %f"), currentRotation.Roll, currentRotation.Pitch);
	}
	if (needsReset)
	{
		if (dronePawn->DroneBody)
		{
			FRotator safeRotation = FRotator(0.0f, currentRotation.Yaw, 0.0f);
			dronePawn->SetActorRotation(safeRotation, ETeleportType::TeleportPhysics);
			dronePawn->DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
			dronePawn->DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
			ResetDroneIntegral();
		}
	}

	// --- Outer Loop: Position Controller ---
	FVector positionError = desiredNewVelocity - currentPosition;
	float Kp_Position = 0.1f;  // Tunable gain (could be exposed via ImGui)
	FVector desiredVelocity = positionError * Kp_Position;
	desiredVelocity = DroneMathUtils::ClampVectorMagnitude(desiredVelocity, maxVelocity);

	// Draw Debug: Desired velocity vector (from current position)
	{
		FVector debugEnd = currentPosition + desiredVelocity;
		DrawDebugLine(dronePawn->GetWorld(), currentPosition, debugEnd, FColor::Yellow, false, 0.1f, 0, 2.0f);
	}

	// --- Middle Loop: Velocity Controller ---
	FVector velocityError = desiredNewVelocity - currentVelocity;  // Now correctly using desired velocity
    
	// Compute acceleration commands from velocity PIDs
	float ax = CurrentSet->XPID->Calculate(velocityError.X, a_deltaTime);
	float ay = CurrentSet->YPID->Calculate(velocityError.Y, a_deltaTime);
	float az = CurrentSet->ZPID->Calculate(velocityError.Z, a_deltaTime) + 980.0f; // Add gravity compensation
	
	// Normalize acceleration vector for attitude computation
	FVector normalizedAccel = FVector(ax, ay, az).GetSafeNormal();
    
	// Use your existing functions for attitude calculation
	float desiredRoll = DroneMathUtils::CalculateDesiredRoll(
		normalizedAccel, droneForwardVector, maxAngle, altitudeThresh);
	float desiredPitch = DroneMathUtils::CalculateDesiredPitch(
		normalizedAccel, droneForwardVector, maxAngle, altitudeThresh);

	// --- Inner Loop: Attitude Controller ---
	float rollError = desiredRoll - currentRotation.Roll;
	float pitchError = desiredPitch - currentRotation.Pitch;

	float rollCorr = CurrentSet->RollPID->Calculate(rollError, a_deltaTime);
	float pitchCorr = CurrentSet->PitchPID->Calculate(pitchError, a_deltaTime);

	// Compute thrust in Newtons
	float droneMass = dronePawn->DroneBody->GetMass();
	float totalThrust = droneMass * az;  // F = ma
	float thrustPerMotor = totalThrust / 4.0f;

	// Mix thrust and attitude corrections
	ThrustMixer(thrustPerMotor, rollCorr, pitchCorr);

	// Debug visualization
	DrawDebugLine(dronePawn->GetWorld(), currentPosition, 
				  currentPosition + desiredNewVelocity, 
				  FColor::Yellow, false, 0.1f, 0, 2.0f);

	// Update HUD
	VelocityHUD->VelocityHud(Thrusts, rollError, pitchError, currentRotation,
		FVector::ZeroVector, currentPosition, FVector::ZeroVector, 
		currentVelocity, ax, ay, az, a_deltaTime);
}

// ---------------------- RPM Conversion Functions ------------------------

// Converts a thrust command (in Newtons) to an RPM value.
float UQuadDroneController::ThrustToRPM(float thrust)
{
	if (thrust < 0.f) thrust = 0.f;
	float omega = FMath::Sqrt(thrust / MotorThrustCoefficient); // rad/s
	return (omega * 60.f) / (2.f * PI);
}

// Converts an RPM value back to thrust (in Newtons).
float UQuadDroneController::RPMToThrust(float rpm)
{
	float omega = (rpm * 2.f * PI) / 60.f; // rad/s
	return MotorThrustCoefficient * FMath::Pow(omega, 2);
}

// ---------------------- Thrust Mixer ------------------------
//
// This mixer now takes the base RPM (from vertical thrust) and adds attitude corrections
// (in RPM) from the inner loop. It produces a desired RPM for each motor in an X configuration:
//   Motor 0 (Front Left):  baseRPM + rollCorr + pitchCorr
//   Motor 1 (Front Right): baseRPM - rollCorr + pitchCorr
//   Motor 2 (Back Left):   baseRPM + rollCorr - pitchCorr
//   Motor 3 (Back Right):  baseRPM - rollCorr - pitchCorr
//
// Then, when applying force, we convert each motor’s desired RPM back to thrust (Newtons).
void UQuadDroneController::ThrustMixer(float baseThrust, float rollCorr, float pitchCorr)
{
	// Compute the desired RPM for each motor.
	const float ROLL_SCALE = 0.2f;  // 20% of base thrust for roll authority
	const float PITCH_SCALE = 0.2f; // 20% of base thrust for pitch authority
    
	float rollAdjustment = baseThrust * ROLL_SCALE * rollCorr;
	float pitchAdjustment = baseThrust * PITCH_SCALE * pitchCorr;

	// Calculate thrust for each motor (in Newtons)
	// Front Left
	Thrusts[0] = baseThrust + rollAdjustment + pitchAdjustment;
	// Front Right  
	Thrusts[1] = baseThrust - rollAdjustment + pitchAdjustment;
	// Back Left
	Thrusts[2] = baseThrust + rollAdjustment - pitchAdjustment;
	// Back Right
	Thrusts[3] = baseThrust - rollAdjustment - pitchAdjustment;
	// --- Apply Forces ---
	// For each motor, convert the desired RPM back into thrust (in Newtons)
	// and apply that force at the thruster's location.

	const float MAX_THRUST_PER_MOTOR = 26.0f; // About 2x hover thrust for your 5.3kg drone
	for(int i = 0; i < Thrusts.Num(); i++)
	{
		Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, MAX_THRUST_PER_MOTOR);
	}
	
	for (int i = 0; i < Thrusts.Num(); ++i)
	{
		if (!dronePawn || !dronePawn->Thrusters.IsValidIndex(i))
			continue;
		
		float appliedForce = RPMToThrust(Thrusts[i]);
		dronePawn->Thrusters[i]->ApplyForce(appliedForce);
		
		// (Torque and yaw are ignored in this configuration.)
	}
}

//
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


void UQuadDroneController::bufferDebug(FFullPIDSet* PID_Set)
{
	// In AutoWaypointControl (and similarly in VelocityControl)
	if (ImGui::CollapsingHeader("PID Debug Info"))
	{
		// Position Controllers
		ImGui::Text("Position Controllers:");
		ImGui::Text("X PID - Buffer Size: %d, Sum: %.4f", 
			PID_Set->XPID->GetBufferSize(), 
			PID_Set->XPID->GetCurrentBufferSum());
		ImGui::Text("Y PID - Buffer Size: %d, Sum: %.4f", 
			PID_Set->YPID->GetBufferSize(), 
			PID_Set->YPID->GetCurrentBufferSum());
		ImGui::Text("Z PID - Buffer Size: %d, Sum: %.4f", 
			PID_Set->ZPID->GetBufferSize(), 
			PID_Set->ZPID->GetCurrentBufferSum());

		// Attitude Controllers
		ImGui::Text("\nAttitude Controllers:");
		ImGui::Text("Roll PID - Buffer Size: %d, Sum: %.4f", 
			PID_Set->RollPID->GetBufferSize(), 
			PID_Set->RollPID->GetCurrentBufferSum());
		ImGui::Text("Pitch PID - Buffer Size: %d, Sum: %.4f", 
			PID_Set->PitchPID->GetBufferSize(), 
			PID_Set->PitchPID->GetCurrentBufferSum());
		ImGui::Text("Yaw PID - Buffer Size: %d, Sum: %.4f", 
			PID_Set->YawPID->GetBufferSize(), 
			PID_Set->YawPID->GetCurrentBufferSum());

		// Time Window Info
		ImGui::Text("\nIntegral Window Duration: %.1f seconds", 2);
    
		// Add a button to manually reset integrals for testing
		if (ImGui::Button("Reset All Integrals"))
		{
			ResetDroneIntegral();
		}
	}
}
