// QuadDroneController.cpp

#include "Controllers/QuadDroneController.h"
#include "Pawns/QuadPawn.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"
#include "Core/DroneGlobalState.h"
#include "UI/ImGuiUtil.h"
#include "Core/DroneJSONConfig.h"
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



// ---------------------- Thrust, Desired Vel, Pitch, and Roll ------------------------

void UQuadDroneController::ThrustMixer(float xOutput, float yOutput, float zOutput,
                                       float rollOutput, float pitchOutput, float yawOutput)
{
	
    switch (currentFlightMode)
    {
    case EFlightMode::None:
        // no thrust
        break;

    case EFlightMode::VelocityControl:
        // Mix the translational forces as before
        Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput + yawOutput;  // FL
        Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput - yawOutput;  // FR
        Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput - yawOutput;  // BL
        Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput + yawOutput;  // BR
        break;
    }
	
	static const int SpinDirections[4] = { +1, -1, -1, +1 };

	for (int i = 0; i < Thrusts.Num(); ++i)
	{
		if (!dronePawn || !dronePawn->Thrusters.IsValidIndex(i))
			continue;

		// Apply thrust force
		float droneMass = dronePawn->DroneBody->GetMass();
		const float mult = 0.5f;
		float forceVal = droneMass * mult * Thrusts[i];
		dronePawn->Thrusters[i]->ApplyForce(forceVal);

		// Calculate and apply torque
		FVector RotorAxisWorld = dronePawn->Thrusters[i]->GetComponentTransform().GetUnitAxis(EAxis::Z);
		
		// Scale torque based on thrust and spin direction
		// This couples the torque to the thrust magnitude for more realistic behavior
		float torqueValue = forceVal * 0.01f * SpinDirections[i] + yawOutput * 0.5f * SpinDirections[i];
		
		// Apply both the yaw torque and the rotor-induced torque
		FVector torqueVec = RotorAxisWorld * torqueValue;
		dronePawn->Thrusters[i]->ApplyTorque(torqueVec, false); // Using radians

		FVector upRef = FVector::UpVector;
		if (FMath::Abs(FVector::DotProduct(RotorAxisWorld, upRef)) > 0.95f)
		{
			upRef = FVector::ForwardVector;
		}

		FVector tangentDir = RotorAxisWorld ^ upRef; // cross product
		tangentDir.Normalize();

		float debugLength = torqueValue * 20.f; 
		FVector thrusterLocation = dronePawn->Thrusters[i]->GetComponentLocation();

		FColor lineColor = (torqueValue >= 0.0f) ? FColor::Magenta : FColor::Cyan;
		DrawDebugLine(
			dronePawn->GetWorld(),
			thrusterLocation,
			thrusterLocation + (tangentDir * debugLength),
			lineColor,
			false, -1.f, 0, 2.f
		);
	}
	
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

void UQuadDroneController::VelocityControl(double a_deltaTime)
{
	FFullPIDSet* CurrentSet = PIDMap.Find(EFlightMode::VelocityControl);
	if (!CurrentSet) return;
	if (!dronePawn) return;
	
	FVector currentVelocity = dronePawn->GetVelocity();
	FRotator currentRotation = dronePawn->GetActorRotation();
	FVector currentPosition = dronePawn->GetActorLocation();
	// Safety checks
	bool needsReset = false;
	
	// Check for dangerous orientation
	if (currentPosition.Z && FMath::Abs(currentRotation.Roll) > 60.0f || 
		FMath::Abs(currentRotation.Pitch) > 60.0f)
	{
		needsReset = true;
		UE_LOG(LogTemp, Warning, TEXT("Safety Reset: Dangerous orientation - Roll: %f, Pitch: %f"), 
			   currentRotation.Roll, currentRotation.Pitch);
	}
	
	if (needsReset)
	{
		// Reset drone's orientation and velocities
		if (dronePawn->DroneBody)
		{
			// Reset rotation but keep position
			FRotator safeRotation = FRotator(0.0f, currentRotation.Yaw, 0.0f);
			dronePawn->SetActorRotation(safeRotation, ETeleportType::TeleportPhysics);
            
			// Zero out velocities
			dronePawn->DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
			dronePawn->DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
            
			// Reset PID controllers
			ResetDroneIntegral();
		}
	}

	static float totalTime = 0.0f;
	totalTime += a_deltaTime;
    
	// Calculate velocity error
	FVector velocityError = desiredNewVelocity - currentVelocity;

	// Use PID controllers to calculate outputs
	float x_output = CurrentSet->XPID->Calculate(velocityError.X, a_deltaTime);
	float y_output = CurrentSet->YPID->Calculate(velocityError.Y, a_deltaTime);
	float z_output = CurrentSet->ZPID->Calculate(velocityError.Z, a_deltaTime);

	// Attitude stabilization (keeping roll and pitch at zero)
	float roll_error = -currentRotation.Roll;
	float pitch_error = -currentRotation.Pitch;

	float roll_output = CurrentSet->RollPID->Calculate(roll_error, a_deltaTime);
	float pitch_output = CurrentSet->PitchPID->Calculate(pitch_error, a_deltaTime);

	// Calculate desired yaw based on velocity direction
	FVector horizontalVelocity = desiredNewVelocity;
	horizontalVelocity.Z = 0; 

	// Only update desired yaw if we have significant horizontal velocity
	static constexpr float MIN_VELOCITY_FOR_YAW = 10.0f;
	if (horizontalVelocity.SizeSquared() > MIN_VELOCITY_FOR_YAW * MIN_VELOCITY_FOR_YAW)
	{
		desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(horizontalVelocity.Y, horizontalVelocity.X));
	}
	else
	{
		desiredYaw = currentRotation.Yaw;
	}

	// Normalize desiredYaw to [-180, 180]
	desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

	// Calculate yaw error
	float yaw_error = desiredYaw - currentRotation.Yaw;
	yaw_error = FMath::UnwindDegrees(yaw_error);

	// Calculate yaw output using PID
	float yaw_output = CurrentSet->YawPID->Calculate(yaw_error, a_deltaTime);

	ThrustMixer(x_output, y_output, z_output, roll_output, pitch_output, yaw_output);

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
