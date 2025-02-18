
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
	, desiredNewVelocity(FVector::ZeroVector)
	, initialTakeoff(true)
	, altitudeReached(false)
	, Debug_DrawDroneCollisionSphere(true)
	, Debug_DrawDroneWaypoint(true)
	, thrustInput(0.0f)
	, yawInput(0.0f)
	, pitchInput(0.0f)
	, rollInput(0.0f)
	, hoverThrust(0.0f)
	, bHoverThrustInitialized(false)
	, MaxAngularVelocity(180.0)  
	, YawTorqueForce(2.0)       
	, LastYawTorqueApplied(0.0)
	, UpsideDown(false)
{
	// Load config values
	const auto& Config = UDroneJSONConfig::Get().Config;
	maxVelocity = Config.FlightParams.MaxVelocity;
	maxAngle = Config.FlightParams.MaxAngle;
	maxPIDOutput = Config.FlightParams.MaxPIDOutput;
	altitudeThresh = Config.FlightParams.AltitudeThreshold;
	minAltitudeLocal = Config.FlightParams.MinAltitudeLocal;
	acceptableDistance = Config.FlightParams.AcceptableDistance;
	
    FFullPIDSet VelocitySet;
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

	// Save the pawn pointer.
	if (dronePawn != InPawn)
	{
		UE_LOG(LogTemp, Display, TEXT("Initializing controller for pawn: %s"), *InPawn->GetName());
		dronePawn = InPawn;
	}


}




// ---------------------- Reset PIDs ------------------------

void UQuadDroneController::ResetPID()
{
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


	if (dronePawn && dronePawn->ZMQController)
	{
		FString DroneID = dronePawn->ZMQController->GetConfiguration().DroneID;
		FVector DroneLocation = dronePawn->GetActorLocation();
        
		FVector Offset(0, 0, dronePawn->DroneBody->Bounds.BoxExtent.Z + 20.f);
		FVector DebugLocation = DroneLocation + Offset;

		DrawDebugString(
			dronePawn->GetWorld(),   
			DebugLocation,           
			DroneID,                 
			nullptr,                 
			FColor::Green,           
			0.f,                     
			true,                    
			1.2f                     
		);
	}
}

void UQuadDroneController::VelocityControl(double a_deltaTime)
{
    FFullPIDSet* CurrentSet = PIDMap.Find(EFlightMode::VelocityControl);
    if (!CurrentSet || !dronePawn)
        return;
    
    FVector currentPosition = dronePawn->GetActorLocation();
    FVector currentVelocity = dronePawn->GetVelocity();
    FRotator currentRotation = dronePawn->GetActorRotation();

	FVector horizontalVelocity = desiredNewVelocity;
	horizontalVelocity.Z = 0;
	
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

    // --- Middle Loop: Velocity Controller ---
    FVector velocityError = desiredNewVelocity - currentVelocity;  
	float x_output = CurrentSet->XPID->Calculate(velocityError.X, a_deltaTime);
    float y_output = CurrentSet->YPID->Calculate(velocityError.Y, a_deltaTime);
    float z_output = CurrentSet->ZPID->Calculate(velocityError.Z, a_deltaTime);
	
	float roll_error = -currentRotation.Roll;
	float pitch_error = -currentRotation.Pitch;

    float roll_output = CurrentSet->RollPID->Calculate(roll_error, a_deltaTime);
    float pitch_output = CurrentSet->PitchPID->Calculate(pitch_error, a_deltaTime);

    // --- Yaw Control using PID ---
	static constexpr float MIN_VELOCITY_FOR_YAW = 10.0f;
	if (horizontalVelocity.SizeSquared() > MIN_VELOCITY_FOR_YAW * MIN_VELOCITY_FOR_YAW)
	{
		desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(horizontalVelocity.Y, horizontalVelocity.X));
	}
	else
	{
		desiredYaw = currentRotation.Yaw;
	}


	//desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;
	YawStabilization(a_deltaTime);
	
    ThrustMixer(x_output, y_output, z_output, roll_output, pitch_output);

    // Debug visualization (unchanged)
    if (Debug_DrawDroneWaypoint)
    {
        FVector dronePos = dronePawn->GetActorLocation();
        FVector velocityDirection = desiredNewVelocity.GetSafeNormal() * 200.0f;

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
	if (dronePawn && dronePawn->ImGuiUtil)
	{
		dronePawn->ImGuiUtil->VelocityHud(Thrusts, roll_output, pitch_output, currentRotation,FVector::ZeroVector, currentPosition, FVector::ZeroVector,currentVelocity, x_output, y_output, z_output, a_deltaTime);
	}   
}

void UQuadDroneController::ThrustMixer(float xOutput, float yOutput, float zOutput,
										 float rollOutput, float pitchOutput)
{
	float droneMass = dronePawn->DroneBody->GetMass();
	const float mult = 0.5f;
	
	Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
	Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
	Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
	Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
    
	for (int i = 0; i < Thrusts.Num(); i++)
	{
		Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, 700.0f);
	}
    
	for (int i = 0; i < Thrusts.Num(); i++)
	{
		if (!dronePawn || !dronePawn->Thrusters.IsValidIndex(i))
			continue;
		float force = droneMass * mult * Thrusts[i];
		dronePawn->Thrusters[i]->ApplyForce(force);
	}
}



void UQuadDroneController::YawStabilization(double DeltaTime)
{
	if (!dronePawn || !dronePawn->DroneBody) return;

	// Calculate yaw error
	FRotator CurrentRot = dronePawn->GetActorRotation();
	float CurrentYaw = CurrentRot.Yaw;
	float YawError = FMath::FindDeltaAngleDegrees(CurrentYaw, desiredYaw);
	
	static constexpr float YAW_ERROR_THRESHOLD = 1.0f; 
	if (FMath::Abs(YawError) < YAW_ERROR_THRESHOLD)
	{
		return;
	}
	FFullPIDSet* CurrentSet = PIDMap.Find(EFlightMode::VelocityControl);
	if (!CurrentSet) return;
	float PIDOutput = CurrentSet->YawPID->Calculate(YawError, DeltaTime);

	FVector UpVector = dronePawn->DroneBody->GetUpVector();
	FVector TorqueVector = UpVector * PIDOutput * YawTorqueForce;

	for (UThrusterComponent* Thruster : dronePawn->Thrusters)
	{
		if (Thruster)
		{
			Thruster->ApplyTorque(TorqueVector, true);
		}
	}

	FVector DroneLocation = dronePawn->GetActorLocation();
	DrawDebugDirectionalArrow(
		GetWorld(),
		DroneLocation,
		DroneLocation + UpVector * 100.f,
		50.f,
		FColor::Blue,
		false,
		0.1f,
		0,
		3.f
	);
    
	FVector ForwardVector = dronePawn->DroneBody->GetForwardVector();
	DrawDebugDirectionalArrow(
		GetWorld(),
		DroneLocation,
		DroneLocation + ForwardVector * 100.f,
		50.f,
		FColor::Red,
		false,
		-1.f,
		0,
		3.f
	);

	for(int i = 0; i < dronePawn->Thrusters.Num(); i++)
	{
		FVector MotorPos = dronePawn->Thrusters[i]->GetComponentLocation();
		FString DirText = dronePawn->MotorClockwiseDirections[i] ? TEXT("CW") : TEXT("CCW");
		DrawDebugString(
			GetWorld(),
			MotorPos + FVector(0,0,15),
			FString::Printf(TEXT("M%d\n%s"), i, *DirText),
			nullptr,
			FColor::White,
			0.f,
			true,
			1.2f
		);
	}
	
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
