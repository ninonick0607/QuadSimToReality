
// QuadDroneController.cpp

#include "Controllers/QuadDroneController.h"
#include "Pawns/QuadPawn.h"
#include "DrawDebugHelpers.h"
#include "Core/DroneGlobalState.h"
#include "UI/ImGuiUtil.h"
#include "Core/DroneJSONConfig.h"
#include "Core/DroneManager.h"
#include "Kismet/GameplayStatics.h"
#include "Math/UnrealMathUtility.h"


// ---------------------- Constructor ------------------------


UQuadDroneController::UQuadDroneController(const FObjectInitializer& ObjectInitializer)
	: dronePawn(nullptr)
	, Thrusts({0, 0, 0, 0})
	, desiredYaw(0.f)
	, desiredAltitude(0.0f)
	, currentFlightMode(EFlightMode::None)
	, desiredNewVelocity(FVector::ZeroVector)
	, initialTakeoff(true)
	, altitudeReached(false)
	, Debug_DrawDroneCollisionSphere(true)
	, Debug_DrawDroneWaypoint(true)
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
    VelocitySet.RollPID->SetGains(11.0f, 6.0f, 3.3f);

    VelocitySet.PitchPID = new QuadPIDController();
    VelocitySet.PitchPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.PitchPID->SetGains(11.0f, 6.0f, 3.3f);

    VelocitySet.YawPID = new QuadPIDController();
    VelocitySet.YawPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.YawPID->SetGains(1.8f, 0.4f, .7f);
	PIDMap.Add(EFlightMode::VelocityControl, MoveTemp(VelocitySet));

	
	DroneGlobalState::Get().BindController(this);
}

UQuadDroneController::~UQuadDroneController()
{
	DroneGlobalState::Get().UnbindController();
}

// ---------------------- Initialization ------------------------

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
    UpdatePropellerRPMs();

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
    
    SafetyReset();
    
    double x_output = 0.f, y_output = 0.f, z_output = 0.f;
    double roll_output = 0.f, pitch_output = 0.f;
    
    if (!bManualThrustMode)
    {

		FVector velocityError = desiredNewVelocity - currentVelocity;
        x_output = CurrentSet->XPID->Calculate(velocityError.X, a_deltaTime);
        y_output = CurrentSet->YPID->Calculate(velocityError.Y, a_deltaTime);
        z_output = CurrentSet->ZPID->Calculate(velocityError.Z, a_deltaTime);
    
        float roll_error = -currentRotation.Roll;
        float pitch_error = -currentRotation.Pitch;
        roll_output = CurrentSet->RollPID->Calculate(roll_error, a_deltaTime);
        pitch_output = CurrentSet->PitchPID->Calculate(pitch_error, a_deltaTime);
    
        // Yaw control, etc.
		if (horizontalVelocity.SizeSquared() > 10.0f) // Adjust threshold as needed
		{
			horizontalVelocity.Normalize();
			desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(horizontalVelocity.Y, horizontalVelocity.X));
		}
    
        ThrustMixer(x_output, y_output, z_output, roll_output, pitch_output);
    }
    else
    {
        ApplyManualThrusts();
    }
    
	YawStabilization(a_deltaTime);

    // Debug visualization, etc.
    if (Debug_DrawDroneWaypoint)
    {
        FVector dronePos = dronePawn->GetActorLocation();
        FVector velocityDirection = desiredNewVelocity.GetSafeNormal() * 200.0f;
        DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + velocityDirection, FColor::Yellow, false, -1.0f, 0, 2.0f);
    }
    
    if (dronePawn && dronePawn->ImGuiUtil)
    {
        ADroneManager* Manager = Cast<ADroneManager>(UGameplayStatics::GetActorOfClass(dronePawn->GetWorld(), ADroneManager::StaticClass()));
        if (Manager)
        {
            TArray<AQuadPawn*> DroneList = Manager->GetDroneList();
            int32 idx = Manager->SelectedDroneIndex;
            AQuadPawn* selectedPawn = (DroneList.IsValidIndex(idx)) ? DroneList[idx] : nullptr;
            if (dronePawn == selectedPawn)
            {
                dronePawn->ImGuiUtil->VelocityHud(Thrusts, roll_output, pitch_output, currentRotation,
                                                    FVector::ZeroVector, currentPosition, FVector::ZeroVector,
                                                    currentVelocity, x_output, y_output, z_output, a_deltaTime);

				dronePawn->ImGuiUtil->RenderImPlot(Thrusts, a_deltaTime);
			}
        }
    }
}

// ---------------------- Thrust Functions ------------------------

void UQuadDroneController::ThrustMixer(double xOutput, double yOutput, double zOutput,
										 double rollOutput, double pitchOutput)
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
		double force = droneMass * mult * Thrusts[i];
		dronePawn->Thrusters[i]->ApplyForce(force);
	}
}



void UQuadDroneController::YawStabilization(double DeltaTime)
{
	if (!dronePawn || !dronePawn->DroneBody) return;

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

	FVector CurrentAngularVelocity = dronePawn->DroneBody->GetPhysicsAngularVelocityInDegrees();
	float CurrentYawRate = CurrentAngularVelocity.Z; 

	float PIDOutput = CurrentSet->YawPID->Calculate(YawError, DeltaTime);

	float MaxYawTorque = 2.0f; 
	PIDOutput = FMath::Clamp(PIDOutput, -MaxYawTorque, MaxYawTorque);


	float YawDamping = -CurrentYawRate * 0.05f; 

	float FinalYawTorque = PIDOutput + YawDamping;

	FVector UpVector = dronePawn->DroneBody->GetUpVector();
	FVector TorqueVector = UpVector * FinalYawTorque * YawTorqueForce;

	LastYawTorqueApplied = FinalYawTorque * YawTorqueForce;

	for (UThrusterComponent* Thruster : dronePawn->Thrusters)
	{
		if (Thruster)
		{
			Thruster->ApplyTorque(TorqueVector, true);
		}
	}

	// Debug visualizatio
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

	// Display yaw debugging info
	DrawDebugString(
		GetWorld(),
		DroneLocation + FVector(0, 0, 150),
		FString::Printf(TEXT("Yaw Error: %.2f°\nTorque: %.2f"), YawError, FinalYawTorque),
		nullptr,
		FColor::Yellow,
		0.0f,
		true,
		1.0f
	);

	for (int i = 0; i < dronePawn->Thrusters.Num(); i++)
	{
		FVector MotorPos = dronePawn->Thrusters[i]->GetComponentLocation();
		FString DirText = dronePawn->MotorClockwiseDirections[i] ? TEXT("CW") : TEXT("CCW");
		DrawDebugString(
			GetWorld(),
			MotorPos + FVector(0, 0, 15),
			FString::Printf(TEXT("M%d\n%s"), i, *DirText),
			nullptr,
			FColor::White,
			0.0f,
			true,
			1.2f
		);
	}
}



// ---------------------- Reset Functions ------------------------

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


// ---------------------- Helper Functions ------------------------


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

void UQuadDroneController::SafetyReset()
{

	FRotator currentRotation = dronePawn->GetActorRotation();
	FVector currentPosition = dronePawn->GetActorLocation();

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
}

void UQuadDroneController::ApplyManualThrusts()
{
	if (!dronePawn)
		return;
    
	float droneMass = dronePawn->DroneBody->GetMass();
	const float mult = 0.5f; // same multiplier as before
	// Iterate over your thrust array and apply the user-defined thrust values
	for (int i = 0; i < Thrusts.Num(); i++)
	{
		Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, 700.0f);
		if (!dronePawn->Thrusters.IsValidIndex(i))
			continue;
		float force = droneMass * mult * Thrusts[i];
		dronePawn->Thrusters[i]->ApplyForce(force);
	}
}

void UQuadDroneController::UpdatePropellerRPMs()
{

	for (int i = 0; i < Thrusts.Num(); i++)
	{
		// Calculate RPM from thrust
		float RPM = FMath::Sqrt(Thrusts[i]/T_k);

		if (dronePawn)
		{
			dronePawn->SetPropellerRPM(i, RPM);
		}
	}
}


// ------------ Setter and Getter -------------------
void UQuadDroneController::SetDesiredVelocity(const FVector& NewVelocity)
{
	desiredNewVelocity = NewVelocity;
	UE_LOG(LogTemp, Display, TEXT("[QuadDroneController] SetDesiredVelocity called: X=%.2f, Y=%.2f, Z=%.2f"),
			NewVelocity.X, NewVelocity.Y, NewVelocity.Z);
}

void UQuadDroneController::SetFlightMode(EFlightMode NewMode)
{
	currentFlightMode = NewMode;
}

EFlightMode UQuadDroneController::GetFlightMode() const
{
	return currentFlightMode;
}


void UQuadDroneController::SetManualThrustMode(bool bEnable)
{
	bManualThrustMode = bEnable;
	if (bManualThrustMode)
	{
		UE_LOG(LogTemp, Display, TEXT("Manual Thrust Mode ENABLED"));
	}
	else
	{
		UE_LOG(LogTemp, Display, TEXT("Manual Thrust Mode DISABLED"));
	}
}

