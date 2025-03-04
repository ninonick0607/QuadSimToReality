
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
	, desiredForwardVector(FVector(1.0f, 0.0f, 0.0f))
	, desiredAltitude(0.0f)
	, currentFlightMode(EFlightMode::None)
	, desiredNewVelocity(FVector::ZeroVector)
	, initialTakeoff(true)
	, altitudeReached(false)
	, MaxAngularVelocity(180.0)  
	, YawTorqueForce(2.0)       
	, LastYawTorqueApplied(0.0)
	, UpsideDown(false)
{
	const auto& Config = UDroneJSONConfig::Get().Config;
	maxVelocity = Config.FlightParams.MaxVelocity;
	maxAngle = Config.FlightParams.MaxAngle;
	maxPIDOutput = 350.f;//Config.FlightParams.MaxPIDOutput;
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
    VelocitySet.RollPID->SetGains(4.75f, 0.3f, 2.347f);

    VelocitySet.PitchPID = new QuadPIDController();
    VelocitySet.PitchPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.PitchPID->SetGains(4.75f, 0.3f, 2.347f);

    VelocitySet.YawPID = new QuadPIDController();
    VelocitySet.YawPID->SetLimits(-maxPIDOutput, maxPIDOutput);
    VelocitySet.YawPID->SetGains(0.f, 0.f, 0.f);
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


	// TODO: Change to only one game mode please
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
	// Calls the mapped PID set
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
    
    	const float VELOCITY_YAW_THRESHOLD = 20.0f;
    	if (horizontalVelocity.SizeSquared() > VELOCITY_YAW_THRESHOLD)
    	{
    		horizontalVelocity.Normalize();
    		desiredForwardVector = FVector(horizontalVelocity.X, horizontalVelocity.Y, 0.0f).GetSafeNormal();

    		float rawDesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(horizontalVelocity.X, -horizontalVelocity.Y));
    		float normalizedDesiredYaw = FMath::UnwindDegrees(rawDesiredYaw);
    		desiredYaw = normalizedDesiredYaw;
        
    		// Debug info
    		UE_LOG(LogTemp, Display, TEXT("Velocity Direction: X=%.2f, Y=%.2f, DesiredForward: X=%.2f, Y=%.2f"),
				  horizontalVelocity.X, horizontalVelocity.Y, desiredForwardVector.X, desiredForwardVector.Y);
    	}
    
        ThrustMixer(x_output, y_output, z_output, roll_output, pitch_output);
    }
    else
    {
        ApplyManualThrusts();
    }
    
    YawStabilization(a_deltaTime);
	DrawDebugVisuals(horizontalVelocity);


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
                dronePawn->ImGuiUtil->VelocityHud(Thrusts, roll_output, pitch_output, currentRotation,FVector::ZeroVector, currentPosition, FVector::ZeroVector,currentVelocity, x_output, y_output, z_output, a_deltaTime);
            	//FVector currentForwardVector = dronePawn->GetActorForwardVector();
            	//dronePawn->ImGuiUtil->RenderImPlot(Thrusts, desiredForwardVector, currentForwardVector, a_deltaTime);
            }
        }	
    }
}

// ---------------------- Thrust Functions ------------------------

void UQuadDroneController::ThrustMixer(double xOutput, double yOutput, double zOutput,
										 double rollOutput, double pitchOutput)
{
	float droneMass = dronePawn->DroneBody->GetMass();
	const float gravity = 980.0f; 
	const float hoverThrust = (droneMass * gravity) / 4.0f; // Divided among 4 motors
	
	float thrustAdjustmentFactor = 0.8f; 
	float zThrustAdjustment = (zOutput / maxPIDOutput) * hoverThrust * thrustAdjustmentFactor;
    
	float baseThrust = hoverThrust + zThrustAdjustment;
    
	Thrusts[0] = baseThrust - xOutput + yOutput + rollOutput + pitchOutput;
	Thrusts[1] = baseThrust - xOutput - yOutput - rollOutput + pitchOutput;
	Thrusts[2] = baseThrust + xOutput + yOutput + rollOutput - pitchOutput;
	Thrusts[3] = baseThrust + xOutput - yOutput - rollOutput - pitchOutput;

	for (int i = 0; i < Thrusts.Num(); i++)
	{
		Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, 700.0f);
	}
    
	// Apply thrusts to motors
	for (int i = 0; i < Thrusts.Num(); i++)
	{
		if (!dronePawn || !dronePawn->Thrusters.IsValidIndex(i))
			continue;
		double force = droneMass * 0.5f * Thrusts[i];
		dronePawn->Thrusters[i]->ApplyForce(force);
	}
}



void UQuadDroneController::YawStabilization(double DeltaTime)
{
    // Early exit if the drone pawn or its physics body is missing.
    if (!dronePawn || !dronePawn->DroneBody) return;

    // Get the drone's forward vector and flatten it to the XY plane (ignore Z).
    FVector CurrentForwardVector = dronePawn->GetActorForwardVector();
    CurrentForwardVector.Z = 0.0f;  // Remove Z component to ensure it's 2D.
    CurrentForwardVector.Normalize();  // Normalize to get unit vector.

    // Normalize the desired forward vector (assumed to be set elsewhere).
    FVector DesiredForwardVector = desiredForwardVector.GetSafeNormal();

    // Compute the angle between the current and desired forward vectors using the dot product.
    float DotProduct = FVector::DotProduct(CurrentForwardVector, DesiredForwardVector);
    FVector CrossProduct = FVector::CrossProduct(CurrentForwardVector, DesiredForwardVector);  // For direction sense.

    // Convert angle from radians to degrees and clamp to avoid precision issues.
    float VectorError = FMath::Acos(FMath::Clamp(DotProduct, -1.0f, 1.0f));
    VectorError = FMath::RadiansToDegrees(VectorError);

    // Determine the sign (clockwise or counter-clockwise) using the cross product and up vector.
    FVector UpVector = dronePawn->DroneBody->GetUpVector();
    float DirectionSign = FMath::Sign(FVector::DotProduct(CrossProduct, UpVector));
    VectorError *= DirectionSign;  // Apply the sign to the angle error.

    // Skip small errors within threshold — prevents overcorrection when close to target.
    static constexpr float YAW_ERROR_THRESHOLD = 1.0f;
    if (FMath::Abs(VectorError) < YAW_ERROR_THRESHOLD)
    {
        return;
    }

    // Retrieve the current PID controller settings for the VelocityControl flight mode.
    FFullPIDSet* CurrentSet = PIDMap.Find(EFlightMode::VelocityControl);
    if (!CurrentSet) return;

    // Get the drone's current angular velocity around the Z-axis (current yaw rate).
    FVector CurrentAngularVelocity = dronePawn->DroneBody->GetPhysicsAngularVelocityInDegrees();
    float CurrentYawRate = CurrentAngularVelocity.Z;

    // Compute the desired yaw torque using the PID controller.
    float PIDOutput = CurrentSet->YawPID->Calculate(VectorError, DeltaTime);

    // Clamp the output torque to prevent excessive forces.
    float MaxYawTorque = 2.0f;
    PIDOutput = FMath::Clamp(PIDOutput, -MaxYawTorque, MaxYawTorque);

    // Apply a simple damping term to counteract excess rotational velocity.
    float YawDamping = -CurrentYawRate * 0.05f;  
    float FinalYawTorque = PIDOutput + YawDamping;  

    // Convert scalar torque into a world-space torque vector.
    FVector TorqueVector = UpVector * FinalYawTorque * YawTorqueForce;

    // Save the applied yaw torque for potential debugging or telemetry.
    LastYawTorqueApplied = FinalYawTorque * YawTorqueForce;

    // Apply the computed torque to each thruster that supports torque input.
    for (UThrusterComponent* Thruster : dronePawn->Thrusters)
    {
        if (Thruster)
        {
            Thruster->ApplyTorque(TorqueVector, true);
        }
    }

    if (Debug_DrawDroneWaypoint)
    {
        FVector dronePos = dronePawn->GetActorLocation();
        DrawDebugLine(
            dronePawn->GetWorld(),
            dronePos,
            dronePos + DesiredForwardVector * 200.0f,  // Desired direction.
            FColor::Cyan,
            false, -1.0f, 0, 3.0f
        );
        DrawDebugLine(
            dronePawn->GetWorld(),
            dronePos,
            dronePos + CurrentForwardVector * 200.0f,  // Current direction.
            FColor::Orange,
            false, -1.0f, 0, 3.0f
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
		desiredYaw = 0.0f;
		desiredForwardVector = FVector(1.0f, 0.0f, 0.0f);  // Reset to forward direction
		initialTakeoff = true;
		altitudeReached = false;
	}
}


// ---------------------- Helper Functions ------------------------

void UQuadDroneController::DrawDebugVisuals(const FVector& horizontalVelocity) const
{
	if (!dronePawn || !dronePawn->DroneBody) return;
	FVector dronePos = dronePawn->GetActorLocation();
	const float scaleXYZ = .5f; 
	const float scaleHorizontal = 100.0f;

	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(desiredNewVelocity.X, 0, 0) * scaleXYZ, FColor::Red, false, -1.0f, 0, 2.0f);
	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(0, desiredNewVelocity.Y, 0) * scaleXYZ, FColor::Green, false, -1.0f, 0, 2.0f);
	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(0, 0, desiredNewVelocity.Z) * scaleXYZ, FColor::Blue, false, -1.0f, 0, 2.0f);
	DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + FVector(horizontalVelocity.X, horizontalVelocity.Y, horizontalVelocity.Z) * scaleHorizontal, FColor::Yellow, false, -1.0f, 0, 3.0f);
	
	FVector ForwardVector = dronePawn->DroneBody->GetForwardVector();
	DrawDebugDirectionalArrow(GetWorld(), dronePos, dronePos + ForwardVector * 100.f, 50.f, FColor::Red, false, -1.f, 0, 3.f);

	if (Debug_DrawDroneWaypoint) 
	{ 
		FVector velocityDirection = desiredNewVelocity.GetSafeNormal() * 200.0f; 
		DrawDebugLine(dronePawn->GetWorld(), dronePos, dronePos + velocityDirection, FColor::Magenta, false, -1.0f, 0, 3.0f); 
	}

	for (int i = 0; i < dronePawn->Thrusters.Num(); i++) 
	{ 
		FVector MotorPos = dronePawn->Thrusters[i]->GetComponentLocation(); 
		FString DirText = dronePawn->MotorClockwiseDirections[i] ? TEXT("CW") : TEXT("CCW"); 
		DrawDebugString(GetWorld(), MotorPos + FVector(0, 0, 15), FString::Printf(TEXT("M%d\n%s"), i, *DirText), nullptr, FColor::White, 0.0f, true, 1.2f); 
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

