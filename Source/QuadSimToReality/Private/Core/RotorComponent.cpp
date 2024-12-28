// RotorComponent.cpp

#include "Core/RotorComponent.h"
#include "Components/StaticMeshComponent.h"
#include "PhysicsEngine/PhysicsThrusterComponent.h"
#include "Engine/World.h"

URotorComponent::URotorComponent()
{
	PrimaryComponentTick.bCanEverTick = false; // We'll drive it from Pawn Tick manually
	AngularVelocity = 0.f;
}

void URotorComponent::OnRegister()
{
	Super::OnRegister();
	// You could do additional setup here if needed
}

void URotorComponent::InitializeRotor(
	const FName& InMotorName,
	const FName& InThrusterName,
	const FName& SocketName,
	class USkeletalMeshComponent* ParentSkeletalMesh
)
{
	if (!ParentSkeletalMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("URotorComponent::InitializeRotor - Invalid ParentSkeletalMesh!"));
		return;
	}

	// Attach this RotorComponent to the DroneBody (the SkeletalMesh)
	this->SetupAttachment(ParentSkeletalMesh, SocketName);

	// Create the motor mesh subobject
	MotorMesh = NewObject<UStaticMeshComponent>(this, InMotorName);
	if (MotorMesh)
	{
		MotorMesh->SetupAttachment(this); // Attach motor mesh to this RotorComponent
		MotorMesh->RegisterComponent();
	}

	// Create the thruster subobject
	Thruster = NewObject<UPhysicsThrusterComponent>(this, InThrusterName);
	if (Thruster)
	{
		Thruster->SetupAttachment(MotorMesh); // Attach thruster to the motor mesh
		Thruster->SetRelativeRotation(FRotator(-90.0f, 0.0f, 0.0f));
		Thruster->bAutoActivate = true;
		Thruster->RegisterComponent();
	}
}

void URotorComponent::AnimateRotor(float DeltaTime)
{
	if (!MotorMesh || !Thruster)
		return;

	// --- The same logic that used to live in Rotor::Animate ---
	const float Multiplier = 20.f;
	const float Inertia = 0.1f;
	const float BaseRotationSpeed = 1000.f;

	// ThrustStrength from the Thruster
	const float CurrentThrust = Thruster->ThrustStrength;
	const float RotorAcceleration = ((CurrentThrust * Multiplier + BaseRotationSpeed) - AngularVelocity) / Inertia;

	AngularVelocity += RotorAcceleration * DeltaTime;

	// Update rotor rotation
	FRotator Rotation = MotorMesh->GetRelativeRotation();
	Rotation.Yaw += AngularVelocity * DeltaTime;
	MotorMesh->SetRelativeRotation(Rotation);
}
