// DroneMotorComponent.cpp
#include "Core/DroneMotorComponent.h"

UDroneMotorComponent::UDroneMotorComponent()
	: AngularVelocity(0.0f)
	, AssociatedMesh(nullptr)
{
	PrimaryComponentTick.bCanEverTick = true;

	// Create and setup the thruster
	Thruster = CreateDefaultSubobject<UPhysicsThrusterComponent>(TEXT("Thruster"));
	Thruster->SetupAttachment(this);
	Thruster->SetRelativeRotation(FRotator(-90.0f, 0.0f, 0.0f));
	Thruster->bAutoActivate = true;
	Thruster->ThrustStrength = 0.0f;
}

void UDroneMotorComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UDroneMotorComponent::SetThrustStrength(float NewStrength)
{
	if (Thruster)
	{
		Thruster->ThrustStrength = NewStrength;
	}
}

void UDroneMotorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	UpdateRotation(DeltaTime);
}

void UDroneMotorComponent::UpdateRotation(float DeltaTime)
{
	if (!AssociatedMesh) return;

	// Calculate rotor acceleration with base speed
	const float RotorAcceleration = ((Thruster->ThrustStrength * SpinMultiplier + BaseRotationSpeed) - AngularVelocity) / SpinInertia;

	// Update angular velocity
	AngularVelocity += RotorAcceleration * DeltaTime;

	// Update rotor rotation
	FRotator NewRotation = AssociatedMesh->GetRelativeRotation();
	NewRotation.Yaw += AngularVelocity * DeltaTime;
	AssociatedMesh->SetRelativeRotation(NewRotation);
}