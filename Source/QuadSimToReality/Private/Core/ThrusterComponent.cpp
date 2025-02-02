#include "Core/ThrusterComponent.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"

UThrusterComponent::UThrusterComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UThrusterComponent::BeginPlay()
{
	Super::BeginPlay();

}

void UThrusterComponent::ApplyForce(float Force)
{
	// The actor owning this component
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: No owner, cannot apply force."));
		return;
	}

	// The root must be a simulating physics body
	UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
	if (!RootPrim || !RootPrim->IsSimulatingPhysics())
	{
		UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: Root is not simulating physics!"));
		return;
	}

	// Determine the direction. By default, we push along local +X.
	const FVector Direction = GetComponentTransform().GetUnitAxis(EAxis::X);
	const FVector ForceVector = Direction * Force;

	// Use this component's location as the point of force application.
	const FVector ForceLocation = GetComponentLocation();
    
	RootPrim->AddForceAtLocation(ForceVector, ForceLocation);

	UE_LOG(LogTemp, Warning, TEXT("Thruster: Applying Force=%f at %s in Dir=%s"),
		   Force, *ForceLocation.ToString(), *Direction.ToString());

#if WITH_EDITOR
	// Debug draw
	if (Force > 0.01f)
	{
		DrawDebugLine(
			GetWorld(),
			ForceLocation,
			ForceLocation + (Direction * Force * 0.01f),
			FColor::Blue,
			false, // bPersistentLines
			-1.0f, // Lifetime
			0,
			2.0f
		);
	}
#endif
}