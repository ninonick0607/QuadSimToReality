// ThrusterComponent.cpp

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
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: No owner, cannot apply force."));
        return;
    }

    UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
    if (!RootPrim || !RootPrim->IsSimulatingPhysics())
    {
        UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: Root is not simulating physics!"));
        return;
    }

    const FVector Direction = GetComponentTransform().GetUnitAxis(EAxis::X);
    const FVector ForceVector = Direction * Force;

    const FVector ForceLocation = GetComponentLocation();
    RootPrim->AddForceAtLocation(ForceVector, ForceLocation);

    UE_LOG(LogTemp, Warning,
           TEXT("Thruster: Applying Force=%f at %s in Dir=%s"),
           Force, *ForceLocation.ToString(), *Direction.ToString()
    );

#if WITH_EDITOR
    if (Force > 0.01f)
    {
        DrawDebugLine(GetWorld(),
            ForceLocation,
            ForceLocation + (Direction * Force * 0.01f),
            FColor::Blue,
            false, 
            -1.0f, 
            0,
            2.0f
        );
    }
#endif
}
void UThrusterComponent::ApplyTorque(const FVector& Torque, bool bIsDegrees /*= true*/)
{
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: No owner, cannot apply torque."));
        return;
    }

    UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
    if (!RootPrim || !RootPrim->IsSimulatingPhysics())
    {
        UE_LOG(LogTemp, Warning, TEXT("ThrusterComponent: Root is not simulating physics!"));
        return;
    }
    RootPrim->AddTorqueInDegrees(Torque, NAME_None, true);

    UE_LOG(LogTemp, Warning,
           TEXT("Thruster: Applying Torque=%s (%s)"),
           *Torque.ToString(),
           bIsDegrees ? TEXT("Degrees") : TEXT("Radians")
    );
    
}

void UThrusterComponent::ApplyTorque(float TorqueValue, bool bIsDegrees)
{
    FVector TorqueVector(0.0f, 0.0f, TorqueValue);
    ApplyTorque(TorqueVector, bIsDegrees);
}
