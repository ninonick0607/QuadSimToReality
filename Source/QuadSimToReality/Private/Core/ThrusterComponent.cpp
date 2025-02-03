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

    // By default, push along local +X.
    const FVector Direction = GetComponentTransform().GetUnitAxis(EAxis::X);
    const FVector ForceVector = Direction * Force;

    // Use this component's location as the point of force application.
    const FVector ForceLocation = GetComponentLocation();

    // Apply force at that location
    RootPrim->AddForceAtLocation(ForceVector, ForceLocation);

    UE_LOG(LogTemp, Warning,
           TEXT("Thruster: Applying Force=%f at %s in Dir=%s"),
           Force, *ForceLocation.ToString(), *Direction.ToString()
    );

#if WITH_EDITOR
    // Debug line
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

    // Decide whether to use degrees or radians
    if (bIsDegrees)
    {
        RootPrim->AddTorqueInDegrees(Torque, NAME_None, true);
    }
    else
    {
        RootPrim->AddTorqueInRadians(Torque, NAME_None, true);
    }

    UE_LOG(LogTemp, Warning,
           TEXT("Thruster: Applying Torque=%s (%s)"),
           *Torque.ToString(),
           bIsDegrees ? TEXT("Degrees") : TEXT("Radians")
    );

#if WITH_EDITOR
    // (Optional) Debug visualization
    FVector StartLocation = GetComponentLocation();
    FVector TorqueDir = Torque.GetSafeNormal(); 
    float Magnitude = Torque.Size();

    // Visual scale factor, so the line isn't huge or too small
    const float DebugScale = 0.02f;

    DrawDebugLine(
        GetWorld(),
        StartLocation,
        StartLocation + (TorqueDir * Magnitude * DebugScale),
        FColor::Red,
        false,
        -1.0f,
        0,
        2.0f
    );
#endif
}
