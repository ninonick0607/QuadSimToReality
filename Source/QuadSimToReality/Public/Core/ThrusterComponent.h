// ThrusterComponent.h

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "ThrusterComponent.generated.h"

/**
 * A custom thruster component that can manually apply force or torque 
 * to the actor’s root physics body.
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UThrusterComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    UThrusterComponent();

    void ApplyForce(float Force);
    void ApplyTorque(const FVector& Torque, bool bIsDegrees = true);

protected:
    virtual void BeginPlay() override;
};
