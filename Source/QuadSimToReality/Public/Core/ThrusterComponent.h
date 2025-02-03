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

    /** Applies force along the component’s local +X axis (unless oriented differently). */
    void ApplyForce(float Force);

    /**
     * Applies a torque to the actor’s root physics body.
     * @param Torque A vector describing the torque about each axis.
     * @param bIsDegrees If true, uses AddTorqueInDegrees; otherwise uses AddTorqueInRadians.
     */
    void ApplyTorque(const FVector& Torque, bool bIsDegrees = true);

protected:
    virtual void BeginPlay() override;
};
