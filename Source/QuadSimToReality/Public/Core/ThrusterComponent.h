// ThrusterComponent.h

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "ThrusterComponent.generated.h"


UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UThrusterComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    UThrusterComponent();

    void ApplyForce(double Force);
    void ApplyTorque(const FVector& Torque, bool bIsDegrees);
    void ApplyTorque(float TorqueValue, bool bIsDegrees = true);

protected:
    virtual void BeginPlay() override;
};
