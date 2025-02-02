#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "ThrusterComponent.generated.h"

/**
 * A custom thruster component that manually applies force at the component's location,
 * acting on the actor's root physics body.
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UThrusterComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UThrusterComponent();

	/** Applies force along the component?s local +X axis (unless oriented differently). */
	void ApplyForce(float Force);

protected:
	virtual void BeginPlay() override;
};