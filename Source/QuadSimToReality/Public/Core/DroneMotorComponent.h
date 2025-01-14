// DroneMotorComponent.h
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "PhysicsEngine/PhysicsThrusterComponent.h"
#include "DroneMotorComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UDroneMotorComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UDroneMotorComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// The physics thruster that provides the force
	UPROPERTY(VisibleAnywhere, Category = "Motor")
	UPhysicsThrusterComponent* Thruster;

	// Getter/Setter for thrust strength
	UFUNCTION(BlueprintCallable, Category = "Motor")
	float GetThrustStrength() const { return Thruster ? Thruster->ThrustStrength : 0.0f; }

	UFUNCTION(BlueprintCallable, Category = "Motor")
	void SetThrustStrength(float NewStrength);

	// Reference to associated mesh for animation
	void SetAssociatedMesh(UStaticMeshComponent* InMesh) { AssociatedMesh = InMesh; }

	// Motor spin animation
	void UpdateRotation(float DeltaTime);

protected:
	// Reference to the visual mesh (owned by QuadPawn)
	UPROPERTY()
	UStaticMeshComponent* AssociatedMesh;

	float AngularVelocity;

	static constexpr float SpinMultiplier = 20.0f;
	static constexpr float SpinInertia = 0.1f;
	static constexpr float BaseRotationSpeed = 1000.0f;
};