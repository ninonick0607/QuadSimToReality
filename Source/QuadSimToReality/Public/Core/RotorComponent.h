// RotorComponent.h

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "PhysicsEngine/PhysicsThrusterComponent.h"
#include "Components/StaticMeshComponent.h"
#include "RotorComponent.generated.h"

/**
 * URotorComponent
 * 
 * Responsible for creating and managing a single rotor on the drone:
 *   - A motor mesh (visual spinning prop)
 *   - A thruster component (for actual propulsion)
 *   - Rotor animation (yaw rotation)
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API URotorComponent : public USceneComponent
{
	GENERATED_BODY()
	
public:
	/** Constructor */
	URotorComponent();

	// Called after the component is created (or when placed in-editor)
	virtual void OnRegister() override;

	/**
	 * Initialize the rotor by attaching a static mesh (motor) and a physics thruster
	 * @param InMotorName  Name to be used for the motor subobject
	 * @param InThrusterName  Name to be used for the thruster subobject
	 * @param SocketName  Name of the socket on the parent SkeletalMesh (DroneBody) to attach to
	 * @param ParentSkeletalMesh  The SkeletalMeshComponent on the pawn that will serve as this rotor's parent
	 */
	void InitializeRotor(
		const FName& InMotorName,
		const FName& InThrusterName,
		const FName& SocketName,
		class USkeletalMeshComponent* ParentSkeletalMesh
	);

	/**
	 * Called every frame from the Pawn's Tick or wherever is appropriate
	 * @param DeltaTime 
	 */
	void AnimateRotor(float DeltaTime);

	/** Returns the thruster so external code can set ThrustStrength. */
	UPhysicsThrusterComponent* GetThruster() const { return Thruster; }

	/** Returns the static mesh of the rotor (motor). */
	UStaticMeshComponent* GetMotorMesh() const { return MotorMesh; }

private:
	/** Mesh that visually represents the rotor blades */
	UPROPERTY()
	UStaticMeshComponent* MotorMesh;

	/** Physics thruster for applying force/lift */
	UPROPERTY()
	UPhysicsThrusterComponent* Thruster;

	/** Current angular velocity (for spinning the rotor mesh) */
	float AngularVelocity;
};
