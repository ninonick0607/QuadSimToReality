#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Controllers/ZMQController.h"
#include "Core/ThrusterComponent.h"
#include "UI/ImGuiUtil.h"
#include "QuadPawn.generated.h"

#define ACCEPTABLE_DIST 200

// -- Waypoint Mode related types --
// These are used for navigation / waypoint input. If you�re not implementing 
// autonomous waypoint control right now, you could remove these.
enum class EWaypointMode
{
	WaitingForModeSelection,	
	ManualWaypointInput,
	ReadyToStart
};

UCLASS()
class QUADSIMTOREALITY_API AQuadPawn : public APawn
{
	GENERATED_BODY()

public:
	// Constructor
	AQuadPawn();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	
	// --- Drone Components ---
	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* DroneBody;

	// --- Camera Components ---
	UPROPERTY(VisibleAnywhere, Category = "Camera")
	USpringArmComponent* SpringArm;

	UPROPERTY(VisibleAnywhere, Category = "Camera")
	UCameraComponent* Camera;

	UPROPERTY(VisibleAnywhere, Category = "Camera")
	UCameraComponent* CameraFPV;
	
	// --- Thruster Components ---
	UPROPERTY(VisibleAnywhere, Category = "Components")
	TArray<UStaticMeshComponent*> Propellers;
	UPROPERTY(VisibleAnywhere, Category = "Components")
	TArray<UThrusterComponent*> Thrusters;

	// --- Drone Configuration ---
	// Array to specify motor rotation directions.
	UPROPERTY(EditDefaultsOnly, Category = "Drone Configuration")
	TArray<bool> MotorClockwiseDirections = { false, true, true, false };

	// Propeller RPM values (used to visually animate the propellers)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone Components")
	TArray<float> PropellerRPMs;
	
	// --- Controller Components ---
	UPROPERTY(VisibleAnywhere, Category = "Controller")
	class UQuadDroneController* QuadController;

	UPROPERTY()
	class UImGuiUtil* ImGuiUtil;
	
	// --- Identification ---
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	FString DroneID;
	
	UPROPERTY(VisibleAnywhere)
	FString PawnLocalID;

	// --- Helper Functions ---
	void SwitchCamera() const;
	void ToggleImguiInput();
	void ReloadJSONConfig();

	// This helper is used by the controller to get the drone�s mass.
	float GetMass() { return DroneBody->GetMass(); };

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	// Updates control each tick.
	void UpdateControl(float DeltaTime);
	

	UPROPERTY(VisibleAnywhere)
	UInputComponent* Input_ToggleImguiInput;
};