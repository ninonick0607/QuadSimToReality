#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Controllers/ZMQController.h"
#include "Core/ThrusterComponent.h"
#include "UI/ImGuiUtil.h"
#include "Utility/NavigationComponent.h"	
#include "Components/PrimitiveComponent.h" 
#include "QuadPawn.generated.h"

#define ACCEPTABLE_DIST 200

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
	
	EWaypointMode WaypointMode;
	TArray<FVector> ManualWaypoints;
	FVector NewWaypoint;

	
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

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UImGuiUtil> ImGuiUtil;


	UPROPERTY(VisibleAnywhere, Category = "Controller")
	class UQuadDroneController* QuadController;
	
	// --- Identification ---
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	FString DroneID;
	
	UPROPERTY(VisibleAnywhere)
	FString PawnLocalID;

	// --- Helper Functions ---
	void SwitchCamera() const;
	void ToggleImguiInput();
	void ReloadJSONConfig();

	float GetMass() { return DroneBody->GetMass(); };

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Collision")
	bool bHasCollidedWithObstacle; 

	UFUNCTION(BlueprintPure, Category = "Collision")
	bool HasCollided() const { return bHasCollidedWithObstacle; }

	UFUNCTION(BlueprintCallable, Category = "Collision")
	void ResetCollisionStatus();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation")
	UNavigationComponent* NavigationComponent;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	UFUNCTION() 
	void OnDroneHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

private:
	// Updates control each tick.
	void UpdateControl(float DeltaTime);
	
	bool bWaypointModeSelected;

	UPROPERTY(VisibleAnywhere)
	UInputComponent* Input_ToggleImguiInput;
};
