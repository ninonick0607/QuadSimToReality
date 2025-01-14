#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Camera/CameraComponent.h"
#include "Controllers/QuadDroneController.h"  
#include "GameFramework/SpringArmComponent.h"
#include "Core/DroneMotorComponent.h"
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
    AQuadPawn();
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

    // Drone body components
    UPROPERTY(EditAnywhere, Category = "Drone Components")
    UStaticMeshComponent* DroneBody;

    UPROPERTY(EditAnywhere, Category = "Drone Components")
    UStaticMeshComponent* DroneCamMesh;

    // Camera components
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USpringArmComponent* SpringArm;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* Camera;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent* CameraFPV;

    // Motor meshes (attached to sockets)
    UPROPERTY(EditAnywhere, Category = "Motor Meshes")
    UStaticMeshComponent* MotorMeshFL;

    UPROPERTY(EditAnywhere, Category = "Motor Meshes")
    UStaticMeshComponent* MotorMeshFR;

    UPROPERTY(EditAnywhere, Category = "Motor Meshes")
    UStaticMeshComponent* MotorMeshBL;

    UPROPERTY(EditAnywhere, Category = "Motor Meshes")
    UStaticMeshComponent* MotorMeshBR;

    // Motor components (attached to meshes)
    UPROPERTY(VisibleAnywhere, Category = "Motors")
    UDroneMotorComponent* ThrusterFL;

    UPROPERTY(VisibleAnywhere, Category = "Motors")
    UDroneMotorComponent* ThrusterFR;

    UPROPERTY(VisibleAnywhere, Category = "Motors")
    UDroneMotorComponent* ThrusterBL;

    UPROPERTY(VisibleAnywhere, Category = "Motors")
    UDroneMotorComponent* ThrusterBR;

    // Arrays for easy access
    UPROPERTY()
    TArray<UDroneMotorComponent*> Thrusters;

    UPROPERTY()
    TArray<UStaticMeshComponent*> MotorMeshes;

    // Waypoint system
    EWaypointMode WaypointMode;
    TArray<FVector> ManualWaypoints;
    FVector NewWaypoint;

    // Controller
    UPROPERTY(VisibleAnywhere, Category = "Controller")
    UQuadDroneController* QuadController;

    // Helper functions
    void LockToOnlyPitch();
    void UnlockAllMovement();
    void SwitchCamera() const;
    void ToggleImguiInput();

    // Input handlers
    void HandleThrustInput(float Value);
    void HandleYawInput(float Value);
    void HandlePitchInput(float Value);
    void HandleRollInput(float Value);

protected:
    virtual void BeginPlay() override;

private:
    void UpdateControl(float DeltaTime);
    void InitializeMotors();

    bool bWaypointModeSelected;
    
    UPROPERTY(VisibleAnywhere)
    UInputComponent* Input_ToggleImguiInput;
};