// QuadPawn.h

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Controllers/ZMQController.h"
#include "Core/ThrusterComponent.h" 
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

    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(UInputComponent *PlayerInputComponent) override;

    // Drone components
    UPROPERTY(EditAnywhere, Category = "Drone Components")
    UStaticMeshComponent *DroneBody;

    UPROPERTY(EditAnywhere, Category = "Drone Components")
    UStaticMeshComponent *DroneCamMesh;

    // Camera components
    UPROPERTY(VisibleAnywhere, Category = "Camera")
    USpringArmComponent *SpringArm;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent *Camera;

    UPROPERTY(VisibleAnywhere, Category = "Camera")
    UCameraComponent *CameraFPV;
    
    EWaypointMode WaypointMode;
    TArray<FVector> ManualWaypoints;
    FVector NewWaypoint;
   
    // Thruster and Meshes
    UPROPERTY(VisibleAnywhere, Category = "Components")
    TArray<UStaticMeshComponent*> Propellers;
    UPROPERTY(VisibleAnywhere, Category = "Components")
    TArray<UThrusterComponent*> Thrusters;

    // Controllers
    UPROPERTY(VisibleAnywhere, Category = "Controller")
    UQuadDroneController* QuadController;
    
    UPROPERTY(VisibleAnywhere, Category = "ZMQ")
    UZMQController* ZMQController;
    
    void SwitchCamera() const;
    void ToggleImguiInput();
    float GetMass(){return DroneBody->GetMass();};

protected:
    virtual void BeginPlay() override;

private:
    // Helper functions
    void UpdateControl(float DeltaTime);

    bool bWaypointModeSelected;
    // Input components
    UPROPERTY(VisibleAnywhere)
    UInputComponent *Input_ToggleImguiInput;
};