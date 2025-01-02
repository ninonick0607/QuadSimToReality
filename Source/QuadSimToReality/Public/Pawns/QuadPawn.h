// QuadPawn.h

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PhysicsEngine/PhysicsThrusterComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Controllers/QuadDroneController.h"
#include "Controllers/ZMQController.h"
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

    // Thruster components
    UPROPERTY(VisibleAnywhere, Category = "Thrusters")
    UPhysicsThrusterComponent *ThrusterFL;

    UPROPERTY(VisibleAnywhere, Category = "Thrusters")
    UPhysicsThrusterComponent *ThrusterFR;

    UPROPERTY(VisibleAnywhere, Category = "Thrusters")
    UPhysicsThrusterComponent *ThrusterBL;

    UPROPERTY(VisibleAnywhere, Category = "Thrusters")
    UPhysicsThrusterComponent *ThrusterBR;

    // Rotor meshes
    UPROPERTY(EditAnywhere, Category = "Rotor Meshes")
    UStaticMeshComponent *MotorFL;

    UPROPERTY(EditAnywhere, Category = "Rotor Meshes")
    UStaticMeshComponent *MotorFR;

    UPROPERTY(EditAnywhere, Category = "Rotor Meshes")
    UStaticMeshComponent *MotorBL;

    UPROPERTY(EditAnywhere, Category = "Rotor Meshes")
    UStaticMeshComponent *MotorBR;

    struct Rotor
    {
        UPhysicsThrusterComponent *Thruster;
        UStaticMeshComponent *Mesh;
        float AngularVelocity = 0.0f;

        Rotor(UPhysicsThrusterComponent *InThruster, UStaticMeshComponent *InMesh)
            : Thruster(InThruster), Mesh(InMesh), AngularVelocity(0.0f)
        {
        }

        Rotor()
            : Thruster(nullptr), Mesh(nullptr), AngularVelocity(0.0f)
        {
        }

        void Animate(float DeltaTime)
        {
            // Increased multiplier for more realistic rotor speed
            const float Multiplier = 20.f; // Changed from 1.f to 20.f

            // Reduced inertia to allow for quicker acceleration
            const float Inertia = 0.1f; // Changed from 0.2f to 0.1f

            // Added base rotation speed to maintain minimum rotation even at low thrust
            const float BaseRotationSpeed = 1000.f;

            // Calculate rotor acceleration with base speed
            const float RotorAcceleration = ((Thruster->ThrustStrength * Multiplier + BaseRotationSpeed) - AngularVelocity) / Inertia;

            // Update angular velocity
            AngularVelocity += RotorAcceleration * DeltaTime;

            // Update rotor rotation
            FRotator Rotation = Mesh->GetRelativeRotation();
            Rotation.Yaw += AngularVelocity * DeltaTime;
            Mesh->SetRelativeRotation(Rotation);
        }
    };                                                                          
    EWaypointMode WaypointMode;
    TArray<FVector> ManualWaypoints;
    FVector NewWaypoint;
    Rotor Rotors[4]; // FL, FR, BL, BR

    //--------
    UPROPERTY(VisibleAnywhere)
    TArray<UStaticMeshComponent *> Motors;
    UPROPERTY(VisibleAnywhere)
    TArray<UPhysicsThrusterComponent *> Thrusters;

    UPROPERTY(VisibleAnywhere, Category = "Controller")
    UQuadDroneController* QuadController;
    
    // UPROPERTY(VisibleAnywhere, Category = "ZMQ")
    // UZMQController* ZMQController;
    void SetupFlightMode(EFlightOptions Mode);


    void SwitchCamera() const;
    void ToggleImguiInput();
    virtual void HandleThrustInput(float Value);  
    virtual void HandleYawInput(float Value);     
    virtual void HandlePitchInput(float Value);   
    virtual void HandleRollInput(float Value);
    
    void ActivateDrone(bool bActivate);
    bool IsActive() const { return bIsActive; }

    UQuadDroneController* GetDroneController() const { return QuadController; }

protected:
    // Overridden function called when the game starts or when spawned
    virtual void BeginPlay() override;

private:
    
    // Helper functions
    void UpdateControl(float DeltaTime);
    void InitializeRotors();

    bool bWaypointModeSelected;
    // Input components
    UPROPERTY(VisibleAnywhere)
    UInputComponent *Input_ToggleImguiInput;
    bool bIsActive;

};