// QuadPawn.h

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PhysicsEngine/PhysicsThrusterComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "QuadDroneController.h"
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

    // Overridden functions
    
    virtual void Tick(float DeltaTime) override;
    void ToggleImguiInput();
    void SwitchCamera();

    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

    // Drone components
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

    // Thruster components
    UPROPERTY(VisibleAnywhere, Category = "Thrusters")
    UPhysicsThrusterComponent* ThrusterFL;

    UPROPERTY(VisibleAnywhere, Category = "Thrusters")
    UPhysicsThrusterComponent* ThrusterFR;

    UPROPERTY(VisibleAnywhere, Category = "Thrusters")
    UPhysicsThrusterComponent* ThrusterBL;

    UPROPERTY(VisibleAnywhere, Category = "Thrusters")
    UPhysicsThrusterComponent* ThrusterBR;

    // Rotor meshes
    UPROPERTY(EditAnywhere, Category = "Rotor Meshes")
    UStaticMeshComponent* MotorFL;

    UPROPERTY(EditAnywhere, Category = "Rotor Meshes")
    UStaticMeshComponent* MotorFR;

    UPROPERTY(EditAnywhere, Category = "Rotor Meshes")
    UStaticMeshComponent* MotorBL;

    UPROPERTY(EditAnywhere, Category = "Rotor Meshes")
    UStaticMeshComponent* MotorBR;
    
    struct Rotor
    {
        UPhysicsThrusterComponent* Thruster;
        UStaticMeshComponent* Mesh;
        float AngularVelocity = 0.0f;

        Rotor(UPhysicsThrusterComponent* InThruster, UStaticMeshComponent* InMesh)
            : Thruster(InThruster), Mesh(InMesh), AngularVelocity(0.0f)
        {
        }

        Rotor()
            : Thruster(nullptr), Mesh(nullptr), AngularVelocity(0.0f)
        {
        }

        void Animate(float DeltaTime)
        {
            const float Multiplier = 1.f;
            const float Inertia = 0.2f;
            const float RotorAcceleration = (Thruster->ThrustStrength * Multiplier - AngularVelocity) / Inertia;
            AngularVelocity += RotorAcceleration * DeltaTime;
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
    TArray<UStaticMeshComponent*> Motors;
    UPROPERTY(VisibleAnywhere)
    TArray<UPhysicsThrusterComponent*> Thrusters;
    
    // Drone controller
    QuadDroneController* quadController;
    void HandleThrustInput(float Value);
    void HandleYawInput(float Value);
    void HandlePitchInput(float Value);
    void HandleRollInput(float Value);
    
protected:
    // Overridden function called when the game starts or when spawned
    virtual void BeginPlay() override;

private:


    // Function declarations
    void RenderWaypointModeSelection();
    void RenderManualWaypointInput();
    
    // Helper functions
    //void UpdateAnimation(float DeltaTime);
    void UpdateControl(float DeltaTime);
    void InitializeRotors();
    void InitializeWaypointMode();

    bool bWaypointModeSelected;
    // Input components
    UInputComponent* Input_ToggleImguiInput;



};