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
    USkeletalMeshComponent* DroneBody;

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
            // Increased multiplier for more realistic rotor speed
            const float Multiplier = 20.f;  // Changed from 1.f to 20.f

            // Reduced inertia to allow for quicker acceleration
            const float Inertia = 0.1f;     // Changed from 0.2f to 0.1f

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
    TArray<UStaticMeshComponent*> Motors;
    UPROPERTY(VisibleAnywhere)
    TArray<UPhysicsThrusterComponent*> Thrusters;
    
    // Drone controller
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Controller", meta = (AllowPrivateAccess = "true"))
    UQuadDroneController* QuadController;

    void HandleThrustInput(float Value);
    void HandleYawInput(float Value);
    void HandlePitchInput(float Value);
    void HandleRollInput(float Value);
    
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
    UInputComponent* Input_ToggleImguiInput;



};