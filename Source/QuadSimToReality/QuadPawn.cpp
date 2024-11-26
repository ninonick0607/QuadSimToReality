// QuadPawn.cpp

#include "QuadPawn.h"
#include "QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"

#define EPSILON 0.0001f


AQuadPawn::AQuadPawn()
    : DroneBody(nullptr)                           // 1
    , DroneCamMesh(nullptr)                        // 2
    , SpringArm(nullptr)                           // 3
    , Camera(nullptr)                              // 4
    , CameraFPV(nullptr)                           // 5
    , ThrusterFL(nullptr)                          // 6
    , ThrusterFR(nullptr)                          // 7
    , ThrusterBL(nullptr)                          // 8
    , ThrusterBR(nullptr)                          // 9
    , MotorFL(nullptr)                             // 10
    , MotorFR(nullptr)                             // 11
    , MotorBL(nullptr)                             // 12
    , MotorBR(nullptr)                             // 13
    , WaypointMode(EWaypointMode::WaitingForModeSelection)  // 14
    , ManualWaypoints()                            // 15
    , NewWaypoint(FVector::ZeroVector)             // 16
    , Rotors()                                     // 17
    , Motors()                                     // 18
    , Thrusters()                                  // 19
    , QuadController(nullptr)                      // 20
    , bWaypointModeSelected(false)                 // 21
    , Input_ToggleImguiInput(nullptr)
{
    PrimaryActorTick.bCanEverTick = true;

    // Initialize arrays
    Motors.SetNum(4);
    Thrusters.SetNum(4);
    
    const float DroneSize = 12.0f;
    const float ThrusterHeight = 16.0f;
    const FString MotorNames[] = { TEXT("MotorFL"), TEXT("MotorFR"), TEXT("MotorBL"), TEXT("MotorBR") };
    const FString ThrusterNames[] = { TEXT("ThrusterFL"), TEXT("ThrusterFR"), TEXT("ThrusterBL"), TEXT("ThrusterBR") };

    // Initialize components
    DroneBody = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("DroneBody"));
    SetRootComponent(DroneBody);

    DroneBody->SetSimulatePhysics(true);
    DroneBody->SetLinearDamping(1.0f);
    DroneBody->SetAngularDamping(1.0f);

    DroneCamMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CamMesh"));
    DroneCamMesh->SetupAttachment(DroneBody);

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(DroneBody);

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm);

    CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
    CameraFPV->SetupAttachment(DroneBody);

    CameraFPV->SetRelativeLocation(FVector(0, 0, 10));


    const FVector MotorPositions[] = {
        FVector(DroneSize, -DroneSize, ThrusterHeight),   // Front-Left
        FVector(DroneSize, DroneSize, ThrusterHeight),    // Front-Right
        FVector(-DroneSize, -DroneSize, ThrusterHeight),  // Back-Left
        FVector(-DroneSize, DroneSize, ThrusterHeight)    // Back-Right
    };

    // Initialize Rotors
    for (int32 i = 0; i < 4; ++i)
    {
        // Create the motor mesh component
        Motors[i] = CreateDefaultSubobject<UStaticMeshComponent>(*MotorNames[i]);
        Motors[i]->SetupAttachment(DroneBody);
        Motors[i]->SetRelativeLocation(MotorPositions[i]);

        // Optionally set other properties for Motors[i] here

        // Create the thruster component
        Thrusters[i] = CreateDefaultSubobject<UPhysicsThrusterComponent>(*ThrusterNames[i]);
        Thrusters[i]->SetupAttachment(Motors[i]);
        Thrusters[i]->SetRelativeRotation(FRotator(-90.0f, 0.0f, 0.0f));
        Thrusters[i]->bAutoActivate = true;
        Thrusters[i]->ThrustStrength = 0.0f; // Initialize thrust to zero

        // Initialize the Rotor struct
        Rotors[i] = { Thrusters[i], Motors[i] };
    }

    // SpringArm settings
    SpringArm->TargetArmLength = 200.0f;
    SpringArm->SetRelativeRotation(FRotator(-20.0f, 0.0f, 0.0f));
    SpringArm->bDoCollisionTest = false;
    SpringArm->bInheritPitch = false;
    SpringArm->bInheritRoll = false;

    // Automatically possess pawn for testing
    AutoPossessPlayer = EAutoReceiveInput::Player0;
    
    bWaypointModeSelected = false;

}

void AQuadPawn::BeginPlay()
{
    Super::BeginPlay();
    if (!QuadController)
    {
        QuadController = NewObject<UQuadDroneController>(this, TEXT("QuadDroneController"));
        if (QuadController)
        {
            UE_LOG(LogTemp, Error, TEXT("Initialize called with QuadPawn"));
            QuadController->Initialize(this);
        }
    }
    for (auto& rotor : Rotors)
    {
        rotor.Thruster->ThrustStrength = 0.0f;
    }
    
    WaypointMode = EWaypointMode::WaitingForModeSelection;
    NewWaypoint = FVector(0.0f, 0.0f, 0.0f);
    ManualWaypoints.Empty();
   
    for (auto& rotor : Rotors)
    {
        rotor.Thruster->ThrustStrength = 0.0f;
;
    }
    
    QuadController->ResetPID();
    Camera->SetActive(true);
    CameraFPV->SetActive(false);
}

void AQuadPawn::SwitchCamera()
{
    if (CameraFPV->IsActive())
    {
        // enable 3rd person
        CameraFPV->SetActive(false);
        Camera->SetActive(true);
        DroneCamMesh->SetHiddenInGame(false);
    }
    else
    {
        // enable 1st person
        CameraFPV->SetActive(true);
        Camera->SetActive(false);
        DroneCamMesh->SetHiddenInGame(true);
    }
}
void AQuadPawn::ToggleImguiInput()
{
    UGameplayStatics::GetPlayerController(GetWorld(), 0)->ConsoleCommand("ImGui.ToggleInput");
}

void AQuadPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    
    for (auto& rotor : Rotors)
    {
        rotor.Animate(DeltaTime);
    }

    
    // UpdateAnimation(DeltaTime);
    UpdateControl(DeltaTime);
}

void AQuadPawn::HandleThrustInput(float Value)
{
    if (QuadController)
    {
        QuadController->HandleThrustInput(Value);
    }
}

void AQuadPawn::HandleYawInput(float Value)
{
    if (QuadController)
    {
        QuadController->HandleYawInput(Value);
    }
}

void AQuadPawn::HandlePitchInput(float Value)
{
    if (QuadController)
    {
        QuadController->HandlePitchInput(Value);
    }
}

void AQuadPawn::HandleRollInput(float Value)
{
    if (QuadController)
    {
        QuadController->HandleRollInput(Value);
    }
}

void AQuadPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // Bind axis mappings for controller input to wrapper functions
    PlayerInputComponent->BindAxis("Thrust", this, &AQuadPawn::HandleThrustInput);
    PlayerInputComponent->BindAxis("Yaw", this, &AQuadPawn::HandleYawInput);
    PlayerInputComponent->BindAxis("Pitch", this, &AQuadPawn::HandlePitchInput);
    PlayerInputComponent->BindAxis("Roll", this, &AQuadPawn::HandleRollInput);

    PlayerInputComponent->BindAction("ToggleImGui", IE_Pressed, this, &AQuadPawn::ToggleImguiInput);

}

void AQuadPawn::UpdateControl(float DeltaTime)
{
    this->QuadController->Update(DeltaTime);
}


void AQuadPawn::InitializeRotors()
{
    const FString motorNames[] = { TEXT("MotorFL"), TEXT("MotorFR"), TEXT("MotorBL"), TEXT("MotorBR") };
    const FString thrusterNames[] = { TEXT("ThrusterFL"), TEXT("ThrusterFR"), TEXT("ThrusterBL"), TEXT("ThrusterBR") };
    const FString socketNames[] = { TEXT("MotorSocketFL"), TEXT("MotorSocketFR"), TEXT("MotorSocketBL"), TEXT("MotorSocketBR") };

    for (int i = 0; i < 4; ++i)
    {
        // Create and attach motor components
        Motors[i] = CreateDefaultSubobject<UStaticMeshComponent>(*motorNames[i]);
        Motors[i]->AttachToComponent(DroneBody, FAttachmentTransformRules::KeepRelativeTransform);
        Motors[i]->SetupAttachment(DroneBody, *socketNames[i]);

        // Create and attach thruster components under the motor
        Thrusters[i] = CreateDefaultSubobject<UPhysicsThrusterComponent>(*thrusterNames[i]);
        Thrusters[i]->AttachToComponent(Motors[i], FAttachmentTransformRules::KeepRelativeTransform);
        Thrusters[i]->SetRelativeRotation(FRotator(-90.0f, 0.0f, 0.0f));
        Thrusters[i]->bAutoActivate = true;
        
    }
}
