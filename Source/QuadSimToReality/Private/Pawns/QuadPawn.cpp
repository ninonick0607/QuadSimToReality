// QuadPawn.cpp

#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"

#define EPSILON 0.0001f

AQuadPawn::AQuadPawn()
    : DroneBody(nullptr)
    , DroneCamMesh(nullptr)
    , SpringArm(nullptr)
    , Camera(nullptr)
    , CameraFPV(nullptr)
    , ThrusterFL(nullptr)
    , ThrusterFR(nullptr)
    , ThrusterBL(nullptr)
    , ThrusterBR(nullptr)
    , MotorFL(nullptr)
    , MotorFR(nullptr)
    , MotorBL(nullptr)
    , MotorBR(nullptr)
    , WaypointMode(EWaypointMode::WaitingForModeSelection)
    , NewWaypoint(FVector::ZeroVector)
    , Rotors()
    , QuadController(nullptr)
    , bWaypointModeSelected(false)
    , Input_ToggleImguiInput(nullptr)
{
    PrimaryActorTick.bCanEverTick = true;
    bIsActive = false;

    // Initialize arrays
    Motors.SetNum(4);
    Thrusters.SetNum(4);
    
    // Initialize components
    DroneBody = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneBody"));
    SetRootComponent(DroneBody);
    DroneBody->SetSimulatePhysics(true);

    DroneCamMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CamMesh"));
    DroneCamMesh->AttachToComponent(DroneBody, FAttachmentTransformRules::KeepRelativeTransform);

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->AttachToComponent(DroneBody, FAttachmentTransformRules::KeepRelativeTransform);

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepRelativeTransform);
    
    CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
    CameraFPV->AttachToComponent(DroneBody, FAttachmentTransformRules::KeepWorldTransform);
    CameraFPV->SetRelativeLocation(FVector(5.5, 0,-6));
    CameraFPV->SetRelativeScale3D(FVector(.1));

    InitializeRotors();
    
    // SpringArm settings
    SpringArm->TargetArmLength = 200.0f;
    SpringArm->SetRelativeRotation(FRotator(-20.0f, 0.0f, 0.0f));
    SpringArm->bDoCollisionTest = false;
    SpringArm->bInheritPitch = false;
    SpringArm->bInheritRoll = false;

    // Automatically possess pawn for testing
    AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void AQuadPawn::BeginPlay()
{
    Super::BeginPlay();

    // Initialize the drone controller
    if (!QuadController)
    {
        QuadController = NewObject<UQuadDroneController>(this, TEXT("QuadDroneController"));
        if (QuadController)
        {
            QuadController->Initialize(this);
            bIsActive = true;  // Activate the drone once controller is initialized
        }
    }
}

void AQuadPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!bIsActive)
        return;

    for (auto& rotor : Rotors)
    {
        rotor.Animate(DeltaTime);
    }

    UpdateControl(DeltaTime);
}

void AQuadPawn::SwitchCamera() const 
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
    if (QuadController)
    {
        QuadController->Update(DeltaTime);
    }
}

void AQuadPawn::InitializeRotors()
{
    const FString motorNames[] = {TEXT("MotorFL"), TEXT("MotorFR"), TEXT("MotorBL"), TEXT("MotorBR")};
    const FString thrusterNames[] = {TEXT("ThrusterFL"), TEXT("ThrusterFR"), TEXT("ThrusterBL"), TEXT("ThrusterBR")};
    const FString socketNames[] = {
        TEXT("MotorSocketFL"), TEXT("MotorSocketFR"), TEXT("MotorSocketBL"), TEXT("MotorSocketBR")
    };

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

        Rotors[i].Thruster = Thrusters[i];
        Rotors[i].Mesh = Motors[i];
        Rotors[i].AngularVelocity = 0.f;
    }
}